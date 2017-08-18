/*
 * File: mrpt_icp_slam_live_2d_wrapper.cpp
 * Author: tyu
 * Reference: Vladislav Tananaev(mrpt_icp_slam_2d)
 *
 */

#include "mrpt_icp_slam_live_2d/mrpt_icp_slam_live_2d_wrapper.h"
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x150
#include <mrpt_bridge/utils.h>
#endif

// thread relative
mrpt::hwdrivers::CGenericSensor::TListObservations global_list_obs;
mrpt::synch::CCriticalSection cs_global_list_obs;
bool allThreadsMustExit = false;

void SensorThread(ICPslamLiveWrapper::TThreadParams params) {
	using namespace mrpt::hwdrivers;
	using namespace mrpt::system;
	try {
		string driver_name = params.cfgFile->read_string(params.section_name,"driver","",true);
		CGenericSensorPtr sensor = CGenericSensor::createSensorPtr(driver_name);
		if (!sensor)
			throw std::runtime_error( string("***ERROR***: Class name not recognized: ") + driver_name );

		// Load common & sensor specific parameters:
		sensor->loadConfig( *params.cfgFile, params.section_name );
		cout << format("[thread_%s] Starting...",params.section_name.c_str()) << " at " << sensor->getProcessRate() <<  " Hz" << endl;

		ASSERTMSG_(sensor->getProcessRate()>0,"process_rate must be set to a valid value (>0 Hz).");
		const int process_period_ms = mrpt::utils::round( 1000.0 / sensor->getProcessRate() );
	
		sensor->initialize(); // Init device:
		while (! allThreadsMustExit ) {
			const TTimeStamp t0= now();
			sensor->doProcess();  // Process
			// Get new observations
			CGenericSensor::TListObservations lstObjs;
			sensor->getObservations( lstObjs );
			{
				synch::CCriticalSectionLocker lock (&cs_global_list_obs);
				global_list_obs.insert( lstObjs.begin(), lstObjs.end() );
			}
			lstObjs.clear();
			// wait for the process period:
			TTimeStamp t1= now();
			double At = timeDifference(t0,t1);
			int At_rem_ms = process_period_ms - At*1000;
			if (At_rem_ms>0)
				mrpt::system::sleep(At_rem_ms);
		}
		sensor.clear();
		cout << format("[thread_%s] Closing...",params.section_name.c_str()) << endl;
	}
	catch (std::exception &e) {
		cerr << "[SensorThread]  Closing due to exception:\n" << e.what() << endl;
		allThreadsMustExit = true;
	}
	catch (...) {
		cerr << "[SensorThread] Untyped exception! Closing." << endl;
		allThreadsMustExit = true;
	}
}

ICPslamLiveWrapper::ICPslamLiveWrapper() {
	stamp = ros::Time(0);
	// Default parameters for 3D window
	SHOW_PROGRESS_3D_REAL_TIME = false;
	SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;  // this parameter is not used
	SHOW_LASER_SCANS_3D = true;
	CAMERA_3DSCENE_FOLLOWS_ROBOT = true;

	SAVE_RAWLOG = true;

}

ICPslamLiveWrapper::~ICPslamLiveWrapper() {
	try {
		std::string sOutMap = "mrpt_icpslam_";
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(now(), parts, true);
		sOutMap += format("%04u-%02u-%02u_%02uh%02um%02us",
			(unsigned int)parts.year,
			(unsigned int)parts.month,
			(unsigned int)parts.day,
			(unsigned int)parts.hour,
			(unsigned int)parts.minute,
			(unsigned int)parts.second );
		sOutMap += ".simplemap";

		sOutMap = mrpt::system::fileNameStripInvalidChars( sOutMap );
		ROS_INFO("Saving built map to `%s`", sOutMap.c_str());
		mapBuilder.saveCurrentMapToFile(sOutMap);

		const CMultiMetricMap *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
		const string str = format("%s/finalmap.txt", OUT_DIR_STD.c_str());
		ROS_INFO("SAVE FINAL METRIC MAPS!");
		finalPointsMap->saveMetricMapRepresentationToFile(str);

		if (SHOW_PROGRESS_3D_REAL_TIME && win3D_.present())
		{
			CFileGZOutputStream f(format("%s/buildingmap_final.3Dscene", OUT_DIR_STD.c_str()));
			f << *scene;
		}
		allThreadsMustExit = true;
		mrpt::system::joinThread(hSensorThread);
		ROS_INFO("Sensor thread is closed. Bye-bye!");
	}
	catch (std::exception &e) {
		ROS_ERROR("Exception: %s",e.what());
	}
}

bool ICPslamLiveWrapper::is_file_exists(const std::string &name) {
	std::ifstream f(name.c_str());
	return f.good();
}

void ICPslamLiveWrapper::read_iniFile(std::string ini_filename) {
	CConfigFile iniFile(ini_filename);

	mapBuilder.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile(iniFile, "ICP");
	mapBuilder.initialize();

#if MRPT_VERSION < 0x150
	mapBuilder.options.verbose = true;
#else
	log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	mapBuilder.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
	mapBuilder.logging_enable_console_output = false;
	mapBuilder.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#endif
	mapBuilder.options.alwaysInsertByClass.fromString(
			iniFile.read_string("MappingApplication", "alwaysInsertByClass", ""));

	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();

	// parameters for mrpt3D window
	CAMERA_3DSCENE_FOLLOWS_ROBOT =
		iniFile.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true, /*Force existence:*/ true);
	SAVE_RAWLOG = iniFile.read_bool("MappingApplication", "SAVE_RAWLOG", true, false);
	MRPT_LOAD_CONFIG_VAR(SHOW_PROGRESS_3D_REAL_TIME, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(SHOW_LASER_SCANS_3D, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");

	OUT_DIR_STD = iniFile.read_string("MappingApplication", "logOutput_dir", "log_out", /*Force existence*/ true);
	mrpt::system::deleteFilesInDirectory(OUT_DIR_STD.c_str());
	mrpt::system::createDirectory(OUT_DIR_STD.c_str());
	LOG_FREQUENCY = iniFile.read_int("MappingApplication", "LOG_FREQUENCY", 5, /*Force existence*/ true);

	f_estimated.open(format("%s/log_estimated_path.txt", OUT_DIR_STD.c_str()));

}

void ICPslamLiveWrapper::get_param() {
	ROS_INFO("READ PARAM FROM LAUNCH FILE");
	//  n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
	//  ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

	//  n_.getParam("rawlog_filename", rawlog_filename);
	//  ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

	n_.getParam("ini_filename", ini_filename);
	ROS_INFO("ini_filename: %s", ini_filename.c_str());
	iniFile.setFileName(ini_filename);

	n_.param<std::string>("global_frame_id", global_frame_id, "map");
	ROS_INFO("global_frame_id: %s", global_frame_id.c_str());

	n_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

	n_.param<std::string>("base_frame_id", base_frame_id, "base_link");
	ROS_INFO("base_frame_id: %s", base_frame_id.c_str());

	n_.param<std::string>("sensor_source", sensor_source, "scan");
	ROS_INFO("sensor_source: %s", sensor_source.c_str());

	//mrpt::utils::CConfigFile iniFile(ini_filename);
	params.cfgFile = &iniFile;
	params.section_name = "LIDAR_SENSOR";
}

void ICPslamLiveWrapper::init3Dwindow() {
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME) {
		win3D_ = mrpt::gui::CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
		win3D_->setCameraZoom(20);
		win3D_->setCameraAzimuthDeg(-45);
	}

#endif
}

void ICPslamLiveWrapper::run3Dwindow() {
	// Create 3D window if requested (the code is copied from ../mrpt/apps/icp-slam/icp-slam_main.cpp):
	if (SHOW_PROGRESS_3D_REAL_TIME && win3D_.present()) {
		// get currently builded map
		metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

		lst_current_laser_scans.clear();

		CPose3D robotPose;
		mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);
//		COpenGLScenePtr scene = COpenGLScene::Create();
		scene = COpenGLScene::Create();

		COpenGLViewportPtr view = scene->getViewport("main");

		COpenGLViewportPtr view_map = scene->createViewport("mini-map");
		view_map->setBorderSize(2);
		view_map->setViewportPosition(0.01, 0.01, 0.35, 0.35);
		view_map->setTransparent(false);

		{
			mrpt::opengl::CCamera &cam = view_map->getCamera();
			cam.setAzimuthDegrees(-90);
			cam.setElevationDegrees(90);
			cam.setPointingAt(robotPose);
			cam.setZoomDistance(20);
			cam.setOrthogonal();
		}

		// The ground:
		mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
		groundPlane->setColor(0.4, 0.4, 0.4);
		view->insert(groundPlane);
		view_map->insert(CRenderizablePtr(groundPlane));  // A copy

		// The camera pointing to the current robot pose:
		if (CAMERA_3DSCENE_FOLLOWS_ROBOT) {
			scene->enableFollowCamera(true);

			mrpt::opengl::CCamera &cam = view_map->getCamera();
			cam.setAzimuthDegrees(-45);
			cam.setElevationDegrees(45);
			cam.setPointingAt(robotPose);
		}

		// The maps:
		{
			opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
			metric_map_->getAs3DObject(obj);
			view->insert(obj);

			// Only the point map:
			opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
			if (metric_map_->m_pointsMaps.size()) {
				metric_map_->m_pointsMaps[0]->getAs3DObject(ptsMap);
				view_map->insert(ptsMap);
			}
		}

		// Draw the robot path:
		CPose3DPDFPtr posePDF = mapBuilder.getCurrentPoseEstimation();
		CPose3D curRobotPose;
		posePDF->getMean(curRobotPose);
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
			obj->setPose(curRobotPose);
			view->insert(obj);
		}
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
			obj->setPose(curRobotPose);
			view_map->insert(obj);
		}

		opengl::COpenGLScenePtr &ptrScene = win3D_->get3DSceneAndLock();
		ptrScene = scene;

		win3D_->unlockAccess3DScene();

		// Move camera:
		win3D_->setCameraPointingToPoint(curRobotPose.x(), curRobotPose.y(), curRobotPose.z());

		// Update:
		win3D_->forceRepaint();

		// Build list of scans:
		if (SHOW_LASER_SCANS_3D) {
			if (IS_CLASS(observation, CObservation2DRangeScan)) {
				lst_current_laser_scans.push_back(CObservation2DRangeScanPtr(observation));
			}
		}

		// Draw laser scanners in 3D:
		if (SHOW_LASER_SCANS_3D) {
			for (size_t i = 0; i < lst_current_laser_scans.size(); i++) {
				// Create opengl object and load scan data from the scan observation:
				opengl::CPlanarLaserScanPtr obj = opengl::CPlanarLaserScan::Create();
				obj->setScan(*lst_current_laser_scans[i]);
				obj->setPose(curRobotPose);
				obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);
				// inser into the scene:
				view->insert(obj);
			}
		}
	}
}

void ICPslamLiveWrapper::init() {
	// get parameters from ini file
	if (!is_file_exists(ini_filename)) {
		ROS_ERROR_STREAM("CAN'T READ INI FILE");
		return;
	}

	ROS_INFO("\n\n===== Launching LIDAR grabbing thread ===\n");
	hSensorThread = mrpt::system::createThread(SensorThread, params);
	if (allThreadsMustExit)
		throw std::runtime_error("\n\n==== ABORTING: It seems that we could not connect to the LIDAR. See reported errors. ==== \n");
	read_iniFile(ini_filename);
	// read rawlog file if it  exists
	/*
	 *if (is_file_exists(rawlog_filename))
	 *{
	 *  ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
	 *  rawlog_play_ = true;
	 *}
	 */
	if (SAVE_RAWLOG) {
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
		string rawlog_postfix = "_";
		rawlog_postfix += format("%04u-%02u-%02u_%02uh%02um%o2us",
				(unsigned int) parts.year,
				(unsigned int) parts.month,
				(unsigned int) parts.day,
				(unsigned int) parts.hour,
				(unsigned int) parts.minute,
				(unsigned int) parts.second);
		rawlog_postfix +=string(".rawlog");
		rawlog_postfix = mrpt::system::fileNameStripInvalidChars( rawlog_postfix );
		const string rawlog_filename = "icpslamlive_dataset_" + rawlog_postfix;
		out_rawlog.open(rawlog_filename);
	}

	/// Create publishers///
	// publish grid map
	pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	// publish point map
	pub_point_cloud_ = n_.advertise<sensor_msgs::PointCloud>("PointCloudMap", 1, true);

	// robot pose
	pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);

	// read sensor topics
	/*
	 *std::vector<std::string> lstSources;
	 *mrpt::system::tokenize(sensor_source, " ,\t\n", lstSources);
	 *ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. " "\"scan\" or \"beacon\")");
	 */

	/// Create subscribers///
	/*
	 *sensorSub_.resize(lstSources.size());
	 *for (size_t i = 0; i < lstSources.size(); i++) {
	 *    if (lstSources[i].find("scan") != std::string::npos) {
	 *        sensorSub_[i] = n_.subscribe(lstSources[i], 1, &ICPslamLiveWrapper::laserCallback, this);
	 *    } else {
	 *        std::cout << "Sensor topics should be 2d laser scans which inlude in the name the word scan " << "\n";
	 *    }
	 *}
	 */

	init3Dwindow();
}

#if 0
void ICPslamLiveWrapper::laserCallback(const sensor_msgs::LaserScan &_msg) {
#if MRPT_VERSION >= 0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif
	CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();
	if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
		updateSensorPose(_msg.header.frame_id);
	} else {
		mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
		mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id], *laser);
		// CObservationPtr obs = CObservationPtr(laser);
		observation = CObservationPtr(laser);
		stamp = ros::Time(0);
		tictac.Tic();
		mapBuilder.processObservation(observation);
		t_exec = tictac.Tac();
		ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

		run3Dwindow();
		publishTF();
		publishMapPose();
	}
}

void ICPslamLiveWrapper::publishMapPose() {
	// get currently builded map
	metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

	// publish map
	if (metric_map_->m_gridMaps.size()) {
		nav_msgs::OccupancyGrid _msg;
		// if we have new map for current sensor update it
		mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
		pub_map_.publish(_msg);
		pub_metadata_.publish(_msg.info);
	}

	if (metric_map_->m_pointsMaps.size()) {
		sensor_msgs::PointCloud _msg;
		std_msgs::Header header;
		header.stamp = ros::Time(0);
		header.frame_id = global_frame_id;
		// if we have new map for current sensor update it
		mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0], header, _msg);
		pub_point_cloud_.publish(_msg);
	}

	CPose3D robotPose;
	mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

	// publish pose
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = global_frame_id;

	// the pose
	pose.pose.position.x = robotPose.x();
	pose.pose.position.y = robotPose.y();
	pose.pose.position.z = 0.0;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());

	pub_pose_.publish(pose);
}

void ICPslamLiveWrapper::updateSensorPose(std::string _frame_id) {
	CPose3D pose;
	tf::StampedTransform transform;
	try {
		listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);

		tf::Vector3 translation = transform.getOrigin();
		tf::Quaternion quat = transform.getRotation();
		pose.x() = translation.x();
		pose.y() = translation.y();
		pose.z() = translation.z();
		double roll, pitch, yaw;
		tf::Matrix3x3 Rsrc(quat);
		CMatrixDouble33 Rdes;
		for (int c = 0; c < 3; c++)
			for (int r = 0; r < 3; r++)
				Rdes(r, c) = Rsrc.getRow(r)[c];
		pose.setRotationMatrix(Rdes);
		laser_poses_[_frame_id] = pose;
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
}

void ICPslamLiveWrapper::publishTF() {
	// Most of this code was copy and pase from ros::amcl
	mrpt::poses::CPose3D robotPoseTF;
	mapBuilder.getCurrentPoseEstimation()->getMean(robotPoseTF);

	stamp = ros::Time(0);
	tf::Stamped<tf::Pose> odom_to_map;
	tf::Transform tmp_tf;
	mrpt_bridge::convert(robotPoseTF, tmp_tf);

	try {
		tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id);
		listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
	}
	catch (tf::TransformException) {
		ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
		return;
	}

	tf::Transform latest_tf_ =
		tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

	tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), stamp, global_frame_id, odom_frame_id);

	tf_broadcaster_.sendTransform(tmp_tf_stamped);
}
#endif


bool ICPslamLiveWrapper::run() {
	CTicTac timeout_read_scans;
	while (!allThreadsMustExit) {
		if (ros::ok()) {
			if (mrpt::system::os::kbhit()) {
				const char c = mrpt::system::os::getch();
				if (27 == c)
					break;
			}
			if (win3D_ && !win3D_->isOpen()) break;

			//CObservation2DRangeScanPtr temp;
			bool update = false;
			{
				mrpt::hwdrivers::CGenericSensor::TListObservations obs_copy;
				{
					mrpt::synch::CCriticalSectionLocker csl(&cs_global_list_obs);
					obs_copy = global_list_obs;
					global_list_obs.clear();
				}
				for (mrpt::hwdrivers::CGenericSensor::TListObservations::reverse_iterator it = obs_copy.rbegin(); it != obs_copy.rend(); ++it)
					if (it->second.present() && IS_CLASS(it->second,CObservation2DRangeScan))
					{
						ROS_INFO("observation update");
						observation = CObservation2DRangeScanPtr(it->second);
						update = true;
					}

				if (out_rawlog.fileOpenCorrectly()) {
					for (mrpt::hwdrivers::CGenericSensor::TListObservations::iterator it = obs_copy.begin(); it!= obs_copy.end(); ++it)
						if (it->second.present() && IS_CLASS(it->second, CObservation2DRangeScan))
							out_rawlog << *it->second;
				}
			}
			if (!update) {
				if (timeout_read_scans.Tac() > 1.0) {
					timeout_read_scans.Tic();
					ROS_INFO("Waiting for laser scans");
				}
				mrpt::system::sleep(1);
				continue;
			} else { timeout_read_scans.Tic(); }

			mapBuilder.processObservation(observation);
			ROS_INFO("Map building executed");
			/*
			 *mrpt::system::TTimeParts parts;
			 *mrpt::system::timestampToParts(observation->timestamp, parts, true);
			 *ROS_INFO("observation timestamp: %ld --> %04u-%02u-%02u:%02uh%02um%02fs",
			 *         observation->timestamp,
			 *         (unsigned int)parts.year,
			 *         (unsigned int)parts.month,
			 *         (unsigned int)parts.day,
			 *         (unsigned int)parts.hour,
			 *         (unsigned int)parts.minute,
			 *         parts.second);
			 */
			metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

			CPose3D robotPose;
			mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

			if (metric_map_->m_gridMaps.size()) {
				nav_msgs::OccupancyGrid _msg;
				mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
				pub_map_.publish(_msg);
				pub_metadata_.publish(_msg.info);
			}

			if (metric_map_->m_pointsMaps.size()) {
				sensor_msgs::PointCloud _msg;
				std_msgs::Header header;
				header.stamp = ros::Time(0);
				header.frame_id = global_frame_id;
				mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0], header, _msg);
				pub_point_cloud_.publish(_msg);
			}

			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = global_frame_id;
			pose.pose.position.x = robotPose.x();
			pose.pose.position.y = robotPose.y();
			pose.pose.position.z = 0.0;
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());
			pub_pose_.publish(pose);

		}
		run3Dwindow();

		f_estimated.printf("%f %f %f\n",
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().x(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().y(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().yaw());
	}
	return true;
}
