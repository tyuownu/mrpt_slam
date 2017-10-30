/**
  * @file mrpt_icp_slam_live_2d_wrapper.cpp
  * @brief realize live mrpt icpslam, reference to mrpt_icp_slam_2d.
  *
  * @author tyuownu@gmail.com
  *
  */

#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
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
    string driver_name = params.cfgFile->read_string(params.section_name,
            "driver", "", true);
    CGenericSensorPtr sensor = CGenericSensor::createSensorPtr(driver_name);
    if ( !sensor )
      throw std::runtime_error(
              string("***ERROR***: Class name not recognized: ")
              + driver_name);

    // Load common & sensor specific parameters:
    sensor->loadConfig(*params.cfgFile, params.section_name);
    cout << format("[thread_%s] Starting...", params.section_name.c_str())
        << " at " << sensor->getProcessRate() <<  " Hz" << endl;

    ASSERTMSG_(sensor->getProcessRate() > 0,
            "process_rate must be set to a valid value (>0 Hz).");
    const int process_period_ms =
        mrpt::utils::round(1000.0 / sensor->getProcessRate());

    sensor->initialize();  // Init device:
    while ( !allThreadsMustExit ) {
      const TTimeStamp t0 = now();
      sensor->doProcess();  // Process
      // Get new observations
      CGenericSensor::TListObservations lstObjs;
      sensor->getObservations(lstObjs);
      {
        synch::CCriticalSectionLocker lock(&cs_global_list_obs);
        global_list_obs.insert(lstObjs.begin(), lstObjs.end());
      }
      lstObjs.clear();
      // wait for the process period:
      TTimeStamp t1 = now();
      double At = timeDifference(t0, t1);
      int At_rem_ms = process_period_ms - At*1000;
      if ( At_rem_ms > 0 )
        mrpt::system::sleep(At_rem_ms);
    }
    sensor.clear();
    cout << format("[thread_%s] Closing...", params.section_name.c_str())
        << endl;
  }
  catch ( std::exception &e ) {
    cerr << "[SensorThread]  Closing due to exception:\n" << e.what() << endl;
    allThreadsMustExit = true;
  }
  catch ( ... ) {
    cerr << "[SensorThread] Untyped exception! Closing." << endl;
    allThreadsMustExit = true;
  }
}

ICPslamLiveWrapper::ICPslamLiveWrapper() {
  stamp = ros::Time(0);
  // Default parameters for 3D window
  show_progress_3d_real_time_ = false;
  show_progress_3d_real_time_delay_ms_ = 0;  // this parameter is not used
  show_laser_scans_3d_ = true;
  camera_3dscene_follows_robot_ = true;

  save_rawlog_ = true;
  b_first_odom_ = true;
  output_rawlog_ = new CRawlog;
}

CPose3D ICPslamLiveWrapper::laser_base_pose_ = CPose3D(0, 0, 0, 0, 0, 0);

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
      (unsigned int)parts.second);
    sOutMap += ".simplemap";

    sOutMap = mrpt::system::fileNameStripInvalidChars(sOutMap);
    ROS_INFO("Saving built map to `%s`", sOutMap.c_str());
    mapBuilder_.saveCurrentMapToFile(sOutMap);

    const CMultiMetricMap *finalPointsMap
        = mapBuilder_.getCurrentlyBuiltMetricMap();
    const string str = format("%s/finalmap.txt", log_out_dir_.c_str());
    ROS_INFO("SAVE FINAL METRIC MAPS!");
    finalPointsMap->saveMetricMapRepresentationToFile(str);

    if ( show_progress_3d_real_time_ && win3D_.present() ) {
      CFileGZOutputStream f(format("%s/buildingmap_final.3Dscene",
                  log_out_dir_.c_str()));
      f << *scene_;
    }
    allThreadsMustExit = true;
    // mrpt::system::joinThread(hSensorThread);
    ROS_INFO("Sensor thread is closed. Bye-bye!");

    if ( output_rawlog_->size() > 0 ) {
      std::string filename = "output_rawlog_.rawlog";
      output_rawlog_->saveToRawLogFile(filename);
    }
    delete output_rawlog_;
  }
  catch ( std::exception &e ) {
    ROS_ERROR("Exception: %s", e.what());
  }
}

bool ICPslamLiveWrapper::is_file_exists(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
}

void ICPslamLiveWrapper::read_iniFile() {
  // CConfigFile iniFile(ini_filename_);

  mapBuilder_.ICP_options.loadFromConfigFile(ini_file_, "MappingApplication");
  mapBuilder_.ICP_params.loadFromConfigFile(ini_file_, "ICP");
  mapBuilder_.initialize();

#if MRPT_VERSION < 0x150
  mapBuilder_.options.verbose = true;
#else
  log4cxx::LoggerPtr ros_logger =
      log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapBuilder_.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(
              ros_logger->getLevel()));
  mapBuilder_.logging_enable_console_output = false;
  mapBuilder_.logRegisterCallback(static_cast<output_logger_callback_t>(
              &mrpt_bridge::mrptToROSLoggerCallback));
#endif
  mapBuilder_.options.alwaysInsertByClass.fromString(
      ini_file_.read_string("MappingApplication", "alwaysInsertByClass", ""));

  mapBuilder_.ICP_params.dumpToConsole();
  mapBuilder_.ICP_options.dumpToConsole();

  // parameters for mrpt3D window
  camera_3dscene_follows_robot_ =
    ini_file_.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT",
            true, /*Force existence:*/ true);
  save_rawlog_ = ini_file_.read_bool("MappingApplication", "save_rawlog_",
          true, false);
  MRPT_LOAD_CONFIG_VAR(show_progress_3d_real_time_, bool,
          ini_file_, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(show_laser_scans_3d_, bool,
          ini_file_, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(show_progress_3d_real_time_delay_ms_, int,
          ini_file_, "MappingApplication");

  log_out_dir_ = ini_file_.read_string("MappingApplication", "logOutput_dir",
          "log_out", /*Force existence*/ true);
  mrpt::system::deleteFilesInDirectory(log_out_dir_.c_str());
  mrpt::system::createDirectory(log_out_dir_.c_str());
  log_frequency_ = ini_file_.read_int("MappingApplication", "LOG_FREQUENCY",
          5, /*Force existence*/ true);

  f_estimated_.open(format("%s/log_estimated_path.txt", log_out_dir_.c_str()));
}

void ICPslamLiveWrapper::get_param() {
  ROS_INFO("READ PARAM FROM LAUNCH FILE");
  //  n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
  //  ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

  //  n_.getParam("rawlog_filename", rawlog_filename);
  //  ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

  n_.getParam("ini_filename", ini_filename_);
  ROS_INFO("ini_filename: %s", ini_filename_.c_str());
  ini_file_.setFileName(ini_filename_);

  n_.param<std::string>("global_frame_id", global_frame_id_, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id_.c_str());

  n_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id_.c_str());

  n_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id_.c_str());

  n_.param<std::string>("sensor_source", sensor_source_, "scan");
  ROS_INFO("sensor_source: %s", sensor_source_.c_str());

  n_.param("trajectory_update_rate", trajectory_update_rate_, 10.0);
  ROS_INFO("trajectory_update_rate: %f", trajectory_update_rate_);

  n_.param("trajectory_publish_rate", trajectory_publish_rate_, 5.0);
  ROS_INFO("trajectory_publish_rate: %f", trajectory_publish_rate_);

  n_.param("using_odometry", using_odometry_, true);
  ROS_INFO_STREAM("using_odometry: " << using_odometry_ ? "yes" : "no");

  // mrpt::utils::CConfigFile iniFile(ini_filename_);
  params_.cfgFile = &ini_file_;
  params_.section_name = "LIDAR_SENSOR";
}

void ICPslamLiveWrapper::init3Dwindow() {
#if MRPT_HAS_WXWIDGETS
  if ( show_progress_3d_real_time_ ) {
    win3D_ = mrpt::gui::CDisplayWindow3D::Create(
            "pf-localization - The MRPT project", 1000, 600);
    win3D_->setCameraZoom(20);
    win3D_->setCameraAzimuthDeg(-45);
  }

#endif
}

void ICPslamLiveWrapper::run3Dwindow() {
  // Create 3D window if requested (the code is copied from
  // ../mrpt/apps/icp-slam/icp-slam_main.cpp):
  if ( show_progress_3d_real_time_ && win3D_.present() ) {
    // get currently builded map
    metric_map_ = mapBuilder_.getCurrentlyBuiltMetricMap();

    lst_current_laser_scans_.clear();

    CPose3D robotPose;
    mapBuilder_.getCurrentPoseEstimation()->getMean(robotPose);
//    COpenGLScenePtr scene = COpenGLScene::Create();
    scene_ = COpenGLScene::Create();

    COpenGLViewportPtr view = scene_->getViewport("main");

    COpenGLViewportPtr view_map = scene_->createViewport("mini-map");
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
    mrpt::opengl::CGridPlaneXYPtr groundPlane =
        mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
    groundPlane->setColor(0.4, 0.4, 0.4);
    view->insert(groundPlane);
    view_map->insert(CRenderizablePtr(groundPlane));  // A copy

    // The camera pointing to the current robot pose:
    if ( camera_3dscene_follows_robot_ ) {
      scene_->enableFollowCamera(true);

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
      if ( metric_map_->m_pointsMaps.size() ) {
        metric_map_->m_pointsMaps[0]->getAs3DObject(ptsMap);
        view_map->insert(ptsMap);
      }
    }

    // Draw the robot path:
    CPose3DPDFPtr posePDF = mapBuilder_.getCurrentPoseEstimation();
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
    ptrScene = scene_;

    win3D_->unlockAccess3DScene();

    // Move camera:
    win3D_->setCameraPointingToPoint(
            curRobotPose.x(),
            curRobotPose.y(),
            curRobotPose.z());

    // Update:
    win3D_->forceRepaint();

    // Build list of scans:
    if (show_laser_scans_3d_) {
      if (IS_CLASS(observation_, CObservation2DRangeScan)) {
        lst_current_laser_scans_.push_back(
                CObservation2DRangeScanPtr(observation_));
      }
    }

    // Draw laser scanners in 3D:
    if ( show_laser_scans_3d_ ) {
      for ( size_t i = 0; i < lst_current_laser_scans_.size(); i++ ) {
        // Create opengl object and load scan data from the scan observation:
        opengl::CPlanarLaserScanPtr obj = opengl::CPlanarLaserScan::Create();
        obj->setScan(*lst_current_laser_scans_[i]);
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
  if ( !is_file_exists(ini_filename_) ) {
    ROS_ERROR_STREAM("CAN'T READ INI FILE");
    return;
  }

  ROS_INFO("\n\n===== Launching LIDAR grabbing thread ===\n");
  // hSensorThread = mrpt::system::createThread(SensorThread, params);
  if ( allThreadsMustExit )
    throw std::runtime_error(
        "\n== ABORTING: could not connect to LIDAR. See reported errors. ==\n");
  read_iniFile();
  // read rawlog file if it  exists
  /*
   *if (is_file_exists(rawlog_filename))
   *{
   *  ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
   *  rawlog_play_ = true;
   *}
   */

  // Create publishers
  // publish grid map
  pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>(global_frame_id_, 1, true);
  pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  // publish point map
  pub_point_cloud_ =
      n_.advertise<sensor_msgs::PointCloud>("PointCloudMap", 1, true);


  // robot pose
  pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  // publish robot trajectory
  trajectory_pub_ = n_.advertise<nav_msgs::Path>("trajectory", 1, true);

  update_trajector_timer_ =
      n_.createTimer(ros::Duration(1.0 / trajectory_update_rate_),
              &ICPslamLiveWrapper::updateTrajectoryTimerCallback, this, false);

  publish_trajectory_timer_ =
      n_.createTimer(ros::Duration(1.0 / trajectory_publish_rate_),
      &ICPslamLiveWrapper::publishTrajectoryTimerCallback, this, false);

  laser_sub_ =
      n_.subscribe(sensor_source_, 1, &ICPslamLiveWrapper::laserCallback, this);
  if ( using_odometry_ )
    odom_sub_ =
        n_.subscribe(odom_frame_id_, 5,
                &ICPslamLiveWrapper::odometryCallback, this);

  init3Dwindow();
}

void ICPslamLiveWrapper::odometryCallback(const nav_msgs::Odometry& odom) {
  if ( b_first_odom_ ) {
    cur_odom_ = odom;
    last_odom_ = odom;
    b_first_odom_ = false;
  } else {
    cur_odom_ = odom;
  }
}

void ICPslamLiveWrapper::laserCallback(const sensor_msgs::LaserScan &_msg) {
  // CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();
  CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();

  // convert to mrpt lidar scan
  mrpt_bridge::convert(_msg, laser_base_pose_, *laser);
  if ( using_odometry_ ) {
    CActionCollectionPtr  action = CActionCollection::Create();
    convertOdometry(action);

    last_odom_ = cur_odom_;
    CSensoryFramePtr SF = CSensoryFrame::Create();
    SF->insert(laser);

    // save the rawlog?
    if ( save_rawlog_ ) {
      output_rawlog_->addObservationsMemoryReference(SF);
      output_rawlog_->addActionsMemoryReference(action);
    }
    mapBuilder_.processActionObservation(*action, *SF);
  } else {
    mapBuilder_.processObservation(CObservationPtr(laser));
  }

  metric_map_ = mapBuilder_.getCurrentlyBuiltMetricMap();

  CPose3D robotPose;
  mapBuilder_.getCurrentPoseEstimation()->getMean(robotPose);

  if ( metric_map_->m_gridMaps.size() ) {
    nav_msgs::OccupancyGrid _msg;
    mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
    pub_map_.publish(_msg);
    pub_metadata_.publish(_msg.info);
  }

  if ( metric_map_->m_pointsMaps.size() ) {
    sensor_msgs::PointCloud _msg;
    std_msgs::Header header;
    header.stamp = ros::Time(0);
    header.frame_id = global_frame_id_;
    mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0],
            header, _msg);
    pub_point_cloud_.publish(_msg);
  }

  // geometry_msgs::PoseStamped pose;
  pose_.header.frame_id = global_frame_id_;
  pose_.pose.position.x = robotPose.x();
  pose_.pose.position.y = robotPose.y();
  pose_.pose.position.z = 0.0;
  pose_.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());
  pub_pose_.publish(pose_);

  f_estimated_.printf("%f %f %f\n",
      mapBuilder_.getCurrentPoseEstimation()->getMeanVal().x(),
      mapBuilder_.getCurrentPoseEstimation()->getMeanVal().y(),
      mapBuilder_.getCurrentPoseEstimation()->getMeanVal().yaw());

  run3Dwindow();
}

void ICPslamLiveWrapper::updateTrajectoryTimerCallback(
        const ros::TimerEvent& event) {
  // ROS_DEBUG("update trajectory");
  path_.header.frame_id = global_frame_id_;
  path_.header.stamp = ros::Time(0);
  path_.poses.push_back(pose_);
}

void ICPslamLiveWrapper::publishTrajectoryTimerCallback(
        const ros::TimerEvent& event) {
  // ROS_DEBUG("publish trajectory");
  trajectory_pub_.publish(path_);
}

void ICPslamLiveWrapper::convertOdometry(CActionCollectionPtr action) const {
  CActionRobotMovement2D temp_action;
  double x = cur_odom_.pose.pose.position.x - last_odom_.pose.pose.position.x;
  double y = cur_odom_.pose.pose.position.y - last_odom_.pose.pose.position.y;
  tf::Quaternion cur_quat, last_quat;
  tf::quaternionMsgToTF(cur_odom_.pose.pose.orientation, cur_quat);
  tf::quaternionMsgToTF(last_odom_.pose.pose.orientation, last_quat);

  double cur_roll, cur_pitch, cur_yaw;
  double last_roll, last_pitch, last_yaw;

  tf::Matrix3x3(cur_quat).getRPY(cur_roll, cur_pitch, cur_yaw);
  tf::Matrix3x3(last_quat).getRPY(last_roll, last_pitch, last_yaw);
  // ROS_INFO("x: %f, y: %f, roll: %f, pitch: %f, yaw: %f",
  // x, y, cur_roll, cur_pitch, cur_yaw);
  // ROS_INFO("x: %f, y: %f, roll: %f, pitch: %f, yaw: %f",
  // x, y, last_roll, last_pitch, last_yaw);

  CActionRobotMovement2D::TMotionModelOptions options;
  if ( isNaN(cur_yaw) || isNaN(last_yaw) ) {
    temp_action.computeFromOdometry(CPose2D(0, 0, 0), options);
  } else {
    static double angle = last_yaw;
    double yaw = cur_yaw - last_yaw;
    double d_x =  x * cos(angle) + y * sin(angle);
    double d_y = -x * sin(angle) + y * cos(angle);
    // ROS_INFO("angle: %f, dx: %f, dy: %f, yaw: %f", angle, d_x, d_y, yaw);
    angle += yaw;
    temp_action.computeFromOdometry(CPose2D(d_x, d_y, yaw), options);
  }
  // mrpt::system::TTimeStamp cur_time;
  mrpt_bridge::convert(cur_odom_.header.stamp, temp_action.timestamp);
  action->insert(temp_action);
}
