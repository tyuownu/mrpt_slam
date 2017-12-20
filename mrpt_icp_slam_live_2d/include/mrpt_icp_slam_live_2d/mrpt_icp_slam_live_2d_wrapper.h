/**
  * @file mrpt_icp_slam_live_2d_wrapper.h
  * @brief the main class of live mrpt icpslam, reference to mrpt_icp_slam_2d.
  *
  * @author tyuownu@gmail.com
  *
  */

#ifndef MRPT_ICP_SLAM_LIVE_2D_WRAPPER_H
#define MRPT_ICP_SLAM_LIVE_2D_WRAPPER_H

// MRPT libraries
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>

// This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/gui/CDisplayWindow3D.h>

#include <stdint.h>
#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <string>

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/point_cloud.h>
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CRawlog.h>
using namespace mrpt::maps;
using namespace mrpt::obs;
#else
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CRawlog.h>
#endif
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

/**
 * @brief The ICPslamLiveWrapper provides 2d icp based SLAM from MRPT libraries.
 *
 */
class ICPslamLiveWrapper {
 public:
  /**
   *  @brief Thread relative parameter
   */
  struct TThreadParams {
    mrpt::utils::CConfigFile *cfgFile;
    string section_name;
  };

  // switch control the map output
  /**
   * @brief control the output map type
   * status:
   *   OCCUPANCYGRID_MAP
   *       output map isoccupancy grid map [0-100]
   *   NAVIGATION_MAP
   *       output map using for navigation
   *                 -1->UNKNOWN
   *                  0->FREE
   *                100->OBSTACLE
   */
  enum OutputMapType {
    OCCUPANCYGRID_MAP,
    NAVIGATION_MAP
  };

  /**
   * @brief constructor
   */
  ICPslamLiveWrapper();

  /**
  * @brief destructor
  */
  ~ICPslamLiveWrapper();

  /**
   * @brief read ini file
   *
   */
  void read_iniFile();

  /**
   * @brief init 3D window from mrpt lib
   */
  void init3Dwindow();

  /**
   * @brief run 3D window update from mrpt lib
   */
  void run3Dwindow();

  /**
   * @brief read the parameters from launch file
   */
  void get_param();

  /**
   * @brief initialize publishers subscribers and icp slam
   */
  void init();

  /**
   * @brief check the existance of the file
   *
   * @return true if file exists
   */
  bool is_file_exists(const std::string &name);

  /**
   * @brief callback function for the laser scans
   *
   * Given the laser scans,
   * implement one SLAM update,
   * publish map and pose.
   *
   * @param _msg  the laser scan message
   */
  void laserCallback(const sensor_msgs::LaserScan &_msg);

  /**
  * @brief publish point and/or grid map and robot pose
  *
  */
  // void publishMapPose();

  /**
    * @brief  update the pose of the sensor with respect to the robot
    *
    *@param frame_id the frame of the sensors
    */
  // void updateSensorPose(std::string _frame_id);

  /**
    * @brief  the trajectory update timer callback function
    *
    * @param event timer event
    */
  void updateTrajectoryTimerCallback(const ros::TimerEvent& event);

  /**
    * @brief  the trajectory publish timer callback function
    *
    * @param event timer event
    */
  void publishTrajectoryTimerCallback(const ros::TimerEvent& event);

  /**
    * @brief  odometry callback function
    *
    * @param event timer event
    */
  void odometryCallback(const nav_msgs::Odometry& odom);

  /**
    * @brief  convert ROS odom to mrpt CActionCollection
    *
    * @param action fill the action collection using ROS odom
    */
  void convertOdometry(CActionCollectionPtr action) const;

  /**
   * @brief Provide GetMap service
   *
   * @param req GetMap::Request
   * @param res GetMap::Response
   *
   * @return true
   */
  bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

  /**
   * @brief publish tf between map->odom
   */
  void publishTF(const ros::TimerEvent& timer);

 protected:
  /// icp slam class
  CMetricMapBuilderICP mapBuilder_;
  /// Node Handle
  ros::NodeHandle n_;

  /// name of ini file
  std::string ini_filename_;
  /// /map frame
  std::string global_frame_id_;
  /// /odom frame
  std::string odom_frame_id_;
  /// robot frame
  std::string base_frame_id_;
  /// robot pose
  geometry_msgs::PoseStamped pose_;

  /// trajectory publisher
  ros::Publisher trajectory_pub_;
  /// trajectory path
  nav_msgs::Path path_;

  // trajectory relative
  /// trajectory update timer
  ros::Timer update_trajector_timer_;
  /// trajectory publish timer
  ros::Timer publish_trajectory_timer_;
  /// trajectory update frequency(Hz)
  double trajectory_update_rate_;
  /// trajectory publish frequency(Hz)
  double trajectory_publish_rate_;

  // Sensor source
  /// 2D laser scans topic name
  std::string sensor_source_;
  /// laser scan poses with respect to the map
  std::map<std::string, mrpt::poses::CPose3D> laser_poses_;

  // Subscribers
  // /// list of sensors topics
  // std::vector<ros::Subscriber> sensorSub_;

  /// receive map after iteration of SLAM to metric map
  const CMultiMetricMap *metric_map_;
  // /// current robot pose
  // CPose3DPDFPtr curPDF;
  /// publishers for map, pointcloude & pose particles
  ros::Publisher pub_map_, pub_metadata_,
    pub_pose_, pub_point_cloud_;

  /// getmap_msg_ publisher
  ros::Publisher pub_getmap_;

  /// transform listener
  tf::TransformListener listenerTF_;
  /// transform broadcaster
  tf::TransformBroadcaster tf_broadcaster_;

  // /// timer for SLAM performance evaluation
  // CTicTac tictac;
  // /// the time which take one SLAM update execution
  // float t_exec;
  /// CSensoryFramePtr observations;
  CObservationPtr observation_;
  /// last update of the pose and map
  mrpt::system::TTimeStamp timeLastUpdate_;

  /// MRPT window
  mrpt::gui::CDisplayWindow3DPtr win3D_;

  /// for drawing in 3D views
  std::vector<CObservation2DRangeScanPtr> lst_current_laser_scans_;
  // bool isObsBasedRawlog;

  // GUI show relative parameter
  bool show_progress_3d_real_time_;
  int show_progress_3d_real_time_delay_ms_;
  bool show_laser_scans_3d_;
  bool camera_3dscene_follows_robot_;

  // rawlog relative
  /// save or not (switch)
  bool save_rawlog_;

  // thread relative
  /// thread parameter
  struct TThreadParams params_;
  // /// thread handle
  // mrpt::system::TThreadHandle hSensorThread;

  // configure file
  /// ini config file
  mrpt::utils::CConfigFile ini_file_;

  // log relative
  /// log output directory
  string log_out_dir_;
  /// log output frequency
  int log_frequency_;
  /// estimated pose output file name
  CFileOutputStream f_estimated_;

  /// OpenGL scene
  COpenGLScenePtr scene_;

  nav_msgs::Odometry  cur_odom_, last_odom_;
  /// if the first odom?
  bool b_first_odom_;
  /// odometry subscriber
  ros::Subscriber odom_sub_;
  /// laser scan subscriber
  ros::Subscriber laser_sub_;

  /// output CRawlog file
  CRawlog *output_rawlog_;

  // pose between laser and base;
  /// the pose between laser with base(robot)
  static CPose3D laser_base_pose_;
  /// the system using odometry?
  bool using_odometry_;

  // getMap server and message for nav2d navigation
  /// provide 'static_map' server
  ros::ServiceServer map_server_;
  /// provide map for global map, for save obstacle in map
  nav_msgs::OccupancyGrid getmap_msg_;

  /// output map type
  enum OutputMapType output_map_type_;

  /// tf publish timer
  ros::Timer tf_publish_timer_;
  /// tf publish frequency
  double tf_publish_rate_;
};

#endif /* MRPT_ICP_SLAM_LIVE_2D_WRAPPER_H */
