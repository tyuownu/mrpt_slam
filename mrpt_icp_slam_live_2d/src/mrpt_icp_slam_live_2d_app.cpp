/**
  * @file mrpt_icp_slam_live_2d_app.cpp
  *
  *
  * @author tyuownu@gmail.com
  *
  */
#include "mrpt_icp_slam_live_2d/mrpt_icp_slam_live_2d_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrpt_icp_slam_live_2d");
  ros::NodeHandle n;
  ros::Rate r(100);
  ICPslamLiveWrapper slam;
  slam.get_param();
  slam.init();
  ros::Duration(3).sleep();

  while (ros::ok()) {
    if (mrpt::system::os::kbhit()) {
      // press 'ESC' to stop the slam
      const char c = mrpt::system::os::getch();
      if (27 == c) break;
    }
    ros::spinOnce();
  }
  ros::Duration(3).sleep();
  return 0;
}
