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


	if (!slam.run()) {

		while (ros::ok()) {
			ros::spinOnce();
			r.sleep();
		}
	}
}
