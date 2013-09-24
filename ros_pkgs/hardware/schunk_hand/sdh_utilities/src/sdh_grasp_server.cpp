#include "ros/ros.h"
#include "sdh_utilities/Grasp.h"


bool grasp(sdh_utilities::Grasp::Request& req, sdh_utilities::Grasp::Response& res) {
	return false;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "sdh_grasp_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("grasp", grasp);
  ROS_INFO("Ready to grasp.");
  ros::spin();

  return 0;
}
