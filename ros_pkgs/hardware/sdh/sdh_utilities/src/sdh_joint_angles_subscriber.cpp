#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <sstream>
#include <string>
#include <vector>


void jointAnglesCallback(const sensor_msgs::JointState::ConstPtr& state) {
	std::stringstream names;
	std::stringstream positions;
	std::stringstream velocities;
	std::stringstream efforts;

	for ( int i = 0; i < state->name.size(); i++) {
	    names << "[" << state->name[i] << "]";	    
        }

	for ( int i = 0; i < state->position.size(); i++) {
	    positions << "[" << state->position[i] << "]";
        }

	for ( int i = 0; i < state->velocity.size(); i++) {
	    velocities << "[" << state->velocity[i] << "]";
        }

	for ( int i = 0; i < state->effort.size(); i++) {
            efforts << "[" << state->effort[i] << "]";
        }

	ROS_INFO("***NEW STATE***");	
	ROS_INFO("Names: %s", names.str().c_str());
	ROS_INFO("Positions: %s", positions.str().c_str());
	ROS_INFO("Velocities: %s", velocities.str().c_str());
	ROS_INFO("Efforts: %s", efforts.str().c_str());
	ROS_INFO("");
}


int subscribeJointAngles(int argc, char** argv) {
	ros::init(argc, argv, "sdh_joint_states_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointAnglesCallback);
  	ros::spin();
  	return 0;
}


int main (int argc, char** argv) {
	subscribeJointAngles(argc, argv);	
}
