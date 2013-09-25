#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>
#include <vector>


int publishJointAngles(int argc, char** argv, char* positions) {
	ros::init(argc, argv, "sdh_joint_angles_publisher");
	ros::NodeHandle n;
	ros::Publisher sdh_joint_angles_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/sdh_controller/follow_joint_trajectory/goal", 1);
	usleep(1000*1000);
	control_msgs::FollowJointTrajectoryActionGoal actionGoal;
	control_msgs::FollowJointTrajectoryGoal& goal = actionGoal.goal;
	trajectory_msgs::JointTrajectory& traj = goal.trajectory;
	traj.joint_names  = { "sdh_knuckle_joint", 
			      "sdh_thumb_2_joint", 
			      "sdh_thumb_3_joint", 
		              "sdh_finger_12_joint", 
			      "sdh_finger_13_joint", 
			      "sdh_finger_22_joint", 
			      "sdh_finger_23_joint" };

	trajectory_msgs::JointTrajectoryPoint p;
	const char* positions_[7];
	positions_[0] = strtok (positions, ",");
	for(int i = 1; i < 7; i++) {
		positions_[i] = strtok (NULL, ",");	
	}

	p.positions = { atof(positions_[0]),
			atof(positions_[1]), 
			atof(positions_[2]), 
			atof(positions_[3]), 
			atof(positions_[4]), 
			atof(positions_[5]), 
			atof(positions_[6]) };

	traj.points = {p};
	sdh_joint_angles_pub.publish(actionGoal);
	ROS_INFO("FollowJointTrajectoryActionGoal published");
	usleep(1000*1000);
	sdh_joint_angles_pub.shutdown();
	return 0;	
}


int main (int argc, char** argv) {
	if(argc != 2) {
		ROS_ERROR("Please provide a list with 7 positions");
		ROS_INFO("Usage: rosrun sdh_utilities sdh_joint_angles_publisher \"positions\"; use \",\" as delimiter for positons");
	return -1;
	}
	char* positions = argv[1];
	argv[0][0] ='\0';	
	publishJointAngles(argc, argv, positions);	
}
