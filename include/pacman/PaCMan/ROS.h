/** @file ROS.h
 *
 * PaCMan ROS conversions
 *
 */

#pragma once
#ifndef _PACMAN_PACMAN_ROS_H_ // if #pragma once is not supported
#define _PACMAN_PACMAN_ROS_H_

// our pacman definitions
#include <pacman/PaCMan/Defs.h>

// all generated headers from services we have in pacman ros_pkgs
#include <ros/ros.h>
#include "definitions/GraspPlanning.h"
#include "definitions/KinectGrabberService.h"
#include "definitions/ObjectCloudReader.h"
#include "definitions/PoseEstimation.h"
#include "definitions/TrajectoryExecution.h"
#include "definitions/TrajectoryPlanning.h"

/** PaCMan name space */
namespace pacman {

	/** from ros trajectory to pacman interface */
	void convert(const definitions::Trajectory &trajectory, RobotUIBK::Command::Seq &commands, pacman::float_t start_time)
	{
		int NWayPoints = trajectory.robot_path.size();
		commands.resize(NWayPoints);

		for (int i = 0; i < NWayPoints; i++)
		{
			// first the arm
			for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
			{
				commands[i].arm.pos.c[j] = trajectory.robot_path[i].arm.joints[j];
				commands[i].arm.vel.c[j] = trajectory.robot_path[i].arm.velocity[j];
				commands[i].arm.acc.c[j] = trajectory.robot_path[i].arm.acceleration[j];
			}

			// then the hand
			commands[i].hand.pos.left[0] = trajectory.robot_path[i].hand.joints[0];
			commands[i].hand.vel.left[0] = trajectory.robot_path[i].hand.velocity[0];
			commands[i].hand.acc.left[0] = trajectory.robot_path[i].hand.acceleration[0];

			commands[i].hand.pos.left[1] = trajectory.robot_path[i].hand.joints[1];
			commands[i].hand.vel.left[1] = trajectory.robot_path[i].hand.velocity[1];
			commands[i].hand.acc.left[1] = trajectory.robot_path[i].hand.acceleration[1];

			commands[i].hand.pos.middle[0] = trajectory.robot_path[i].hand.joints[2];
			commands[i].hand.vel.middle[0] = trajectory.robot_path[i].hand.velocity[2];
			commands[i].hand.acc.middle[0] = trajectory.robot_path[i].hand.acceleration[2];

			commands[i].hand.pos.middle[1] = trajectory.robot_path[i].hand.joints[3];
			commands[i].hand.vel.middle[1] = trajectory.robot_path[i].hand.velocity[3];
			commands[i].hand.acc.middle[1] = trajectory.robot_path[i].hand.acceleration[3];

			commands[i].hand.pos.right[0] = trajectory.robot_path[i].hand.joints[4];
			commands[i].hand.vel.right[0] = trajectory.robot_path[i].hand.velocity[4];
			commands[i].hand.acc.right[0] = trajectory.robot_path[i].hand.acceleration[4];

			commands[i].hand.pos.right[1] = trajectory.robot_path[i].hand.joints[5];
			commands[i].hand.vel.right[1] = trajectory.robot_path[i].hand.velocity[5];
			commands[i].hand.acc.right[1] = trajectory.robot_path[i].hand.acceleration[5];

			commands[i].hand.pos.rotation = trajectory.robot_path[i].hand.joints[6];
			commands[i].hand.vel.rotation = trajectory.robot_path[i].hand.velocity[6];
			commands[i].hand.acc.rotation = trajectory.robot_path[i].hand.acceleration[6];

			// and finally the time
			if(i==0)
			commands[i].t = start_time + pacman::float_t(0.1);
			else
			commands[i].t = commands[i-1].t + pacman::float_t(trajectory.time_from_previous[i].toSec());
		}
		return;
	}

	// this mapping uses names defined in the urdf of the UIBK robot, so be careful
	void mapStates(const RobotUIBK::State &state, std::string name, sensor_msgs::JointState &joint_states, ros::Time stamp) 
	{

		// initialize the joint state topic
		joint_states.name.resize(RobotUIBK::Config::JOINTS);
		joint_states.position.resize(RobotUIBK::Config::JOINTS);
		joint_states.velocity.resize(RobotUIBK::Config::JOINTS);
		joint_states.effort.resize(RobotUIBK::Config::JOINTS);
		joint_states.name[0] = name + "_arm_0_joint";
		joint_states.name[1] = name + "_arm_1_joint";
		joint_states.name[2] = name + "_arm_2_joint";
		joint_states.name[3] = name + "_arm_3_joint";
		joint_states.name[4] = name + "_arm_4_joint";
		joint_states.name[5] = name + "_arm_5_joint";
		joint_states.name[6] = name + "_arm_6_joint";
		joint_states.name[7] = name + "_sdh_knuckle_joint";
		joint_states.name[8] = name + "_sdh_finger_12_joint";
		joint_states.name[9] = name + "_sdh_finger_13_joint";
		joint_states.name[10] = name + "_sdh_finger_22_joint";
		joint_states.name[11] = name + "_sdh_finger_23_joint";
		joint_states.name[12] = name + "_sdh_thumb_2_joint";
		joint_states.name[13] = name + "_sdh_thumb_3_joint";

		joint_states.header.stamp = stamp;

		// the joint mapping needs to be hardcoded to match Golem.xml and Ros.urdf structures
		// note that, the names are set in the class constructor for the order convention
		// and let the party begin... first the arm:
		for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
		{
			joint_states.position[j] = state.arm.pos.c[j];
		}

		// and continue with the hand:
		joint_states.position[7] = state.hand.pos.rotation;
		joint_states.position[8] = state.hand.pos.left[0];
		joint_states.position[9] = state.hand.pos.left[1];
		joint_states.position[10] = state.hand.pos.right[0];
		joint_states.position[11] = state.hand.pos.right[1];
		joint_states.position[12] = state.hand.pos.middle[0];
		joint_states.position[13] = state.hand.pos.middle[1];

		return;
	}

	// // this mapping uses names defined in the urdf of the UIBK robot, so be careful
	// void mapStates(const RobotEddie::State &state, std::string name, sensor_msgs::JointState &joint_states, ros::Time stamp) 
	// {

	// 	// initialize the joint state topic
	// 	joint_states.name.resize(RobotUIBK::Config::JOINTS);
	// 	joint_states.position.resize(RobotUIBK::Config::JOINTS);
	// 	joint_states.velocity.resize(RobotUIBK::Config::JOINTS);
	// 	joint_states.effort.resize(RobotUIBK::Config::JOINTS);
	// 	joint_states.name[0] = name + "_arm_0_joint";
	// 	joint_states.name[1] = name + "_arm_1_joint";
	// 	joint_states.name[2] = name + "_arm_2_joint";
	// 	joint_states.name[3] = name + "_arm_3_joint";
	// 	joint_states.name[4] = name + "_arm_4_joint";
	// 	joint_states.name[5] = name + "_arm_5_joint";
	// 	joint_states.name[6] = name + "_arm_6_joint";
	// 	joint_states.name[7] = name + "_sdh_knuckle_joint";
	// 	joint_states.name[8] = name + "_sdh_finger_12_joint";
	// 	joint_states.name[9] = name + "_sdh_finger_13_joint";
	// 	joint_states.name[10] = name + "_sdh_finger_22_joint";
	// 	joint_states.name[11] = name + "_sdh_finger_23_joint";
	// 	joint_states.name[12] = name + "_sdh_thumb_2_joint";
	// 	joint_states.name[13] = name + "_sdh_thumb_3_joint";

	// 	joint_states.header.stamp = stamp;

	// 	// the joint mapping needs to be hardcoded to match Golem.xml and Ros.urdf structures
	// 	// note that, the names are set in the class constructor for the order convention
	// 	// and let the party begin... first the arm:
	// 	for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
	// 	{
	// 		joint_states.position[j] = state.arm.pos.c[j];
	// 	}

	// 	// and continue with the hand:
	// 	joint_states.position[7] = state.hand.pos.rotation;
	// 	joint_states.position[8] = state.hand.pos.left[0];
	// 	joint_states.position[9] = state.hand.pos.left[1];
	// 	joint_states.position[10] = state.hand.pos.right[0];
	// 	joint_states.position[11] = state.hand.pos.right[1];
	// 	joint_states.position[12] = state.hand.pos.middle[0];
	// 	joint_states.position[13] = state.hand.pos.middle[1];

	// 	return;
	// }

};

#endif // _PACMAN_PACMAN_PCL_H_
