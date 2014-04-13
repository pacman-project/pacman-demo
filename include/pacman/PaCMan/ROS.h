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

	void convert(const definitions::Trajectory &trajectory, RobotEddie::Command::Seq &commands, pacman::float_t start_time)
	{
		int NWayPoints = trajectory.eddie_path.size();
		commands.resize(NWayPoints);

		for (int i = 0; i < NWayPoints; i++)
		{
			// first the arms
			for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
			{
				commands[i].armRight.pos.c[j] = trajectory.eddie_path[i].armRight.joints[j];
				commands[i].armRight.vel.c[j] = trajectory.eddie_path[i].armRight.velocity[j];
				commands[i].armRight.acc.c[j] = trajectory.eddie_path[i].armRight.acceleration[j];

				commands[i].armLeft.pos.c[j] = trajectory.eddie_path[i].armLeft.joints[j];
				commands[i].armLeft.vel.c[j] = trajectory.eddie_path[i].armLeft.velocity[j];
				commands[i].armLeft.acc.c[j] = trajectory.eddie_path[i].armLeft.acceleration[j];
			}

			// then the hands
			commands[i].handRight.pos.left[0] = trajectory.eddie_path[i].handRight.joints[0];
			commands[i].handRight.vel.left[0] = trajectory.eddie_path[i].handRight.velocity[0];
			commands[i].handRight.acc.left[0] = trajectory.eddie_path[i].handRight.acceleration[0];

			commands[i].handRight.pos.left[1] = trajectory.eddie_path[i].handRight.joints[1];
			commands[i].handRight.vel.left[1] = trajectory.eddie_path[i].handRight.velocity[1];
			commands[i].handRight.acc.left[1] = trajectory.eddie_path[i].handRight.acceleration[1];

			commands[i].handRight.pos.middle[0] = trajectory.eddie_path[i].handRight.joints[2];
			commands[i].handRight.vel.middle[0] = trajectory.eddie_path[i].handRight.velocity[2];
			commands[i].handRight.acc.middle[0] = trajectory.eddie_path[i].handRight.acceleration[2];

			commands[i].handRight.pos.middle[1] = trajectory.eddie_path[i].handRight.joints[3];
			commands[i].handRight.vel.middle[1] = trajectory.eddie_path[i].handRight.velocity[3];
			commands[i].handRight.acc.middle[1] = trajectory.eddie_path[i].handRight.acceleration[3];

			commands[i].handRight.pos.right[0] = trajectory.eddie_path[i].handRight.joints[4];
			commands[i].handRight.vel.right[0] = trajectory.eddie_path[i].handRight.velocity[4];
			commands[i].handRight.acc.right[0] = trajectory.eddie_path[i].handRight.acceleration[4];

			commands[i].handRight.pos.right[1] = trajectory.eddie_path[i].handRight.joints[5];
			commands[i].handRight.vel.right[1] = trajectory.eddie_path[i].handRight.velocity[5];
			commands[i].handRight.acc.right[1] = trajectory.eddie_path[i].handRight.acceleration[5];

			commands[i].handRight.pos.rotation = trajectory.eddie_path[i].handRight.joints[6];
			commands[i].handRight.vel.rotation = trajectory.eddie_path[i].handRight.velocity[6];
			commands[i].handRight.acc.rotation = trajectory.eddie_path[i].handRight.acceleration[6];


			commands[i].handLeft.pos.left[0] = trajectory.eddie_path[i].handLeft.joints[0];
			commands[i].handLeft.vel.left[0] = trajectory.eddie_path[i].handLeft.velocity[0];
			commands[i].handLeft.acc.left[0] = trajectory.eddie_path[i].handLeft.acceleration[0];

			commands[i].handLeft.pos.left[1] = trajectory.eddie_path[i].handLeft.joints[1];
			commands[i].handLeft.vel.left[1] = trajectory.eddie_path[i].handLeft.velocity[1];
			commands[i].handLeft.acc.left[1] = trajectory.eddie_path[i].handLeft.acceleration[1];

			commands[i].handLeft.pos.middle[0] = trajectory.eddie_path[i].handLeft.joints[2];
			commands[i].handLeft.vel.middle[0] = trajectory.eddie_path[i].handLeft.velocity[2];
			commands[i].handLeft.acc.middle[0] = trajectory.eddie_path[i].handLeft.acceleration[2];

			commands[i].handLeft.pos.middle[1] = trajectory.eddie_path[i].handLeft.joints[3];
			commands[i].handLeft.vel.middle[1] = trajectory.eddie_path[i].handLeft.velocity[3];
			commands[i].handLeft.acc.middle[1] = trajectory.eddie_path[i].handLeft.acceleration[3];

			commands[i].handLeft.pos.right[0] = trajectory.eddie_path[i].handLeft.joints[4];
			commands[i].handLeft.vel.right[0] = trajectory.eddie_path[i].handLeft.velocity[4];
			commands[i].handLeft.acc.right[0] = trajectory.eddie_path[i].handLeft.acceleration[4];

			commands[i].handLeft.pos.right[1] = trajectory.eddie_path[i].handLeft.joints[5];
			commands[i].handLeft.vel.right[1] = trajectory.eddie_path[i].handLeft.velocity[5];
			commands[i].handLeft.acc.right[1] = trajectory.eddie_path[i].handLeft.acceleration[5];

			commands[i].handLeft.pos.rotation = trajectory.eddie_path[i].handLeft.joints[6];
			commands[i].handLeft.vel.rotation = trajectory.eddie_path[i].handLeft.velocity[6];
			commands[i].handLeft.acc.rotation = trajectory.eddie_path[i].handLeft.acceleration[6];

			// then he head

			for (int j = 0; j < KITHead::Config::JOINTS_NECK; j++)
			{
				commands[i].head.pos.neck[j] = trajectory.eddie_path[i].head.joints[j];
				commands[i].head.vel.neck[j] = trajectory.eddie_path[i].head.velocity[j];
				commands[i].head.acc.neck[j] = trajectory.eddie_path[i].head.acceleration[j];
			}

			commands[i].head.pos.eyeLeft = trajectory.eddie_path[i].head.jointsLEye;
			commands[i].head.vel.eyeLeft = trajectory.eddie_path[i].head.velocityLEye;
			commands[i].head.acc.eyeLeft = trajectory.eddie_path[i].head.accelerationLEye;

			commands[i].head.pos.eyeRight = trajectory.eddie_path[i].head.jointsREye;
			commands[i].head.vel.eyeRight = trajectory.eddie_path[i].head.velocityREye;
			commands[i].head.acc.eyeRight = trajectory.eddie_path[i].head.accelerationREye;

			// finally the time
			if(i==0)
				commands[i].t = start_time + pacman::float_t(0.1);
			else
				commands[i].t = commands[i-1].t + pacman::float_t(trajectory.time_from_previous[i].toSec());
		}
		return;
	}

	// this mapping uses names defined in the urdf of the UIBK robot, so be careful if you change them
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

	// this mapping uses names defined in the urdf of the UIBK robot, so be careful if you change them
	void mapStates(const RobotEddie::State &state, sensor_msgs::JointState &joint_states, ros::Time stamp) 
	{

		// initialize the joint state topic
		joint_states.name.resize(RobotEddie::Config::JOINTS);
		joint_states.position.resize(RobotEddie::Config::JOINTS);
		joint_states.velocity.resize(RobotEddie::Config::JOINTS);
		joint_states.effort.resize(RobotEddie::Config::JOINTS);
		joint_states.name[0] = "right_arm_0_joint";
		joint_states.name[1] = "right_arm_1_joint";
		joint_states.name[2] = "right_arm_2_joint";
		joint_states.name[3] = "right_arm_3_joint";
		joint_states.name[4] = "right_arm_4_joint";
		joint_states.name[5] = "right_arm_5_joint";
		joint_states.name[6] = "right_arm_6_joint";
		joint_states.name[7] = "right_sdh_knuckle_joint";
		joint_states.name[8] = "right_sdh_finger_12_joint";
		joint_states.name[9] = "right_sdh_finger_13_joint";
		joint_states.name[10] = "right_sdh_finger_22_joint";
		joint_states.name[11] = "right_sdh_finger_23_joint";
		joint_states.name[12] = "right_sdh_thumb_2_joint";
		joint_states.name[13] = "right_sdh_thumb_3_joint";
		joint_states.name[14] = "left_arm_0_joint";
		joint_states.name[15] = "left_arm_1_joint";
		joint_states.name[16] = "left_arm_2_joint";
		joint_states.name[17] = "left_arm_3_joint";
		joint_states.name[18] = "left_arm_4_joint";
		joint_states.name[19] = "left_arm_5_joint";
		joint_states.name[20] = "left_arm_6_joint";
		joint_states.name[21] = "left_sdh_knuckle_joint";
		joint_states.name[22] = "left_sdh_finger_12_joint";
		joint_states.name[23] = "left_sdh_finger_13_joint";
		joint_states.name[24] = "left_sdh_finger_22_joint";
		joint_states.name[25] = "left_sdh_finger_23_joint";
		joint_states.name[26] = "left_sdh_thumb_2_joint";
		joint_states.name[27] = "left_sdh_thumb_3_joint";

		joint_states.name[28] = "head_neck_pitch_joint";
		joint_states.name[29] = "head_neck_yaw_joint";
		joint_states.name[30] = "head_neck_roll_joint";
		joint_states.name[31] = "head_head_tilt_joint";
		joint_states.name[32] = "head_eyes_tilt_joint";
		joint_states.name[33] = "head_left_eye_joint";
		joint_states.name[34] = "head_right_eye_joint";

		joint_states.header.stamp = stamp;

		// the joint mapping needs to be hardcoded to match Golem.xml and Ros.urdf structures
		// note that, the names are set in the class constructor for the order convention
		// and let the party begin... first the arm:
		for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
		{
			joint_states.position[j] = state.armRight.pos.c[j];
			joint_states.position[j+14] = state.armLeft.pos.c[j];
		}

		// and continue with the hands:
		joint_states.position[7] = state.handRight.pos.rotation;
		joint_states.position[8] = state.handRight.pos.left[0];
		joint_states.position[9] = state.handRight.pos.left[1];
		joint_states.position[10] = state.handRight.pos.right[0];
		joint_states.position[11] = state.handRight.pos.right[1];
		joint_states.position[12] = state.handRight.pos.middle[0];
		joint_states.position[13] = state.handRight.pos.middle[1];
		joint_states.position[21] = state.handLeft.pos.rotation;
		joint_states.position[22] = state.handLeft.pos.left[0];
		joint_states.position[23] = state.handLeft.pos.left[1];
		joint_states.position[24] = state.handLeft.pos.right[0];
		joint_states.position[25] = state.handLeft.pos.right[1];
		joint_states.position[26] = state.handLeft.pos.middle[0];
		joint_states.position[27] = state.handLeft.pos.middle[1];

		joint_states.position[28] = state.head.pos.neck[0];
		joint_states.position[29] = state.head.pos.neck[1];
		joint_states.position[30] = state.head.pos.neck[2];
		joint_states.position[31] = state.head.pos.neck[3];
		joint_states.position[32] = state.head.pos.neck[4];
		joint_states.position[33] = state.head.pos.eyeRight;
		joint_states.position[34] = state.head.pos.eyeLeft;

		return;
	}

};

#endif // _PACMAN_PACMAN_PCL_H_
