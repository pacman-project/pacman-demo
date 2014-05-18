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
				commands[i].t = start_time + pacman::float_t(0.5);
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
			commands[i].handRight.pos.rotation = trajectory.eddie_path[i].handRight.joints[0];
			commands[i].handRight.vel.rotation = trajectory.eddie_path[i].handRight.velocity[0];
			commands[i].handRight.acc.rotation = trajectory.eddie_path[i].handRight.acceleration[0];

			commands[i].handRight.pos.left[0] = trajectory.eddie_path[i].handRight.joints[1];
			commands[i].handRight.vel.left[0] = trajectory.eddie_path[i].handRight.velocity[1];
			commands[i].handRight.acc.left[0] = trajectory.eddie_path[i].handRight.acceleration[1];

			commands[i].handRight.pos.left[1] = trajectory.eddie_path[i].handRight.joints[2];
			commands[i].handRight.vel.left[1] = trajectory.eddie_path[i].handRight.velocity[2];
			commands[i].handRight.acc.left[1] = trajectory.eddie_path[i].handRight.acceleration[2];

			commands[i].handRight.pos.right[0] = trajectory.eddie_path[i].handRight.joints[3];
			commands[i].handRight.vel.right[0] = trajectory.eddie_path[i].handRight.velocity[3];
			commands[i].handRight.acc.right[0] = trajectory.eddie_path[i].handRight.acceleration[3];

			commands[i].handRight.pos.right[1] = trajectory.eddie_path[i].handRight.joints[4];
			commands[i].handRight.vel.right[1] = trajectory.eddie_path[i].handRight.velocity[4];
			commands[i].handRight.acc.right[1] = trajectory.eddie_path[i].handRight.acceleration[4];

			commands[i].handRight.pos.middle[0] = trajectory.eddie_path[i].handRight.joints[5];
			commands[i].handRight.vel.middle[0] = trajectory.eddie_path[i].handRight.velocity[5];
			commands[i].handRight.acc.middle[0] = trajectory.eddie_path[i].handRight.acceleration[5];

			commands[i].handRight.pos.middle[1] = trajectory.eddie_path[i].handRight.joints[6];
			commands[i].handRight.vel.middle[1] = trajectory.eddie_path[i].handRight.velocity[6];
			commands[i].handRight.acc.middle[1] = trajectory.eddie_path[i].handRight.acceleration[6];


			commands[i].handLeft.pos.rotation = trajectory.eddie_path[i].handLeft.joints[0];
			commands[i].handLeft.vel.rotation = trajectory.eddie_path[i].handLeft.velocity[0];
			commands[i].handLeft.acc.rotation = trajectory.eddie_path[i].handLeft.acceleration[0];

			commands[i].handLeft.pos.left[0] = trajectory.eddie_path[i].handLeft.joints[1];
			commands[i].handLeft.vel.left[0] = trajectory.eddie_path[i].handLeft.velocity[1];
			commands[i].handLeft.acc.left[0] = trajectory.eddie_path[i].handLeft.acceleration[1];

			commands[i].handLeft.pos.left[1] = trajectory.eddie_path[i].handLeft.joints[2];
			commands[i].handLeft.vel.left[1] = trajectory.eddie_path[i].handLeft.velocity[2];
			commands[i].handLeft.acc.left[1] = trajectory.eddie_path[i].handLeft.acceleration[2];

			commands[i].handLeft.pos.right[0] = trajectory.eddie_path[i].handLeft.joints[3];
			commands[i].handLeft.vel.right[0] = trajectory.eddie_path[i].handLeft.velocity[3];
			commands[i].handLeft.acc.right[0] = trajectory.eddie_path[i].handLeft.acceleration[3];

			commands[i].handLeft.pos.right[1] = trajectory.eddie_path[i].handLeft.joints[4];
			commands[i].handLeft.vel.right[1] = trajectory.eddie_path[i].handLeft.velocity[4];
			commands[i].handLeft.acc.right[1] = trajectory.eddie_path[i].handLeft.acceleration[4];

			commands[i].handLeft.pos.middle[0] = trajectory.eddie_path[i].handLeft.joints[5];
			commands[i].handLeft.vel.middle[0] = trajectory.eddie_path[i].handLeft.velocity[5];
			commands[i].handLeft.acc.middle[0] = trajectory.eddie_path[i].handLeft.acceleration[5];

			commands[i].handLeft.pos.middle[1] = trajectory.eddie_path[i].handLeft.joints[6];
			commands[i].handLeft.vel.middle[1] = trajectory.eddie_path[i].handLeft.velocity[6];
			commands[i].handLeft.acc.middle[1] = trajectory.eddie_path[i].handLeft.acceleration[6];

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
				commands[i].t = start_time + pacman::float_t(0.5);
			else
				commands[i].t = commands[i-1].t + pacman::float_t(trajectory.time_from_previous[i].toSec());
		}
		return;
	}

	void convertLimb(const moveit_msgs::RobotTrajectory &moveitTraj, definitions::Trajectory &trajectory, sensor_msgs::JointState start_state, std::string arm)
	{
        // get the points from robot trajectory
		const std::vector<trajectory_msgs::JointTrajectoryPoint> &points = moveitTraj.joint_trajectory.points;
	    double epsilon = 0.009;

		// pick each point from the trajectory and create a RobotEddie object
		for (size_t i = 0; i < points.size(); ++i) 
		{
			definitions::RobotEddie robot_point;
            bool found = false;
			for( size_t j = 0;( (j < points[i].positions.size()) && ( i > 0 ) ); j++ )
			{
               if( fabs(points[i].positions[j] - points[i-1].positions[j] ) > epsilon  )
               {
               	  found = true;
               	  //std::cout << "joint " << j << " are different " << points[i].positions[j]  << " : " << points[i-1].positions[j] << std::endl;
               	  break;
               }
			}
            if( ( !found ) && ( i > 0 ) )
            	continue;			

			// for testing with arms, later, it should be fixed to obtain eddie's trajectories
			if(arm.compare(std::string("right")) == 0)
			{
				// arm mapping
				for(int j = 0; j < KukaLWR::Config::JOINTS; j++)
				{
					robot_point.armRight.joints.push_back(points[i].positions.at(j));
					robot_point.armRight.velocity.push_back(points[i].velocities.at(j));
					robot_point.armRight.acceleration.push_back(points[i].accelerations.at(j));

					robot_point.armLeft.joints.push_back(start_state.position[j]);
					robot_point.armLeft.velocity.push_back(start_state.velocity[j]);
					robot_point.armLeft.acceleration.push_back(0.0);
				}

				// hand mapping
				// this order depends on the moveit trajectory planner
				robot_point.handRight.joints.resize(SchunkDexHand::Config::JOINTS);
				robot_point.handRight.velocity.resize(SchunkDexHand::Config::JOINTS);
				robot_point.handRight.acceleration.resize(SchunkDexHand::Config::JOINTS);

				for(int h = 0; h < SchunkDexHand::Config::JOINTS; h++)
				{
					robot_point.handRight.joints[h] = points[i].positions[7+h];
					robot_point.handRight.velocity[h] = points[i].velocities[7+h];
					robot_point.handRight.acceleration[h] = points[i].accelerations[7+h];

					robot_point.handLeft.joints.push_back(start_state.position[h+14]);
					robot_point.handLeft.velocity.push_back(start_state.velocity[h+14]);
					robot_point.handLeft.acceleration.push_back(0.0);
				}
			}

			if(arm.compare(std::string("left")) == 0)
			{
				for(int j = 0; j < KukaLWR::Config::JOINTS; j++)
				{
					robot_point.armLeft.joints.push_back(points[i].positions.at(j));
					robot_point.armLeft.velocity.push_back(points[i].velocities.at(j));
					robot_point.armLeft.acceleration.push_back(points[i].accelerations.at(j));

					robot_point.armRight.joints.push_back(start_state.position[j+7]);
					robot_point.armRight.velocity.push_back(start_state.velocity[j+7]);
					robot_point.armRight.acceleration.push_back(0.0);
				}

				// hand mapping
				// this order depends on the moveit trajectory planner
				robot_point.handLeft.joints.resize(SchunkDexHand::Config::JOINTS);
				robot_point.handLeft.velocity.resize(SchunkDexHand::Config::JOINTS);
				robot_point.handLeft.acceleration.resize(SchunkDexHand::Config::JOINTS);

				for(int h = 0; h < SchunkDexHand::Config::JOINTS; h++)
				{
					robot_point.handLeft.joints[h] = points[i].positions[7+h];
					robot_point.handLeft.velocity[h] = points[i].velocities[7+h];
					robot_point.handLeft.acceleration[h] = points[i].accelerations[7+h];

					robot_point.handRight.joints.push_back(start_state.position[h+21]);
					robot_point.handRight.velocity.push_back(start_state.velocity[h+21]);
					robot_point.handRight.acceleration.push_back(0.0);
				}
			}

			for(int k = 0; k < KITHead::Config::JOINTS_NECK; k++)
			{
				robot_point.head.joints.push_back(start_state.position[k+28]);
				robot_point.head.velocity.push_back(start_state.velocity[k+28]);
				robot_point.head.acceleration.push_back(0.0);
			}

			robot_point.head.jointsLEye = start_state.position[33];
			robot_point.head.velocityLEye = start_state.velocity[33];
			robot_point.head.accelerationLEye = 0.0;

			robot_point.head.jointsREye = start_state.position[34];
			robot_point.head.velocityREye = start_state.velocity[34];
			robot_point.head.accelerationREye = 0.0;

			trajectory.eddie_path.push_back(robot_point);

			if (i == 0)
			{
				trajectory.time_from_previous.push_back( ros::Duration().fromSec(0.) );	
			}
			else
			{
				// the RobotTrajectory gives time_from_start, we prefer from previous for easier transformation to pacman commands
				ros::Duration dt( points[i].time_from_start - points[i-1].time_from_start );
				trajectory.time_from_previous.push_back( dt );
			}
		}
	}

	void convertJointStateToEddie(const sensor_msgs::JointState &state, definitions::RobotEddie &eddie)
	{	

		// arms mapping
		eddie.armRight.joints.resize(KukaLWR::Config::JOINTS);
		eddie.armRight.velocity.resize(KukaLWR::Config::JOINTS);
		eddie.armRight.acceleration.resize(KukaLWR::Config::JOINTS);
		eddie.armLeft.joints.resize(KukaLWR::Config::JOINTS);
		eddie.armLeft.velocity.resize(KukaLWR::Config::JOINTS);
		eddie.armLeft.acceleration.resize(KukaLWR::Config::JOINTS);
		for(int j = 0; j < KukaLWR::Config::JOINTS; j++)
		{
			eddie.armRight.joints.at(j) = state.position[j+7];
			eddie.armRight.velocity.at(j) = state.velocity[j+7];
			eddie.armRight.acceleration.at(j) = 0.0;

			eddie.armLeft.joints.at(j) = state.position[j];
			eddie.armLeft.velocity.at(j) = state.velocity[j];
			eddie.armLeft.acceleration.at(j) = 0.0;
		}

		// hands mapping
		eddie.handRight.joints.resize(SchunkDexHand::Config::JOINTS);
		eddie.handRight.velocity.resize(SchunkDexHand::Config::JOINTS);
		eddie.handRight.acceleration.resize(SchunkDexHand::Config::JOINTS);
		eddie.handLeft.joints.resize(SchunkDexHand::Config::JOINTS);
		eddie.handLeft.velocity.resize(SchunkDexHand::Config::JOINTS);
		eddie.handLeft.acceleration.resize(SchunkDexHand::Config::JOINTS);
		for(int h = 0; h < SchunkDexHand::Config::JOINTS; h++)
		{
			eddie.handLeft.joints.at(h) = state.position[h+14];
			eddie.handLeft.velocity.at(h) = state.velocity[h+14];
			eddie.handLeft.acceleration.at(h) = 0.0;

			eddie.handRight.joints.at(h) = state.position[h+21];
			eddie.handRight.velocity.at(h) = state.velocity[h+21];
			eddie.handRight.acceleration.at(h) = 0.0;
		}

		// head mapping
		eddie.head.joints.resize(KITHead::Config::JOINTS);
		eddie.head.velocity.resize(KITHead::Config::JOINTS);
		eddie.head.acceleration.resize(KITHead::Config::JOINTS);
		for(int k = 0; k < KITHead::Config::JOINTS_NECK; k++)
		{
			eddie.head.joints.at(k) = state.position[k+28];
			eddie.head.velocity.at(k) = state.velocity[k+28];
			eddie.head.acceleration.at(k) = 0.0;
		}

		eddie.head.jointsLEye = state.position[33];
		eddie.head.velocityLEye = state.velocity[33];
		eddie.head.accelerationLEye = 0.0;

		eddie.head.jointsREye = state.position[34];
		eddie.head.velocityREye = state.velocity[34];
		eddie.head.accelerationREye = 0.0;
	}

	// this mapping uses names defined in the urdf of the UIBK robot, so be careful if you change them
	void mapStates(const RobotEddie::State &state, sensor_msgs::JointState &joint_states, ros::Time stamp) 
	{

		// initialize the joint state topic
		joint_states.name.resize(RobotEddie::Config::JOINTS);
		joint_states.position.resize(RobotEddie::Config::JOINTS);
		joint_states.velocity.resize(RobotEddie::Config::JOINTS);
		joint_states.effort.resize(RobotEddie::Config::JOINTS);
		joint_states.name[0] = "left_arm_0_joint";
		joint_states.name[1] = "left_arm_1_joint";
		joint_states.name[2] = "left_arm_2_joint";
		joint_states.name[3] = "left_arm_3_joint";
		joint_states.name[4] = "left_arm_4_joint";
		joint_states.name[5] = "left_arm_5_joint";
		joint_states.name[6] = "left_arm_6_joint";
		joint_states.name[7] = "right_arm_0_joint";
		joint_states.name[8] = "right_arm_1_joint";
		joint_states.name[9] = "right_arm_2_joint";
		joint_states.name[10] = "right_arm_3_joint";
		joint_states.name[11] = "right_arm_4_joint"; 
		joint_states.name[12] = "right_arm_5_joint";
		joint_states.name[13] = "right_arm_6_joint"; 
		joint_states.name[14] = "left_sdh_knuckle_joint";
		joint_states.name[15] = "left_sdh_finger_12_joint";
		joint_states.name[16] = "left_sdh_finger_13_joint";
		joint_states.name[17] = "left_sdh_finger_22_joint";
		joint_states.name[18] = "left_sdh_finger_23_joint";
		joint_states.name[19] = "left_sdh_thumb_2_joint";
		joint_states.name[20] = "left_sdh_thumb_3_joint";
		joint_states.name[21] = "right_sdh_knuckle_joint";
		joint_states.name[22] = "right_sdh_finger_12_joint";
		joint_states.name[23] = "right_sdh_finger_13_joint";
		joint_states.name[24] = "right_sdh_finger_22_joint";
		joint_states.name[25] = "right_sdh_finger_23_joint";
		joint_states.name[26] = "right_sdh_thumb_2_joint";
		joint_states.name[27] = "right_sdh_thumb_3_joint";
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
			joint_states.position[j] = state.armLeft.pos.c[j];
			joint_states.position[j+7] = state.armRight.pos.c[j];
		}

		// and continue with the hands:
		joint_states.position[14] = state.handLeft.pos.rotation;
		joint_states.position[15] = state.handLeft.pos.left[0];
		joint_states.position[16] = state.handLeft.pos.left[1];
		joint_states.position[17] = state.handLeft.pos.right[0];
		joint_states.position[18] = state.handLeft.pos.right[1];
		joint_states.position[19] = state.handLeft.pos.middle[0];
		joint_states.position[20] = state.handLeft.pos.middle[1];
		joint_states.position[21] = state.handRight.pos.rotation;
		joint_states.position[22] = state.handRight.pos.left[0];
		joint_states.position[23] = state.handRight.pos.left[1];
		joint_states.position[24] = state.handRight.pos.right[0];
		joint_states.position[25] = state.handRight.pos.right[1];
		joint_states.position[26] = state.handRight.pos.middle[0];
		joint_states.position[27] = state.handRight.pos.middle[1];
		joint_states.position[28] = state.head.pos.neck[0];
		joint_states.position[29] = state.head.pos.neck[1];
		joint_states.position[30] = state.head.pos.neck[2];
		joint_states.position[31] = state.head.pos.neck[3];
		joint_states.position[32] = state.head.pos.neck[4];
		joint_states.position[33] = state.head.pos.eyeLeft;
		joint_states.position[34] = state.head.pos.eyeRight;

		return;
	}

	// this mapping uses names defined in the urdf of the UIBK robot, so be careful if you change them
	void mapStates(const sensor_msgs::JointState &joint_states, RobotEddie::State &state) 
	{

		// the joint mapping needs to be hardcoded to match Golem.xml and Ros.urdf structures
		// note that, the names are set in the class constructor for the order convention
		// and let the party begin... first the arm:
		for (int j = 0; j < KukaLWR::Config::JOINTS; j++)
		{
			state.armLeft.pos.c[j] = joint_states.position[j];
			state.armRight.pos.c[j] = joint_states.position[j+7];
		}

		// and continue with the hands:
		state.handLeft.pos.rotation = joint_states.position[14];
		state.handLeft.pos.left[0] = joint_states.position[15];
		state.handLeft.pos.left[1] = joint_states.position[16];
		state.handLeft.pos.right[0] = joint_states.position[17];
		state.handLeft.pos.right[1] = joint_states.position[18];
		state.handLeft.pos.middle[0] = joint_states.position[19];
		state.handLeft.pos.middle[1] = joint_states.position[20];

		state.handRight.pos.rotation = joint_states.position[21];
		state.handRight.pos.left[0] = joint_states.position[22];
		state.handRight.pos.left[1] = joint_states.position[23];
		state.handRight.pos.right[0] = joint_states.position[24];
		state.handRight.pos.right[1] = joint_states.position[25];
		state.handRight.pos.middle[0] = joint_states.position[26]; 
		state.handRight.pos.middle[1] = joint_states.position[27];

		state.head.pos.neck[0] = joint_states.position[28];
		state.head.pos.neck[1] = joint_states.position[29];
		state.head.pos.neck[2] = joint_states.position[30];
		state.head.pos.neck[3] = joint_states.position[31];
		state.head.pos.neck[4] = joint_states.position[32];
		state.head.pos.eyeLeft = joint_states.position[33];
		state.head.pos.eyeRight = joint_states.position[34];

		return;
	}

    void interpolateHandJoints(const definitions::SDHand &goalState, const sensor_msgs::JointState &startState, moveit_msgs::RobotTrajectory &baseTrajectory, const std::string &arm,bool is_cart = true)
	{
		int NWayPoints = baseTrajectory.joint_trajectory.points.size();
		int right_hand_index = 21;
		int left_hand_index = 14;
		int hand_index = 0;

		//std::cout << "startState NWayPoints" << NWayPoints << std::endl << startState << std::endl;

		//std::cout << goalState << std::endl;

		if(arm.compare(std::string("right")) == 0)
			hand_index = right_hand_index;
		if(arm.compare(std::string("left")) == 0)
			hand_index = left_hand_index;

		for (int i = 0; i < NWayPoints ; i++)
		{
			// trajectory_msgs::JointTrajectoryPoint &point = baseTrajectory.joint_trajectory.points[i];

			for(int h = 0; h < pacman::SchunkDexHand::Config::JOINTS; h++)
			{
				if( is_cart )
				  baseTrajectory.joint_trajectory.points[i].positions.push_back(startState.position[hand_index + h] + (i)*( (goalState.joints[h] - startState.position[hand_index + h])/(NWayPoints-1) ) );
				else
				  baseTrajectory.joint_trajectory.points[i].positions.push_back( goalState.joints[h] );
				//baseTrajectory.joint_trajectory.points[i].positions.push_back( goalState.joints[h] );
	        }
		
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].velocities.push_back(0.0);

			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
			baseTrajectory.joint_trajectory.points[i].accelerations.push_back(0.0);
		}
		//std::cout << "baseTrajectory" << baseTrajectory << std::endl;
	}

};

#endif // _PACMAN_PACMAN_PCL_H_
