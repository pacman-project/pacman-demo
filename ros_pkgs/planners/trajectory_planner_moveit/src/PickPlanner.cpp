//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
/* to add IterativeParabolicTimeParameterization functionalities */
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "PickPlanner.h"

#include <pacman/PaCMan/ROS.h>
#include <pacman/PaCMan/Defs.h>
// #include <Grasp/Director/Director.h>

namespace trajectory_planner_moveit {

bool PickPlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	if( request.type == request.PICK)
	{
		ROS_INFO("Received pick planning request");
		ros::Time now = ros::Time::now();

		// this is were the response will be filled
		std::vector<definitions::Trajectory> &trajectories = response.trajectory;

		// wait for a joint state to read the current state
		boost::shared_ptr<const sensor_msgs::JointState> current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic_, ros::Duration(3.0));
		if (!current_state_ptr)
		{
			ROS_ERROR("No real start state available, since no joint state recevied in topic: %s", topic_.c_str());
			ROS_ERROR("Did you forget to start a controller?");
			response.result = response.OTHER_ERROR;
			return false;
			// ROS_ERROR("Planning will be done from home position, however this trajectory might not be good for execution!");
		}
		sensor_msgs::JointState startJointState = *current_state_ptr;
		moveit_msgs::RobotState start_state;
		start_state.joint_state = startJointState;

		std::vector<definitions::Grasp> grasp_list = request.ordered_grasp;

		// plan just for the 1st given grasp in the list
		if ( !grasp_list.size() )
		{
			ROS_ERROR("No grasp available: did we finish already?");
			response.result = response.OTHER_ERROR;
			return false;
		}
		
		// the current grasp
		definitions::Grasp current_grasp = grasp_list[0];
		
		// set the group
		// request.arm is either "left" or "right"
		if(request.arm.compare(std::string("right")) != 0 && request.arm.compare(std::string("left")) != 0)
		{
			ROS_ERROR("The request.arm string should either be \"right\" or \"left\", while it is now \"%s\"", request.arm.c_str());
			response.result = response.OTHER_ERROR;
			return false;
		}
		move_group_interface::MoveGroup arm_group( request.arm + "_arm");

		// set the goal
		plan_for_frame_ = request.arm + "_sdh_palm_link";
		arm_group.setEndEffectorLink(plan_for_frame_);

		// setting the planner and general parameters
		arm_group.setPlannerId("PRMstarkConfigDefault");
		arm_group.setGoalPositionTolerance (tolerance_in_position_);
		arm_group.setGoalOrientationTolerance (tolerance_in_orientation_);
		arm_group.setPlanningTime(max_planning_time_);

		// set the current state
		arm_group.setStartState(start_state);

		ROS_INFO("Going for the grasp...");

		// use consecutive IK calculation instead of a single call to computeCartesianPath:
		// this way, only the right poses will be computed (not more waypoints)
		double fraction = 0.0;
		
		std::vector< moveit_msgs::RobotState > IK_grasping_arm;
		moveit_msgs::RobotState solution;
		
		for ( int i=0; i<current_grasp.grasp_trajectory.size(); ++i )
		{
			bool ik_success;
			// all these variables are defined just for clarity, they are not actually needed
			string &arm = request.arm;
			geometry_msgs::PoseStamped &goal = current_grasp.grasp_trajectory[i].wrist_pose;
			sensor_msgs::JointState state;
			if ( i==0 )
			{
				// in the first step, use the initial configuration as initial guess
				state = start_state.joint_state;
			}
			else
			{
				// use last joint value as initial guess
				state = solution.joint_state;
			}
			
			ik_success = ki_helper_.computeIK(arm, goal, state, solution, plan_for_frame_); //, avoid_collisions = true, attempts = 5, timeout = 0.1);
			
			if ( !ik_success )
			{
				ROS_ERROR("Only %f part of the path was computed.", fraction/current_grasp.grasp_trajectory.size());
				ROS_ERROR("The goal configuration can not be reached: no IK solution");
				response.result = response.NO_FEASIBLE_TRAJECTORY_FOUND;
				return false;
			}

// 			// check via direct inspection for closeness of the found solution
// 			// they are actually pretty close!
// 			geometry_msgs::Pose FKsolution;
// 			ki_helper_.computeFK(solution, plan_for_frame_, FKsolution); // , frame_id = "world_link");
// 			std::cout << "goal.pose:" << std::endl << goal.pose << std::endl;
// 			std::cout << "FKsolution:" << std::endl << FKsolution << std::endl;
			
			fraction += 1.0;
			IK_grasping_arm.push_back(solution);
			
		}
		
		// VISUALIZATION PURPOSES ONLY
		// display the path to check that the found positions are good
// 		moveit_msgs::DisplayTrajectory display_trajectory;
// 		display_trajectory.trajectory_start = start_state;
// 		display_trajectory.trajectory.push_back(robot_trajectory);
// 		display_publisher_.publish(display_trajectory);
		moveit_msgs::DisplayRobotState robot_state_display;
		for ( int i=0; i<IK_grasping_arm.size(); ++i )
		{
			robot_state_display.state = IK_grasping_arm.at(i);
			display_publisher_.publish(robot_state_display);
			usleep(500000);
		}
		
		// possible TODO: add check similar to jump_threshold_ one
		
		// build motion_plan_request structure
		moveit_msgs::GetMotionPlan motion_plan;
		moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;
		
		// needed parameters
		std::string group_name_ = request.arm + "_arm_hand";
		motion_plan_request.group_name = group_name_.c_str();
		motion_plan_request.num_planning_attempts = max_planning_attempts_;
		motion_plan_request.allowed_planning_time = max_planning_time_;
		motion_plan_request.planner_id = "PRMstarkConfigDefault";
		// QUESTION: do we need collision_objects_ here?
		// start_state.attached_collision_objects = collision_objects_;
		// collision_objects_.clear();
		motion_plan_request.start_state = start_state;
		
// 		// insert trajectory constraints
// 		for ( int i=0; i<current_grasp.grasp_trajectory.size(); ++i )
// 		{
// 			moveit_msgs::Constraints single_constr;
// 			moveit_msgs::JointConstraint joint_constraint;
// 			
// 			// add joints of the arm
// 			for ( int j=0; j< IK_grasping_arm[i].joint_state.position.size(); ++j )
// 			{
// 				if ( IK_grasping_arm[i].joint_state.name[j].find(request.arm + "_arm") < IK_grasping_arm[i].joint_state.name[j].size()-2 )
// 				{
// 					joint_constraint.joint_name = IK_grasping_arm[i].joint_state.name[j];
// 					joint_constraint.position = IK_grasping_arm[i].joint_state.position[j];
// 					joint_constraint.weight = 1.0;
// 					// // tolerances of 0.0 above and below are the default anyway
// 					// joint_constraint.tolerance_above = 0.0;
// 					// joint_constraint.tolerance_below = 0.0;
// 					single_constr.joint_constraints.push_back(joint_constraint);
// 				}
// 			}
// 			// add joints of the hand
// 			for ( int j=0; j<7; ++j )
// 			{
// 				// TODO: improve this ugly switch to name the hand joints...
// 				joint_constraint.joint_name = request.arm;
// 				switch (j)
// 				{
// 					case 0:
// 						joint_constraint.joint_name += "_sdh_knuckle_joint";
// 						break;
// 					case 1:
// 						joint_constraint.joint_name += "_sdh_finger_12_joint";
// 						break;
// 					case 2:
// 						joint_constraint.joint_name += "_sdh_finger_13_joint";
// 						break;
// 					case 3:
// 						joint_constraint.joint_name += "_sdh_finger_22_joint";
// 						break;
// 					case 4:
// 						joint_constraint.joint_name += "_sdh_finger_23_joint";
// 						break;
// 					case 5:
// 						joint_constraint.joint_name += "_sdh_thumb_2_joint";
// 						break;
// 					case 6:
// 						joint_constraint.joint_name += "_sdh_thumb_3_joint";
// 						break;
// 				}
// 
// 				joint_constraint.position = current_grasp.grasp_trajectory[i].joints[j];
// 				joint_constraint.weight = 1.0;
// 				// // tolerances of 0.0 above and below are the default anyway
// 				// joint_constraint.tolerance_above = 0.0;
// 				// joint_constraint.tolerance_below = 0.0;
// 				single_constr.joint_constraints.push_back(joint_constraint);
// 			}
// 
// 			// all grasp waypoints are trajectory constraints but last one, which is the goal constraint
// 			if (i < current_grasp.grasp_trajectory.size()-1 )
// 			{
// 				motion_plan_request.trajectory_constraints.constraints.push_back(single_constr);
// 			}
// 			else
// 			{
// 				motion_plan_request.goal_constraints.push_back(single_constr);
// 			}
// 			
// 		}
// 		
// 		// // check for correctness the imposed trajectory_constraints
// 		// std::cout << "motion_plan_request.trajectory_constraints: " << std::endl << motion_plan_request.trajectory_constraints << std::endl;
// 		
// 		// call the service with the request
// 		ROS_INFO("Calling plannig service...");
// 		bool success = clt_moveit_planning_.call(motion_plan);
		
		
		// // Instead of calling the motion planner with the defined constraints, use functionalities brought by
		// // IterativeParabolicTimeParameterization tool
		
		// fill a moveit_msgs::RobotTrajectory with the desired waypoints
		moveit_msgs::RobotTrajectory trajectory_msg;
		trajectory_msg.joint_trajectory.points.resize( current_grasp.grasp_trajectory.size() + 1 );
		
		for ( int i=0; i<current_grasp.grasp_trajectory.size(); ++i )
		{
			// add joints of the arm
			for ( int j=0; j< IK_grasping_arm[i].joint_state.position.size(); ++j )
			{
				// only consider "arm" joints of the specified arm
				if ( IK_grasping_arm[i].joint_state.name[j].find( request.arm + "_arm") < IK_grasping_arm[i].joint_state.name[j].size()-2 )
				{
					// add the names in the structure, only once
					// here the hypothesis is: the joint order won't change between different IK iterations
					if ( i==0 )
					{
						trajectory_msg.joint_trajectory.joint_names.push_back(IK_grasping_arm.at(0).joint_state.name.at(j));
						for ( int k=0; k<startJointState.name.size(); ++k )
						{
							if ( !startJointState.name.at(k).compare( trajectory_msg.joint_trajectory.joint_names.back() ) )
							{
								trajectory_msg.joint_trajectory.points.at(0).positions.push_back( startJointState.position.at(k) );
								break;
							}
						}
						
						// // std::cout << "trajectory_msg.joint_trajectory.points.at(0).positions.size(): " << trajectory_msg.joint_trajectory.points.at(0).positions.size() << std::endl;
						// check if the joint value was found, otherwise throw an error
						if ( trajectory_msg.joint_trajectory.points.at(0).positions.size() < trajectory_msg.joint_trajectory.joint_names.size() )
						{
							std::cout << "PickPlanner error: couldn't retrieve initial value of joint " << trajectory_msg.joint_trajectory.joint_names.back() << std::endl;
							response.result = response.OTHER_ERROR;
							return false;
						}
					}
					
					// std::cout << "IK_grasping_arm.at(" << i << ").joint_state.position.at(" << j << "): ";
					// std::cout << IK_grasping_arm.at(i).joint_state.position.at(j) << std::endl;
					// all the cycles, add the corresponding point of the trajectory (joint values)
					trajectory_msg.joint_trajectory.points.at(i+1).positions.push_back(IK_grasping_arm.at(i).joint_state.position.at(j));
					// std::cout << "trajectory_msg.joint_trajectory.points.at(" << i+1 << ").position.back(): " << trajectory_msg.joint_trajectory.points.at(i+1).positions.back() << std::endl;
				}
			}
			
			// add once and for all the hand joint names
			if ( i==0 )
			{
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_knuckle_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_finger_12_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_finger_13_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_finger_22_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_finger_23_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_thumb_2_joint");
				trajectory_msg.joint_trajectory.joint_names.push_back(request.arm + "_sdh_thumb_3_joint");
				
				// initialize hand joints for the first point from start_state
				for ( int j=trajectory_msg.joint_trajectory.joint_names.size()-7; j<trajectory_msg.joint_trajectory.joint_names.size(); ++j )
				{
					for ( int k=0; k<startJointState.name.size(); ++k )
					{
						if ( !startJointState.name.at(k).compare( trajectory_msg.joint_trajectory.joint_names.at(j) ) )
						{
							trajectory_msg.joint_trajectory.points.at(0).positions.push_back( startJointState.position.at(k) );
							break;
						}
					}
				}
			}
			// add hand joint values
			for ( int j=0; j<7; ++j )
			{
				trajectory_msg.joint_trajectory.points.at(i+1).positions.push_back(current_grasp.grasp_trajectory.at(i).joints.at(j));
			}
			
		}
		
// 		std::cout << "trajectory_msg: " << std::endl << trajectory_msg << std::endl;
		
		// // code from https://github.com/ros-planning/moveit_core/issues/141
		// group.computeCartesianPath(waypoints, 0.01, eef_step, 0.0, trajectory_msg, false);
		robot_trajectory::RobotTrajectory rt(arm_group.getCurrentState()->getRobotModel(), request.arm + "_arm_hand");
		rt.setRobotTrajectoryMsg(*arm_group.getCurrentState(), trajectory_msg);
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		bool success = iptp.computeTimeStamps(rt);
		ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");
		moveit_msgs::RobotTrajectory tmp_msg;
		trajectory_msgs::JointTrajectory original_joint_trajectory = trajectory_msg.joint_trajectory;
		rt.getRobotTrajectoryMsg(trajectory_msg);
		tmp_msg = trajectory_msg;
		
// 		std::cout << "trajectory_msg: " << std::endl << trajectory_msg << std::endl << std::endl << std::endl << std::endl << std::endl;
		// as trajectories before and after the time parameterization are different, correct the new one to
		// have the same joint order as the old one
		for ( int i=0; i<original_joint_trajectory.joint_names.size(); ++i )
		{
			// if the names are not exactly in the same order (compare != 0)
			if ( trajectory_msg.joint_trajectory.joint_names.at(i).compare( original_joint_trajectory.joint_names.at(i) ) )
			{
				for ( int j=0; j<trajectory_msg.joint_trajectory.joint_names.size(); ++j )
				{
					// if output_name[j] == name[i]
					if ( !trajectory_msg.joint_trajectory.joint_names.at(j).compare( original_joint_trajectory.joint_names.at(i) ) )
					{
						// replace all joint values at index i with their new values at index j - do this for all points in the trajectory
						// trajectory_msg[k].all[i] = tmp_msg[k].all[j]
						for ( int k=0; k<original_joint_trajectory.points.size(); ++k )
						{
							// std::cout << "tmp_msg.joint_trajectory.points.at(k).positions.at(j): " << tmp_msg.joint_trajectory.points.at(k).positions.at(j) << std::endl;
							// std::cout << "trajectory_msg.joint_trajectory.points.at(k).positions.at(i): " << trajectory_msg.joint_trajectory.points.at(k).positions.at(i) << std::endl;
							trajectory_msg.joint_trajectory.points.at(k).accelerations.at(i) = tmp_msg.joint_trajectory.points.at(k).accelerations.at(j);
							// trajectory_msg.joint_trajectory.points.at(k).effort.at(i) = tmp_msg.joint_trajectory.points.at(k).effort.at(j);
							trajectory_msg.joint_trajectory.points.at(k).positions.at(i) = tmp_msg.joint_trajectory.points.at(k).positions.at(j);
							trajectory_msg.joint_trajectory.points.at(k).velocities.at(i) = tmp_msg.joint_trajectory.points.at(k).velocities.at(j);
						}
						break;
					}
				}
			}
		}
		// in the end, put the new points in the trajectory_msg which will be used
		trajectory_msg.joint_trajectory.joint_names = original_joint_trajectory.joint_names;
		
		motion_plan.response.motion_plan_response.trajectory = trajectory_msg;
		
		// check results 
		if(success) 
		{
			moveit_msgs::MotionPlanResponse &motion_plan_response = motion_plan.response.motion_plan_response;
			int trajSize = (int)motion_plan_response.trajectory.joint_trajectory.points.size();

			ROS_INFO("Planning completed with code %d", motion_plan_response.error_code.val);
			ROS_INFO("Planning took %.2fs", motion_plan_response.planning_time);

			ROS_INFO("Trajectory contains %d points.", trajSize);

			definitions::Trajectory trajectory;
			trajectory.trajectory_id = 0;

			moveit_msgs::RobotTrajectory robot_trajectory = motion_plan_response.trajectory;
			
			// populate trajectory with motion plan data
			// the start state is used to copy the data for the joints that are not being used in the planning			
			pacman::convertLimb(robot_trajectory, trajectory, start_state.joint_state, request.arm);

			trajectories.push_back(trajectory);

			response.result = response.SUCCESS;
			return true;
		} 
		else 
		{
			ROS_WARN("No solution found");
			response.result = response.NO_FEASIBLE_TRAJECTORY_FOUND;
			return false;
		}
		
// // Possibility 3: rely on computeCartesianPath function to find a path for the arm, but then this
// // needs to be aligned with the one fom the hand and then together they have to be time-parameterized
// 		// now we plan move without checking for collision from the pre-grasp to the grasp
// 		std::vector<geometry_msgs::Pose> grasp_waypoints;
// 
// 		// if we succeded, it means we already planned until the pre-grasp
// 		// so, read the grasp definition, the grasp is the n-1 wrist_pose, so we would like to move until there
// 		std::cout << "grasp_trajectory size" << current_grasp.grasp_trajectory.size() << std::endl;
// 		for (int g = 0; g < current_grasp.grasp_trajectory.size(); g++ )
// 		{
// 			grasp_waypoints.push_back(current_grasp.grasp_trajectory[g].wrist_pose.pose);
// 		}
// 
// // 		// set the start state as the last way point of the pre-grasp trajectory
// // 		arm_group.setStartStateToCurrentState ();
// 
// 		moveit_msgs::RobotTrajectory grasp_trajectory;
// 		bool avoid_collisions = false;
// 
// 		double fraction = arm_group.computeCartesianPath(grasp_waypoints, eef_step_, jump_threshold_, grasp_trajectory, avoid_collisions);
// 
// 		if (fraction < 1.0)
// 		{
// 			ROS_ERROR("Only %f part of the path was computed.", fraction);
// 			ROS_ERROR("The goal configuration can not be reached: no IK solution");
// 			return false;
// 		}
// 		
// 		if(trajectory_processing::isTrajectoryEmpty(grasp_trajectory))
//   		{
//   			ROS_ERROR("Couldn't find a nice path for the PICK");
//   			return false;
//   		}
//   		else
//   		{
//   			ROS_INFO("Sucessfully found a good path for the PICK");
// 
// 			if ( grasp_trajectory.joint_trajectory.points.size() != grasp_waypoints.size() )
// 			{
// 				ROS_ERROR( "I don't know what to do when I receive more waypoints than I asked for..." );
// 				request.response = request.OTHER_ERROR;
// 				return false;
// 			}
// 			else
// 			{
// 				// use the arm joints from grasp_trajectory and hand joints from current_grasp to set-up
// 				// path constraints for the trajectory planning
// 			}
// 			
// 			
// 		}

	}
	else
	{
		ROS_INFO("I can't process this request! Check the request type");
	}

}

} // namespace trajectory_planner_moveit
