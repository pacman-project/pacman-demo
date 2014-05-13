//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "CartPlanner.h"

#include <pacman/PaCMan/ROS.h>
#include <pacman/PaCMan/Defs.h>

namespace trajectory_planner_moveit {


bool CartPlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	if( request.type == request.MOVE_TO_CART_GOAL)
	{
		ROS_INFO("Received cartesian planning request");
		ros::Time now = ros::Time::now();

		// this is were the response will be filled
		std::vector<definitions::Trajectory> &trajectories = response.trajectory;

		// wait for a joint state to read the current state
		boost::shared_ptr<const sensor_msgs::JointState> current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic_, ros::Duration(3.0));
		if (!current_state_ptr)
		{
			ROS_ERROR("No real start state available, since no joint state recevied in topic: %s", topic_.c_str());
			ROS_ERROR("Did you forget to start a controller?");
			ROS_ERROR("Planning will be done from home position, however this trajectory might not be good for execution!");
		}
		sensor_msgs::JointState startJointState = *current_state_ptr;
		moveit_msgs::RobotState start_state;
		start_state.joint_state = startJointState;

		// plan for the given wrist goal
		definitions::SDHand goal_hand;

		// note that we plan for wrist frame of the requested arm
		if(request.arm.compare(std::string("right")) == 0)
			goal_hand = request.eddie_goal_state.handRight;
		if(request.arm.compare(std::string("left")) == 0)
			goal_hand = request.eddie_goal_state.handLeft;

		// set the group
		group_name_ = request.arm + "_arm";
		plan_for_frame_ = request.arm + "_sdh_palm_link";
		move_group_interface::MoveGroup arm_group( group_name_.c_str() );

		// set the goal
		arm_group.setEndEffectorLink( plan_for_frame_ );

		/// PATH APPROACH; IT MISSES THE TRAJECTORY COMPUTATION
		// now we plan move to the goal pose
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(goal_hand.wrist_pose.pose);

		moveit_msgs::RobotTrajectory moveit_trajectory;
		bool avoid_collisions = true;

		// and compute the cartesian path
		// fraction means the fraction of the path covered
		// since we have only one waypoint, the fraction must be on or very close to one
		// if not, it means it couldn't find a safe goal position
		double fraction = arm_group.computeCartesianPath(waypoints, eef_step_, jump_threshold_, moveit_trajectory, avoid_collisions);

		if (fraction < 1.0)
		{
			ROS_ERROR("Only %f part of the path was computed.", fraction);
			ROS_ERROR("The goal configuration can not be reached: no IK solution");
			return false;
		}


		// setting the planner and general parameters
		arm_group.setPlannerId("PRMstarkConfigDefault");
		arm_group.setGoalPositionTolerance(tolerance_in_position_);
		arm_group.setGoalOrientationTolerance(tolerance_in_orientation_);
		arm_group.setPlanningTime(max_planning_time_);

		// set the current state
		arm_group.setStartState(start_state);


  		if(trajectory_processing::isTrajectoryEmpty(moveit_trajectory))
  		{
  			ROS_ERROR("Couldn't find a nice goal position for the MOVE_TO_CART_GOAL");
  			return false;
  		}
  		else
  		{
  			ROS_INFO("Sucessfully found a good goal position for the MOVE_TO_CART_GOAL");

			//display the path to check that the found goal position is good
			moveit_msgs::DisplayTrajectory display_trajectory;
			display_trajectory.trajectory_start = start_state;
	  		display_trajectory.trajectory.push_back(moveit_trajectory);
	  		display_publisher_.publish(display_trajectory);
		
			trajectory_msgs::JointTrajectoryPoint goal_point;

			goal_point = moveit_trajectory.joint_trajectory.points[moveit_trajectory.joint_trajectory.points.size()-1];

			// build the joint constraint out of the goal point
			moveit_msgs::Constraints goal_state;
			moveit_msgs::JointConstraint joint_constraint;
			for (int i=0; i<goal_point.positions.size(); i++ )
			{
				joint_constraint.joint_name = moveit_trajectory.joint_trajectory.joint_names[i];
				joint_constraint.position = goal_point.positions[i];
				joint_constraint.weight = 1.0;
				goal_state.joint_constraints.push_back(joint_constraint);
			}
			
			if( !planTrajectory(trajectories, goal_state, request.arm, startJointState, goal_hand) ) 
			{
				ROS_WARN("No trajectory found for the required goal state");
			}

			//ROS_INFO("trajectories.size() %lu",trajectories.size());
			if(trajectories.size() > 0)
			{
				response.result = response.SUCCESS;
				ros::Duration duration = ros::Time::now() - now;

				ROS_INFO("Trajectory planning request completed");
				ROS_INFO_STREAM("Total trajectory calculation took " << duration);
				return true;
			} 
			else 
			{
				response.result = response.NO_FEASIBLE_TRAJECTORY_FOUND;
				return false;
			}
  		}

	}
	else
	{
		ROS_INFO("I can't process this request! Check the request type");
		return false;
	}

}

void CartPlanner::callback_collision_object(const moveit_msgs::AttachedCollisionObject &object)
{
  collision_objects_.clear();
  collision_objects_.push_back(object);   
}

bool CartPlanner::planTrajectory(std::vector<definitions::Trajectory> &trajectories, moveit_msgs::Constraints &goal, std::string &arm, sensor_msgs::JointState &startState, definitions::SDHand &goal_hand) 
{
	// first state the planning constraint
	//moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(plan_for_frame_.c_str(), goal.wrist_pose, tolerance_in_position_, tolerance_in_orientation_);

	//ROS_INFO("Planning for wrist (px, py, pz, qx, qy, qz, qw):\t%f\t%f\t%f\t%f\t%f\t%f\t%f", goal.wrist_pose.pose.position.x, goal.wrist_pose.pose.position.y, goal.wrist_pose.pose.position.z, goal.wrist_pose.pose.orientation.x, goal.wrist_pose.pose.orientation.y, goal.wrist_pose.pose.orientation.z, goal.wrist_pose.pose.orientation.w);
	
	// now construct the motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;

	// constraint for the wrist
	motion_plan_request.group_name = group_name_.c_str();
	motion_plan_request.goal_constraints.push_back(goal);
	//motion_plan_request.goal_constraints[0].joint_constraints = handJointsGoal;

	motion_plan_request.num_planning_attempts = max_planning_attempts_;
	motion_plan_request.allowed_planning_time = max_planning_time_;
	motion_plan_request.planner_id = "PRMstarkConfigDefault";

	moveit_msgs::RobotState start_state;
	start_state.joint_state = startState;
	start_state.attached_collision_objects = collision_objects_;
	collision_objects_.clear();
	motion_plan_request.start_state = start_state;


	// and call the service with the request
	ROS_INFO("Calling plannig service...");
	bool success = clt_moveit_planning_.call(motion_plan);

	// check results 
	if(success) 
	{
		moveit_msgs::MotionPlanResponse &motion_plan_response = motion_plan.response.motion_plan_response;
		int trajSize = (int)motion_plan_response.trajectory.joint_trajectory.points.size();

		ROS_INFO("Planning completed with code %d", motion_plan_response.error_code.val);
		ROS_INFO("Planning took %.2fs", motion_plan_response.planning_time);

		if(trajSize > max_points_in_trajectory_)
		{
			ROS_WARN("Computed trajectory contains too many points, so it will be dropped!");
			return false;
		} 
		else 
		{
			ROS_INFO("Trajectory contains %d points.", trajSize);

			definitions::Trajectory trajectory;
			trajectory.trajectory_id = 0;

			moveit_msgs::RobotTrajectory robot_trajectory = motion_plan.response.motion_plan_response.trajectory;

			// fill the robot trajectory with hand values, given the base trajectory of the arm
			if( trajSize <= min_traj_size_ )
			  pacman::interpolateHandJoints(goal_hand, startState, robot_trajectory, arm,false);
            else
              pacman::interpolateHandJoints(goal_hand, startState, robot_trajectory, arm);			
			// populate trajectory with motion plan data
			// the start state is used to copy the data for the joints that are not being used in the planning
			pacman::convertLimb(robot_trajectory, trajectory, startState, arm);

			trajectories.push_back(trajectory);

			return true;
		}
	} 
	else 
	{
		ROS_WARN("No solution found");
		return false;
	}
}


} // namespace trajectory_planner_moveit
