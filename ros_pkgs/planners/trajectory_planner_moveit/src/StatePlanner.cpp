//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "StatePlanner.h"

#include <pacman/PaCMan/ROS.h>
#include <pacman/PaCMan/Defs.h>

namespace trajectory_planner_moveit {

bool StatePlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	if( request.type == request.MOVE_TO_STATE_GOAL)
	{
		ROS_INFO("Received trajectory planning request");
		ros::Time now = ros::Time::now();

		// clear all previously cached motion plans
		std::vector<definitions::Trajectory> &trajectories = response.trajectory;

		// wait for a joint state
		boost::shared_ptr<const sensor_msgs::JointState> current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic_, ros::Duration(3.0));
		if (!current_state_ptr)
		{
			ROS_WARN("No real start state available, since no joint state recevied in topic: %s", topic_.c_str());
			ROS_WARN("Did you forget to start a controller?");
			ROS_WARN("Planning will be done from home position, however this trajectory might not be good for execution!");
		}

		sensor_msgs::JointState startState = *current_state_ptr;

		group_name_ = request.arm + "_arm";

		if( !planTrajectory(trajectories, request.eddie_goal_state, request.arm, startState) ) 
		{
			ROS_WARN("No trajectory found for the required goal state");
		}

		ROS_INFO("trajectories.size() %lu",trajectories.size());
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
	else
	{
		ROS_INFO("I can't process this request! Check the request type");
	}

}

bool StatePlanner::planTrajectory(std::vector<definitions::Trajectory> &trajectories, definitions::RobotEddie &goal, std::string &arm, sensor_msgs::JointState &startState) 
{
	// first state the planning constraint
	std::vector<moveit_msgs::JointConstraint> joint_constraints;
	for (int i = 0; i < pacman::KukaLWR::Config::JOINTS; i++)
	{
		moveit_msgs::JointConstraint joint_constraint;

		std::string joint_name = arm + "_arm_" + std::to_string(i) + "_joint";
		joint_constraint.joint_name = joint_name;

		if(arm.compare(std::string("right")) == 0)
			joint_constraint.position = goal.armRight.joints[i];
		if(arm.compare(std::string("left")) == 0)
			joint_constraint.position = goal.armLeft.joints[i];

		joint_constraints.push_back(joint_constraint);
	}
	
	// now construct the motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;

	// constraint for the wrist
	motion_plan_request.group_name = group_name_.c_str();
	motion_plan_request.goal_constraints.resize(1);
	motion_plan_request.goal_constraints[0].joint_constraints = joint_constraints;

	motion_plan_request.num_planning_attempts = max_planning_attempts_;
	motion_plan_request.allowed_planning_time = max_planning_time_;
	motion_plan_request.planner_id = "PRMstarkConfigDefault";

	moveit_msgs::RobotState start_state;
	start_state.joint_state = startState;
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

			definitions::SDHand my_hand;
			if(arm.compare(std::string("right")) == 0)
				my_hand = goal.handRight;
			if(arm.compare(std::string("left")) == 0)
				my_hand = goal.handLeft;

			// fill the robot trajectory with hand values, given the base trajectory of the arm
			if( trajSize <= min_traj_size_ )
			  pacman::interpolateHandJoints(my_hand, startState, robot_trajectory, arm,false);
            else
              pacman::interpolateHandJoints(my_hand, startState, robot_trajectory, arm);
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
