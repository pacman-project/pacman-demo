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
#include "CartPlanner_uibk.h"
#include <conversions.h>

#include <pacman/PaCMan/ROS.h>
#include <pacman/PaCMan/Defs.h>

namespace trajectory_planner_moveit {


bool CartPlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	if( request.type == request.MOVE_TO_CART_GOAL)
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

		definitions::SDHand goal;
		// note that we plan for wrist frame of the requested arm
		if(request.arm.compare(std::string("right")) == 0)
			goal = request.eddie_goal_state.handRight;
		if(request.arm.compare(std::string("left")) == 0)
			goal = request.eddie_goal_state.handLeft;

		plan_for_frame_ = request.arm + "_sdh_palm_link";
		group_name_ = request.arm + "_arm";

		if( !planTrajectory(trajectories, goal, request.arm, startState) ) 
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

void CartPlanner::callback_collision_object(const moveit_msgs::AttachedCollisionObject &object)
{
  collision_objects_.clear();
  collision_objects_.push_back(object);   
}

bool CartPlanner::planTrajectory(std::vector<definitions::Trajectory> &trajectories, definitions::SDHand &goal, std::string &arm, sensor_msgs::JointState &startState) 
{
	ROS_INFO("Planning for wrist (px, py, pz, qx, qy, qz, qw):\t%f\t%f\t%f\t%f\t%f\t%f\t%f", goal.wrist_pose.pose.position.x, goal.wrist_pose.pose.position.y, goal.wrist_pose.pose.position.z, goal.wrist_pose.pose.orientation.x, goal.wrist_pose.pose.orientation.y, goal.wrist_pose.pose.orientation.z, goal.wrist_pose.pose.orientation.w);

	// construct the motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;

	// first state the planning constraint
	ROS_DEBUG("Computing possible IK solutions for goal pose");

	vector<string> joint_names;
	getArmJointNames(arm, joint_names);

	// compute a set of ik solutions and construct goal constraint
	for (int i = 0; i < 5; ++i) {
		moveit_msgs::RobotState ik_solution;

		geometry_msgs::PoseStamped pose_goal = goal.wrist_pose;

		if(ki_helper_.computeIK(arm, pose_goal, startState, ik_solution, plan_for_frame_)) {
			vector<double> values;
			getJointPositionsFromState(joint_names, ik_solution, values);

			moveit_msgs::Constraints c;
			c.joint_constraints.resize(joint_names.size());

			for (int j = 0; j < joint_names.size(); ++j) {
				moveit_msgs::JointConstraint &jc = c.joint_constraints[j];
				jc.joint_name = joint_names[j];
				jc.position = values[j];
				jc.tolerance_above = 1e-4;
				jc.tolerance_below = 1e-4;
				jc.weight = 1.0;
			}
			motion_plan_request.goal_constraints.push_back(c);
		}
	}

	if(motion_plan_request.goal_constraints.size() == 0) {
		ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
		return false;
	}

	// constraint for the wrist
	motion_plan_request.group_name = group_name_.c_str();

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
			//pacman::interpolateHandJoints(goal, startState, robot_trajectory, arm);	
			if( trajSize <= min_traj_size_ )
			  pacman::interpolateHandJoints(goal, startState, robot_trajectory, arm,false);
            else
              pacman::interpolateHandJoints(goal, startState, robot_trajectory, arm);					
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
