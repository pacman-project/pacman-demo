//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

//// local headers
#include <definitions/TrajectoryPlanning.h>

namespace uibk_robot_moveit_config {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class PathPlanner
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    //ros::Subscriber sub_some_node_messages_;

    // publishers
    //ros::Publisher pub_class_postprocessing_info_;
    
    // services
    ros::ServiceServer srv_trajectory_planning_;
    //ros::ServiceServer srv_test_trajectory_planning_;

    // clients
    ros::ServiceClient clt_moveit_planning_;

	// planning related parameters
	int max_points_in_trajectory_;
	int max_planning_attempts_;
	int max_planning_time_;
	double tolerance_in_position_;
	double tolerance_in_orientation_;

    // define the names passed in the urdf files corresponding to the current move group for planning
    std::string group_name_;
    std::string base_frame_for_goal_;
	std::string plan_for_frame_;

	// the variable where the plans are stored
	std::vector<moveit_msgs::MotionPlanResponse> motion_plans_;

    // conversion function
    void convertFromMRobotTrajectoryToTrajectory(const moveit_msgs::RobotTrajectory &moveitTraj, definitions::Trajectory &trajectory);
    
  public:

  	// the service callback 
  	bool planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

  	// the actual planning function
    bool planTrajectory(std::vector<definitions::Trajectory> &trajectories, geometry_msgs::PoseStamped &goal);

    // constructor
    PathPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

		// wait for moveit to load
		std::string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		clt_moveit_planning_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		// planning related parameters
		nh_.param<int>("max_points_in_trajectory", max_points_in_trajectory_, 50);
		nh_.param<int>("max_planning_attempts", max_planning_attempts_, 2);
		nh_.param<int>("max_planning_time", max_planning_time_, 2);
		nh_.param<double>("tolerance_in_position", tolerance_in_position_, 0.1);
		nh_.param<double>("tolerance_in_orientation", tolerance_in_orientation_, 0.1);

        // define the names passed in the urdf files corresponding to the current move group for planning
        nh_.param<std::string>("arm_name", group_name_, "right_arm");
        nh_.param<std::string>("base_frame_for_goal", base_frame_for_goal_, "world_link");
		nh_.param<std::string>("plan_for_frame", plan_for_frame_, "right_sdh_palm_link");

		srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/trajectory_planning_srv"),&PathPlanner::planTrajectoryFromCode, this);
		//srv_test_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/test_trajectory_planning_srv"),&PathPlanner::planTrajectoryFromCode, this);

    }

    //! Empty stub
    ~PathPlanner() {}

};

void PathPlanner::convertFromMRobotTrajectoryToTrajectory(const moveit_msgs::RobotTrajectory &moveitTraj, definitions::Trajectory &trajectory)
{
	// get the points from robot trajector
	const std::vector<trajectory_msgs::JointTrajectoryPoint> &points = moveitTraj.joint_trajectory.points;

	// pick each point from the trajectory and create a UIBKRobot object
	for (size_t i = 0; i < points.size(); ++i) {
		definitions::UIBKRobot robot_point;
		robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
		
		for(int h = 0; h < 6; h++)
		 	robot_point.hand.joints.push_back(0);

		trajectory.robot_path.push_back(robot_point);

		if (i == 0)
		{
			trajectory.time_from_previous.push_back( ros::Duration().fromSec(0.) );	
		}
		else
		{
			// the RobotTrajectory gives time_from_start, we prefer from previous for easier transformation to pacman commands
			trajectory.time_from_previous.push_back( points[i].time_from_start - points[i-1].time_from_start );
		}
	}
}

bool PathPlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	ROS_INFO("Received trajectory planning request");
	ros::Time now = ros::Time::now();

	// clear all previously cached motion plans
	motion_plans_.clear();
	std::vector<definitions::Trajectory> &trajectories = response.trajectory;

	for(size_t i = 0; i < request.ordered_grasp.size(); ++i) 
	{

		// note that we plan for the first element [0] of the grasp trajectory of the i-th object grasp
		geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
		goal.header.frame_id = base_frame_for_goal_.c_str();

		if( !planTrajectory(trajectories, goal) ) 
		{
			ROS_WARN("No trajectory found for grasp %d", (int)i);
		}
		ros::Duration(5).sleep();
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

bool PathPlanner::planTrajectory(std::vector<definitions::Trajectory> &trajectories, geometry_msgs::PoseStamped &goal) 
{
	// first state the planning constraint
	goal.header.frame_id = base_frame_for_goal_.c_str(); // just to ensure we plan with respect to the base_frame
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(plan_for_frame_.c_str(), goal, tolerance_in_position_, tolerance_in_orientation_);

	ROS_INFO("Planning for wrist (px, py, pz, qx, qy, qz, qw):\t%f\t%f\t%f\t%f\t%f\t%f\t%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
	

	// now construct the motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;

	motion_plan_request.group_name = group_name_.c_str();
	motion_plan_request.goal_constraints.push_back(pose_goal);
	motion_plan_request.num_planning_attempts = max_planning_attempts_;
	motion_plan_request.allowed_planning_time = max_planning_time_;

	// if 

	// wait for a joint state
	std::string topic = nh_.resolveName("/joint_states");
	boost::shared_ptr<const sensor_msgs::JointState> current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(3.0));
	if (!current_state_ptr)
	{
		ROS_ERROR("No real start state available, since no joint state recevied in topic: %s", topic.c_str());
		ROS_WARN("Did you forget to start a controller?");
		ROS_WARN("Planning will be done from home position, however this trajectory might not be good for execution!");
	}
	else
	{
		// take the current state of the robot as the start state
		moveit_msgs::RobotState start_state;
		start_state.joint_state = *current_state_ptr;
		motion_plan_request.start_state = start_state;
	}

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

			// // store the motion plan for later usage
			// int index = motion_plans.size();
			// moveit_msgs::MotionPlanResponse response = motion_plan.response.motion_plan_response;
			// motion_plans_.push_back(response);

			definitions::Trajectory trajectory;
			trajectory.trajectory_id = 0;

			moveit_msgs::RobotTrajectory robot_trajectory = motion_plan.response.motion_plan_response.trajectory;

			// populate trajectory with motion plan data
			convertFromMRobotTrajectoryToTrajectory(robot_trajectory, trajectory);
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


} // namespace uibk_robot_moveit_config


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    uibk_robot_moveit_config::PathPlanner node(nh);

    ROS_INFO("This node is ready to do path planning!");

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
