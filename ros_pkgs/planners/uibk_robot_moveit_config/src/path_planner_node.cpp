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
    ros::ServiceServer srv_test_trajectory_planning_;

    // clients
    ros::ServiceClient clt_moveit_planning_;

	// planning related parameters
	int max_points_in_trajectory_;
	int max_planning_attempts_;
	int max_planning_time_;

    // define the names passed in the urdf files corresponding to the current move group for planning
    std::string group_name_;
    std::string base_frame_for_goal_;
	std::string plan_for_frame_;

	// the variable where the plans are stored
	std::vector<moveit_msgs::MotionPlanPtr> motion_plans_;

    // conversion function
    convertFromMotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory);
    
  public:

  	// the service callback 
  	bool PlanningServiceCB(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

  	// the actual planning function
    bool planTrajectory(std::vector<Trajectory> &trajectories, geometry_msgs::PoseStamped &goal);

    // constructor
    PathPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

		// wait for moveit to load
		std::string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		clt_moveit_planning_ = this.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		// planning related parameters
		priv_nh_.param<int>("max_points_in_trajectory", max_points_in_trajectory_, "50");
		priv_nh_.param<int>("max_planning_attempts", max_planning_attempts_, "2");
		priv_nh_.param<int>("max_planning_time", max_planning_time_, "2");

        // define the names passed in the urdf files corresponding to the current move group for planning
        priv_nh_.param<std::string>("arm_name", group_name_, "right_arm");
        priv_nh_.param<std::string>("base_frame_for_goal", base_frame_for_goal_, "world_link");
		priv_nh_.param<std::string>("plan_for_frame", plan_for_frame_, "right_sdh_palm_link");

		srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/trajectory_planning_srv"),&GolemController::planTrajectoryFromCode, this);
		srv_test_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/test_trajectory_planning_srv"),&GolemController::planTrajectoryFromCode, this);

    }

    //! Empty stub
    ~PathPlanner() {}

};

void PathPlanner::convertFromMotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory)
{
	const std::vector<trajectory_msgs::JointTrajectoryPoint> &points = plan.trajectory.joint_trajectory.points;

	// pick each point from the trajectory and create a UIBKRobot object
	for (size_t i = 0; i < points.size(); ++i) {
		definitions::UIBKRobot robot_point;
		robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
		trajectory.robot_path.push_back(robot_point);
	}
}

bool PathPlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	ROS_INFO("Received trajectory planning request");
	ros::Time now = ros::Time::now();

	// clear all previously cached motion plans
	motion_plans_.clear();
	std::vector<Trajectory> &trajectories = response.trajectory;

	for(size_t i = 0; i < request.ordered_grasp.size(); ++i) 
	{

		geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
		goal.header.frame_id = base_frame_for_goal_;

		if( !planTrajectory(trajectories, goal) ) 
		{
			ROS_WARN("No trajectory found for grasp %d", (int)i);
		}
	}

	if(trajectories.size() > 0) 
	{
		response.result = TrajectoryPlanning::Response::SUCCESS;
	} else 
	{
		response.result = TrajectoryPlanning::Response::NO_FEASIBLE_TRAJECTORY_FOUND;
	}

	ros::Duration duration = ros::Time::now() - now;

	ROS_INFO("Trajectory planning request completed");
	ROS_INFO_STREAM("Total trajectory calculation took " << duration);

	return true;
}

bool PathPlanner::planTrajectory(std::vector<Trajectory> &trajectories, geometry_msgs::PoseStamped &goal) 
{
	// first state the planning constraint
	goal.header.frame_id = base_frame_for_goal_; // just to ensure we plan with respect to the base_frame
	moveit_msgs::Constraints pose_goal = moveit_msgs::constructGoalConstraints(goal_frame_, goal, 0.001, 0.001);

	// now construct the motion plan request
	moveit_msgs::GetMotionPlan mp;
	moveit_msgs::MotionPlanRequest &mp_request = mp.request.motion_plan_request;
	mp_request.group_name = group_name_;
	mp_request.goal_constraints.push_back(pose_goal);
	mp_request.num_planning_attempts = max_planning_attempts_;
	mp_request.allowed_planning_time = max_planning_time_;

	// and call the service with the request
	ROS_INFO("Calling plannig service...");
	bool success = clt_moveit_planning_.call(mp);

	// check results 
	if(success) 
	{
		MotionPlanResponse &mp_response = mp.response.motion_plan_response;
		int trajSize = (int)mp_response.trajectory.joint_trajectory.points.size();

		ROS_INFO("Planning completed with code %d", mp_response.error_code.val);
		ROS_INFO("Planning took %.2fs", mp_response.planning_time);

		if(trajSize > max_points_in_trajectory_) 
		{
			ROS_WARN("Computed trajectory contains too many points, so it will be dropped!");
			return false;
		} 
		else 
		{
			ROS_INFO("Trajectory contains %d points.", trajSize);

			// store the motion plan for later usage
			int index = motion_plans.size();
			moveit::msgs::MotionPlanPtr motion_plan_ptr(new MotionPlanResponse(mp_response));
			motion_plans.push_back(motion_plan_ptr);

			// populate trajectory with motion plan data
			convertFromMotionPlanToTrajectory(mp_response, trajectory);
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

    ROS_INFO("This node is ready to do path planning!")

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
