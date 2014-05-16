//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayRobotState.h>

//// local headers
#include <definitions/TrajectoryPlanning.h>
#include "KinematicsHelper.h"

namespace trajectory_planner_moveit {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class PickPlanner
{
  private:

    // clients
    ros::ServiceClient clt_moveit_planning_;

	// the variable where the plans are stored
	std::vector<moveit_msgs::MotionPlanResponse> motion_plans_;
	
	// kinematics helper object
	KinematicsHelper ki_helper_;
   
  public:

  	// planning related parameters
	int max_points_in_trajectory_;
	int max_planning_attempts_;
	int max_planning_time_;
	double tolerance_in_position_;
	double tolerance_in_orientation_;
	double eef_step_;
	double jump_threshold_;

    // define the names passed in the urdf files corresponding to the current move group for planning
    std::string group_name_;
    std::string base_frame_for_goal_;
	std::string plan_for_frame_;

	// joint state topic
	std::string topic_;

	// speed scale for trajectory
	double speed_;
	
	// added for visualization purposes only
	ros::Publisher display_publisher_;

  	// the service callback 
  	bool planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

    // constructor
    PickPlanner(ros::NodeHandle nh) : ki_helper_(nh)
    {

		// wait for moveit to load
		std::string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		clt_moveit_planning_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		// planning related parameters default
		max_planning_attempts_ = 100;
		max_planning_time_ = 10;
		tolerance_in_position_ = 0.01;
		tolerance_in_orientation_ = 0.01;
		// use eef_step_==10 to only obtain desired waypoints joint configurations and not other waypoints we should discard anyway
		// (in some complicated manner)
		eef_step_ = 10; // to avoid jumps in the cartesian interpolation
		jump_threshold_ = 5.0; // to avoid jumps in the ik solution

		// joint state topic
		topic_ = nh.resolveName("/joint_states");
		
// 		display_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		display_publisher_ = nh.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1, true);
    }

    //! Empty stub
    ~PickPlanner() {}

};

} // namespace trajectory_planner_moveit
