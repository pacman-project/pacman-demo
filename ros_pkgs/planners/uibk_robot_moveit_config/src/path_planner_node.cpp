/*
 * PlanExecutionNode.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: martin
 */



typedef boost::shared_ptr<moveit_msgs::MotionPlanResponse> MotionPlanPtr;

#define MAX_TRAJ_PTS 50
#define PLAN_ATTEMPTS 2
#define MAX_PLAN_TIME 1

using namespace moveit_msgs;
using namespace kinematic_constraints;

ServiceClient planning_client;
ServiceClient execution_client;

ServiceServer planning_server;
ServiceServer execution_server;
// caches all planned trajectories for later execution
vector<MotionPlanPtr> motion_plans;

/**
 * Convenience method to fill in the trajectory data from the given MotionPlanResponse into
 * the given Trajectory
 *
 * @param plan
 * @param trajectory
 */
void MotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory) {
	const vector<trajectory_msgs::JointTrajectoryPoint> &points = plan.trajectory.joint_trajectory.points;

	// pick each point from the trajectory and create a UIBKRobot object
	for (size_t i = 0; i < points.size(); ++i) {
		definitions::UIBKRobot robot_point;
		robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
		trajectory.robot_path.push_back(robot_point);
	}
}
/**
 * Plan a trajectory for given pose goal and insert the trajectory into given vector of trajectories.
 * Returns true in case of success. Also store a Pointer to the motion plan for later usage.
 *
 * @param trajectories
 * @param goal
 * @return
 */
bool planTrajectory(vector<Trajectory> &trajectories, geometry_msgs::PoseStamped &goal) {

	GetMotionPlan mp;
	MotionPlanRequest &mp_request = mp.request.motion_plan_request;

	// should not be hardcoded...
	mp_request.group_name = "right_arm";

	// ensure that the frame id of our goal is set to the world reference frame
	goal.header.frame_id = "world_link";

	Constraints pose_goal = constructGoalConstraints("right_arm_7_link", goal, 0.001, 0.001);

	mp_request.goal_constraints.push_back(pose_goal);
	// play with theese values to achieve better performance and effizient motion plans...
	mp_request.num_planning_attempts = PLAN_ATTEMPTS;
	mp_request.allowed_planning_time = MAX_PLAN_TIME;

	ROS_INFO("Calling plannig service...");
	bool success = planning_client.call(mp);

	if(success) {
		MotionPlanResponse &mp_response = mp.response.motion_plan_response;
		int trajSize = (int)mp_response.trajectory.joint_trajectory.points.size();

		ROS_INFO("Planning completed with code %d", mp_response.error_code.val);
		ROS_INFO("Planning took %.2fs", mp_response.planning_time);

		if(trajSize > MAX_TRAJ_PTS) {
			ROS_WARN("Computed trajectory contains to many points, so it will be dropped!");
			return false;
		} else {
			ROS_INFO("Trajectory contains %d points.", trajSize);
			// store the motion plan for later usage
			int index = motion_plans.size();
			MotionPlanPtr motion_plan_ptr(new MotionPlanResponse(mp_response));
			motion_plans.push_back(motion_plan_ptr);

			Trajectory trajectory;
			trajectory.trajectory_id = index;
			// populate trajectory with motion plan data
			MotionPlanToTrajectory(mp_response, trajectory);

			trajectories.push_back(trajectory);
			return true;
		}

	} else {
		ROS_WARN("No solution found");
		return false;
	}
}
/**
 * Callback method for the trajectory planning service server
 *
 * @param request
 * @param response
 * @return
 */
bool PlanningServiceCB(TrajectoryPlanning::Request &request,
						TrajectoryPlanning::Response &response) {

	ROS_INFO("Received trajectory planning request");
	ros::Time start_time = ros::Time::now();

	// clear all previously cached motion plans
	motion_plans.clear();
	vector<Trajectory> &trajectories = response.trajectory;

	for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {

		geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
		// ensure that the frame id is set to the world reference frame
		goal.header.frame_id = "world_link";

		if(!planTrajectory(trajectories, goal)) {
			ROS_WARN("No trajectory found for grasp %d", (int)i);
		}
	}

	if(trajectories.size() > 0) {
		response.result = TrajectoryPlanning::Response::SUCCESS;
	} else {
		response.result = TrajectoryPlanning::Response::NO_FEASIBLE_TRAJECTORY_FOUND;
	}

	ros::Duration duration = ros::Time::now() - start_time;

	ROS_INFO("Trajectory planning request completed");
	ROS_INFO_STREAM("Total trajectory calculation took " << duration);

	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "plan_execution_node");
	ros::NodeHandle nh;

	ROS_INFO("Launching plan_executrion_node");

	string planning_service_name = "/plan_kinematic_path";

	ROS_INFO("Waiting for 'plan_kinematic_path' service node...");
	ros::service::waitForService(planning_service_name, -1);
	planning_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

	planning_server = nh.advertiseService("/trajectory_planner_srv", PlanningServiceCB);

	ROS_INFO("Connected!");

	ros::spin();

	return EXIT_SUCCESS;
}



















//// NEW STRUCTURE 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped>

#include <definitions/TrajectoryPlanning.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

#include <boost/shared_ptr.hpp> 


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
    ros::ServiceServer 
    //ros::ServiceServer srv_set_parameters_;



    // it is very useful to have a listener and broadcaster to know where all frames are
    //tf::TransformListener tf_listener_;
    //tf::TransformBroadcaster tf_broadcaster_;

    // conversion function
    convertFromMotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory);
    
  public:

    // callback functions
    //void doClassComputations(const X_msgs::Wrench & msg);
    //bool setClassParameters(ClassParameterSetting::Request &request, ClassParameterSetting::Response &response);
    //void doSomething();

  	// the service callback 
  	bool PlanningServiceCB(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

  	// the actual planning function
    bool planTrajectory(std::vector<Trajectory> &trajectories, geometry_msgs::PoseStamped &goal);

    // constructor
    PathPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

		// wait for moveit to load
		string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		ros::ServiceClient planning_client = node.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		#define MAX_TRAJ_PTS 50
#define PLAN_ATTEMPTS 2
#define MAX_PLAN_TIME 1

        // // subscribe to topics
        // sub_some_node_messages_ = nh_.subscribe(nh_.resolveName("topic_name_you_want_to_subscribe_to"), 10, &PathPlanner::doClassComputations, this);

        // // advertise topics
        // pub_class_postprocessing_info_ =
        //   nh_.advertise<type_msgs::MessageType>(nh_.resolveName("topic_name_you_want_to_pusblih"), 10);
        
        // // advertise service
        // srv_set_parameters_ = 
        //   nh_.advertiseService(nh_.resolveName("service_name_you_want_to_advertise"), 
        //                                                 &PathPlanner::setClassParameterss, this);
    }

    //! Empty stub
    ~PathPlanner() {}

};

void convertFromMotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory)
{
	const vector<trajectory_msgs::JointTrajectoryPoint> &points = plan.trajectory.joint_trajectory.points;

	// pick each point from the trajectory and create a UIBKRobot object
	for (size_t i = 0; i < points.size(); ++i) {
		definitions::UIBKRobot robot_point;
		robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
		trajectory.robot_path.push_back(robot_point);
	}
}

bool planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	return true;

}


} // namespace uibk_robot_moveit_config


int main(int argc, char **argv)
{
    ros::init(argc, argv, "name_performer_node");
    ros::NodeHandle nh;

    uibk_robot_moveit_config::PathPlanner node(nh);

    ROS_INFO("This node is ready to do Path Planning!")

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
