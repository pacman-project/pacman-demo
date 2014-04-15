//// system headers
#include <Eigen/Core>
#include <Eigen/Geometry>

//// ros headers


//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "pickupplanner.h"


namespace trajectory_planner_moveit {


PickupPlanner::PickupPlanner(ros::NodeHandle &nh)
{
    // set some default values...
    planning_time_ = 5.0;
    planning_attempts_ = 5;
    goal_joint_tolerance_ = 1e-4;
    goal_position_tolerance_ = 1e-4; // 0.1 mm
    goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
    planner_id_ = "";

    ROS_INFO("Connecting to pickup client...");
    string pickup_topic = "pickup";
    pick_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(pickup_topic, true));
    pick_action_client_->waitForServer();
}

bool PickupPlanner::planPickup(TrajectoryPlanning::Request &request, TrajectoryPlanning::Response &response)
{
    string arm = "right_arm";

    ROS_INFO("Received trajectory planning request.");
    ROS_INFO("Validating received request data...");
    // we have to make sure that sufficient data has been provided to be able to compute a plan...
    if(request.arm.empty()) {
        ROS_WARN("No arm name provided - assuming %s", arm.c_str());
    } else {
        arm = request.arm;
    }

    if(request.ordered_grasp.size() < 3) {
        ROS_ERROR("Not enough grasp data provided. Size of 'request.ordered_grasp' has to be >= 3!");
        response.result = TrajectoryPlanning::Response::OTHER_ERROR;
        return false;
    }

    vector<moveit_msgs::Grasp> grasp_list;
    // make our grasp list the same size as the provided ordered grasps...
    grasp_list.resize(request.ordered_grasp.size());

    for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {
        // provide each grasp with an id...
        stringstream ss;
        ss << "Grasp" << i;
        string id = ss.str();
        // populate moveit_msgs::Grasp at current indes with our grasp data
        DefGraspToMoveitGrasp(id, request.ordered_grasp[i], grasp_list[i]);
    }
    // now we have a vector of grasps and can invoke the planner...

    // TODO: provide the id of the grasped object here...
    string object_id = "";

    // construct the pickup message and send it to the action server
    moveit_msgs::PickupGoal goal_msg;
    constructPickupGoal(arm, object_id, grasp_list, goal_msg);


    pick_action_client_->sendGoal(goal_msg);

    if (!pick_action_client_) {
        ROS_ERROR("Pick action client not found");
        return false;
    }

    if (!pick_action_client_->isServerConnected()) {
        ROS_ERROR("Pick action server not connected");
        return false;
    }

    moveit_msgs::PickupResultConstPtr result = pick_action_client_->getResult();

    if (pick_action_client_->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Call to pick action server succeeded!");
        // convert the computed motion plan to definitions::trajectory
        PickupResultToTrajectory(result, response.trajectory);
        ROS_INFO("Computed trajectory contains %d points", (int)response.trajectory.size());

        response.result = TrajectoryPlanning::Response::SUCCESS;
    } else {
        ROS_WARN_STREAM("Fail: " << pick_action_client_->getState().toString() << ": " << pick_action_client_->getState().getText());
        ROS_WARN_STREAM("Planning failed with status code '" << result->error_code.val << "'");

        response.result = TrajectoryPlanning::Response::NO_FEASIBLE_TRAJECTORY_FOUND;
        return false;
    }

    return true;
}

/**
 * Convenience method to fill in the trajectory data from the given SDHand into
 * the given Trajectory
 *
 * @param t the trajectory message to fill
 * @param hand the SDHand where the data comes from.
 */
void PickupPlanner::fillTrajectory(trajectory_msgs::JointTrajectory &t, const definitions::SDHand &hand) {

    t.header.frame_id = "world_link";
    t.header.stamp = ros::Time::now();
    // Name of joints:
    t.joint_names.push_back("right_sdh_knuckle_joint");
    t.joint_names.push_back("right_sdh_finger_12_joint");
    t.joint_names.push_back("right_sdh_finger_13_joint");
    t.joint_names.push_back("right_sdh_finger_22_joint");
    t.joint_names.push_back("right_sdh_finger_23_joint");
    t.joint_names.push_back("right_sdh_thumb_2_joint");
    t.joint_names.push_back("right_sdh_thumb_3_joint");
    // Position of joints
    trajectory_msgs::JointTrajectoryPoint point;

    point.positions.push_back(hand.joints[0]);
    point.positions.push_back(hand.joints[1]);
    point.positions.push_back(hand.joints[2]);
    point.positions.push_back(hand.joints[3]);
    point.positions.push_back(hand.joints[4]);
    point.positions.push_back(hand.joints[5]);
    point.positions.push_back(hand.joints[6]);

    point.time_from_start = ros::Duration(2);

    t.points.push_back(point);

}
/**
 * Compute gripper translation from given start and goal poses.
 *
 * @param start The start position of the gripper
 * @param goal The goal position of the gripper
 * @param translation The GripperTranslation message to fill
 */
void PickupPlanner::computeTranslation(const geometry_msgs::Pose &start,
                                        const geometry_msgs::Pose &goal,
                                        moveit_msgs::GripperTranslation &translation) {

    Eigen::Vector3d start_vec(start.position.x, start.position.y, start.position.z);
    Eigen::Vector3d goal_vec(goal.position.x, goal.position.y, goal.position.z);

    Eigen::Vector3d t_vec = goal_vec - start_vec;
    double distance = t_vec.norm();
    t_vec.normalize();

    translation.direction.vector.x = t_vec.x();
    translation.direction.vector.y = t_vec.y();
    translation.direction.vector.z = t_vec.z();

    translation.desired_distance = distance;
    // just to give a little bit of flexibility here...
    translation.min_distance = distance * 0.7;
    translation.direction.header.frame_id = "world_link";
    translation.direction.header.stamp = ros::Time::now();

}
/**
 * Convert given definitions::Grasp message into corresponding moveit_msgs::Grasp message with
 * given grasp_id
 *
 * @param grasp_id The name of the resulting grasp
 * @param d_grasp The definintions::Grasp message where the data comes from
 * @param m_grasp The moveit_msgs::Grasp message to fill
 */
void PickupPlanner::DefGraspToMoveitGrasp(const string &grasp_id, const definitions::Grasp &d_grasp, moveit_msgs::Grasp &m_grasp) {

    trajectory_msgs::JointTrajectory pre_grasp_posture;
    fillTrajectory(pre_grasp_posture, d_grasp.grasp_trajectory[0]);

    trajectory_msgs::JointTrajectory grasp_posture;
    fillTrajectory(grasp_posture, d_grasp.grasp_trajectory[2]);

    geometry_msgs::Pose grasp_pose = d_grasp.grasp_trajectory[2].wrist_pose.pose;
    geometry_msgs::Pose pre_grasp_pose = d_grasp.grasp_trajectory[0].wrist_pose.pose;

    moveit_msgs::GripperTranslation pre_grasp_approach;
    computeTranslation(pre_grasp_pose, grasp_pose, pre_grasp_approach);

    moveit_msgs::GripperTranslation post_grasp_retreat;
    // the retreat translation is the opposite of the approach translation.
    computeTranslation(grasp_pose, pre_grasp_pose, post_grasp_retreat);

    m_grasp.id = grasp_id;
    m_grasp.grasp_quality = 1;
    m_grasp.max_contact_force = 0;
    m_grasp.grasp_pose.header.frame_id = "world_link";
    m_grasp.grasp_pose.header.stamp = ros::Time::now();
    m_grasp.grasp_pose.pose = grasp_pose;
    m_grasp.pre_grasp_posture = pre_grasp_posture;
    m_grasp.grasp_posture = grasp_posture;
    m_grasp.pre_grasp_approach = pre_grasp_approach;
    m_grasp.post_grasp_retreat = post_grasp_retreat;
}

void PickupPlanner::constructPickupGoal(const string &arm,
                                        const string &object_id,
                                        vector<moveit_msgs::Grasp> &grasps,
                                        moveit_msgs::PickupGoal &goal)
{
    // determine the correct end effector name
    string eef_name = "";
    if(arm == "right_arm") {
        eef_name = "right_ee";
    } else if(arm == "left_arm") {
        eef_name = "left_ee";
    } else {
        ROS_ERROR("Unknown arm '%s'! Unable to determine end effector name.", arm.c_str());
    }

    goal.target_name = object_id;
    goal.group_name = arm;
    goal.end_effector = eef_name;
    goal.allowed_planning_time = planning_time_;
    goal.support_surface_name = support_surface_;
    goal.planner_id = planner_id_;

    if (!support_surface_.empty()) {
        goal.allow_gripper_support_collision = true;
    }

    goal.possible_grasps = grasps;
    goal.planning_options.look_around = CAN_LOOK;
    goal.planning_options.replan = ALLOW_REPLAN;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.plan_only = true;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    ROS_INFO("Pickup goal constructed for group '%s' and end effector '%s'", goal.group_name.c_str(), goal.end_effector.c_str());

}

void PickupPlanner::PickupResultToTrajectory(moveit_msgs::PickupResultConstPtr &result, vector<Trajectory> &trajectories)
{
    for(size_t i = 0; i < result->trajectory_stages.size(); ++i) {

        definitions::Trajectory trajectory;

        const vector<trajectory_msgs::JointTrajectoryPoint> &points = result->trajectory_stages[i].joint_trajectory.points;
        // pick each point from the trajectory and create a UIBKRobot object
        for (size_t i = 0; i < points.size(); ++i) {
            definitions::UIBKRobot robot_point;
            robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
            trajectory.robot_path.push_back(robot_point);
        }

        trajectories.push_back(trajectory);
    }
}

} // namespace trajectory_planner_moveit
