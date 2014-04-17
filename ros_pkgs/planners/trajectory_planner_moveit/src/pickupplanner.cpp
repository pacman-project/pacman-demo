//// system headers
#include <Eigen/Core>
#include <Eigen/Geometry>

//// ros headers

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "pickupplanner.h"
#include "conversions.h"

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
    // retrieve the (remapped) joint states topic name
    joint_states_topic_ = nh.resolveName("/joint_states");

}

bool PickupPlanner::planPickup(TrajectoryPlanning::Request &request, TrajectoryPlanning::Response &response)
{
    string arm = "right";

    ROS_INFO("Received trajectory planning request");
    ROS_INFO("Validating received request data...");

    if(!request.type == TrajectoryPlanningRequest::PICK) {
        ROS_ERROR("Invalid request type - pickup planner can only handle pickup requests!");
        return false;
    }

    // we have to make sure that sufficient data has been provided to be able to compute a plan...
    if(request.arm.empty()) {
        ROS_WARN("No arm name provided - assuming '%s'", arm.c_str());
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
        DefGraspToMoveitGrasp(arm, id, request.ordered_grasp[i], grasp_list[i]);
    }
    // now we have a vector of grasps and can invoke the planner...

    // TODO: provide the id of the grasped object here...
    string object_id = "";

    // construct the pickup message and send it to the action server
    moveit_msgs::PickupGoal goal_msg;
    constructPickupGoal(arm, object_id, grasp_list, goal_msg);

    if (!pick_action_client_) {
        ROS_ERROR("Pick action client not found");
        return false;
    }

    if (!pick_action_client_->isServerConnected()) {
        ROS_ERROR("Pick action server not connected");
        return false;
    }

    pick_action_client_->sendGoal(goal_msg);

    if (!pick_action_client_->waitForResult()) {
        ROS_INFO_STREAM("Pickup action returned early");
    }

    moveit_msgs::PickupResultConstPtr result = pick_action_client_->getResult();

    if (pick_action_client_->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED &&
            result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

        ROS_INFO("Call to pick action server succeeded!");

        // fill the computed motion plan into a definitions::trajectory
        definitions::Trajectory trajectory;
        PickupResultToTrajectory(arm, result, trajectory);
        ROS_INFO("Computed trajectory contains %d points", (int)trajectory.eddie_path.size());

        response.trajectory.push_back(trajectory);
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
void PickupPlanner::fillTrajectory(const string &arm, trajectory_msgs::JointTrajectory &t, const definitions::SDHand &hand) {

    t.header.frame_id = "world_link";
    t.header.stamp = ros::Time::now();
    // Name of joints:
    t.joint_names.push_back(arm + "_sdh_knuckle_joint");
    t.joint_names.push_back(arm + "_sdh_finger_12_joint");
    t.joint_names.push_back(arm + "_sdh_finger_13_joint");
    t.joint_names.push_back(arm + "_sdh_finger_22_joint");
    t.joint_names.push_back(arm + "_sdh_finger_23_joint");
    t.joint_names.push_back(arm + "_sdh_thumb_2_joint");
    t.joint_names.push_back(arm + "_sdh_thumb_3_joint");
    // Position of joints
    trajectory_msgs::JointTrajectoryPoint point;

    point.positions.assign(hand.joints.begin(), hand.joints.end());
    point.velocities.assign(hand.velocity.begin(), hand.velocity.end());
    point.accelerations.assign(hand.acceleration.begin(), hand.acceleration.end());

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
void PickupPlanner::DefGraspToMoveitGrasp(const string &arm, const string &grasp_id, const definitions::Grasp &d_grasp, moveit_msgs::Grasp &m_grasp) {

    trajectory_msgs::JointTrajectory pre_grasp_posture;
    fillTrajectory(arm, pre_grasp_posture, d_grasp.grasp_trajectory[0]);

    trajectory_msgs::JointTrajectory grasp_posture;
    fillTrajectory(arm, grasp_posture, d_grasp.grasp_trajectory[2]);

    geometry_msgs::Pose grasp_pose = d_grasp.grasp_trajectory[2].wrist_pose.pose;
    geometry_msgs::Pose pre_grasp_pose = d_grasp.grasp_trajectory[0].wrist_pose.pose;
    geometry_msgs::Pose post_grasp_pose;
    // use post grasp pose if provided, otherwise provide default solution...
    if(d_grasp.grasp_trajectory.size() > 3) {
        post_grasp_pose = d_grasp.grasp_trajectory[3].wrist_pose.pose;
    } else {
        // default solution is to move the hand up for around 20cm
        post_grasp_pose = grasp_pose;
        post_grasp_pose.position.z += 0.2;
    }

    moveit_msgs::GripperTranslation pre_grasp_approach;
    computeTranslation(pre_grasp_pose, grasp_pose, pre_grasp_approach);

    moveit_msgs::GripperTranslation post_grasp_retreat;
    // the retreat translation is the opposite of the approach translation.
    computeTranslation(grasp_pose, post_grasp_pose, post_grasp_retreat);

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
    goal.target_name = object_id;
    goal.group_name = arm + "_arm";
    goal.end_effector = arm + "_ee";
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
/**
 * Convert given pickup result into a single definitions::Trajectory
 *
 * @param result the outcome from the call to the pickup action server
 * @param the vector, holding all the trajectory points
 */
bool PickupPlanner::PickupResultToTrajectory(const string &arm,
                                             moveit_msgs::PickupResultConstPtr &result,
                                             definitions::Trajectory &trajectory)
{
    // wait for a joint state message to retrieve the current configuration of the robot
    sensor_msgs::JointStateConstPtr start_state = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_topic_, ros::Duration(3.0));

    if (!start_state)
    {
        ROS_WARN("No real start state available, since no joint state recevied in topic: %s", joint_states_topic_.c_str());
        ROS_WARN("Did you forget to start a controller?");
        ROS_WARN("The resulting trajectory might be invalid!");

        return false;
    }
    // ensure that the pickup result contains all necessary trajectory stages
    if(!result->trajectory_stages.size() >= 5) {
        ROS_ERROR("Unable to convert pickup result - not enough trajectory stages in pickup result");
        return false;
    }

    // get the initial state from our received joint state message
    definitions::RobotEddie start_config;
    getEddiePointFromJointStates(*start_state, start_config);

    // ------ MOVE TO PREGRASP POSITION STAGE ------
    ros::Duration previous_time(0.0);
    const vector<trajectory_msgs::JointTrajectoryPoint> &points = result->trajectory_stages[0].joint_trajectory.points;

    for(size_t i = 0; i < points.size(); ++i) {
        // initialize current trajectory point with the start state
        definitions::RobotEddie eddie_point = start_config;

        const trajectory_msgs::JointTrajectoryPoint point = points[i];
        // generate arm configuration from moveit trajectory point
        if(arm == "right") {
            trajectoryPointToKukaLWR(point, eddie_point.armRight);
        } else {
            trajectoryPointToKukaLWR(point, eddie_point.armLeft);
        }

        trajectory.eddie_path.push_back(eddie_point);
        trajectory.time_from_previous.push_back(point.time_from_start - previous_time);

        previous_time = point.time_from_start;
    }

    // -----OPEN GRIPPER STAGE ------
    // extract SDHand from PRE_GRASP trajectory stage
    moveit_msgs::RobotTrajectory pre_grasp_traj = result->trajectory_stages[1];
    // take the last state in the trajectory and modify just the hand position
    definitions::RobotEddie pre_grasp_state = trajectory.eddie_path.back();

    if(arm == "right") {
        getSDHandFromMoveitTrajectory(arm, pre_grasp_traj, pre_grasp_state.handRight);
    } else {
        getSDHandFromMoveitTrajectory(arm, pre_grasp_traj, pre_grasp_state.handLeft);
    }

    // add this new state to trajectory and allow around 2secs for that motion
    trajectory.eddie_path.push_back(pre_grasp_state);
    trajectory.time_from_previous.push_back(ros::Duration(2.0));

    // now set our new start_config to the last point of the last trajectory stage
    start_config = trajectory.eddie_path.back();
    previous_time = ros::Duration(-0.1);

    // ------ MOVE TO GRASP POSITION STAGE ------
    const vector<trajectory_msgs::JointTrajectoryPoint> &points2 = result->trajectory_stages[2].joint_trajectory.points;

    for(size_t i = 0; i < points2.size(); ++i) {
        // initialize current trajectory point with the start state
        definitions::RobotEddie eddie_point = start_config;

        const trajectory_msgs::JointTrajectoryPoint point = points2[i];
        // generate arm configuration from moveit trajectory point
        if(arm == "right") {
            trajectoryPointToKukaLWR(point, eddie_point.armRight);
        } else {
            trajectoryPointToKukaLWR(point, eddie_point.armLeft);
        }

        trajectory.eddie_path.push_back(eddie_point);
        trajectory.time_from_previous.push_back(point.time_from_start - previous_time);

        previous_time = point.time_from_start;
    }

    // -----CLOSE GRIPPER STAGE ------
    // extract SDHand from GRASP trajectory stage
    moveit_msgs::RobotTrajectory grasp_traj = result->trajectory_stages[3];

    // now take the last state in the trajectory and modify just the hand position
    definitions::RobotEddie grasp_state = trajectory.eddie_path.back();

    if(arm == "right") {
        getSDHandFromMoveitTrajectory(arm, grasp_traj, grasp_state.handRight);
    } else {
        getSDHandFromMoveitTrajectory(arm, grasp_traj, grasp_state.handLeft);
    }

    // add this new state to trajectory and allow around 2 secs for that motion
    trajectory.eddie_path.push_back(grasp_state);
    trajectory.time_from_previous.push_back(ros::Duration(2.0));

    // now set our new start_config to the last point of the last trajectory stage
    start_config = trajectory.eddie_path.back();
    previous_time = ros::Duration(-0.1);

    // ------ MOVE TO POSTGRASP POSITION STAGE ------
    const vector<trajectory_msgs::JointTrajectoryPoint> &points3 = result->trajectory_stages[4].joint_trajectory.points;

    for(size_t i = 0; i < points3.size(); ++i) {
        // initialize current trajectory point with the start state
        definitions::RobotEddie eddie_point = start_config;

        const trajectory_msgs::JointTrajectoryPoint point = points3[i];
        // generate arm configuration from moveit trajectory point
        if(arm == "right") {
            trajectoryPointToKukaLWR(point, eddie_point.armRight);
        } else {
            trajectoryPointToKukaLWR(point, eddie_point.armLeft);
        }

        trajectory.eddie_path.push_back(eddie_point);
        trajectory.time_from_previous.push_back(point.time_from_start - previous_time);

        previous_time = point.time_from_start;
    }

    return true;
}

} // namespace trajectory_planner_moveit
