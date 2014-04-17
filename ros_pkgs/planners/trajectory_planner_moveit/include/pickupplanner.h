#ifndef PICKUPPLANNER_H
#define PICKUPPLANNER_H

//// system headers
#include <boost/shared_ptr.hpp>

//// ros headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/Grasp.h>

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers


#define CAN_LOOK false
#define ALLOW_REPLAN false
#define SUPPORT_SURFACE "table_surface_link"
#define FRAME_ID "world_link"


using namespace std;
using namespace definitions;


namespace trajectory_planner_moveit {


class PickupPlanner
{

private:

    boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;

    double planning_time_;
    int planning_attempts_;
    double goal_joint_tolerance_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;
    string planner_id_;
    string support_surface_;

    string joint_states_topic_;

    /**
     * Compute gripper translation from given start and goal poses.
     *
     * @param start The start position of the gripper
     * @param goal The goal position of the gripper
     * @param translation The GripperTranslation message to fill
     */
    void computeTranslation(const geometry_msgs::Pose &start,
                            const geometry_msgs::Pose &goal,
                            moveit_msgs::GripperTranslation &translation);
    /**
     * Convert given definitions::Grasp message into corresponding moveit_msgs::Grasp message with
     * given grasp_id
     *
     * @param arm Which arm to use
     * @param grasp_id The name of the resulting grasp
     * @param d_grasp The definintions::Grasp message where the data comes from
     * @param m_grasp The moveit_msgs::Grasp message to fill
     */
    void DefGraspToMoveitGrasp(const string &arm, const string &grasp_id, const definitions::Grasp &d_grasp, moveit_msgs::Grasp &m_grasp);
    /**
     * @brief populatePickupMessage
     * @param arm
     * @param object_id
     * @param grasps
     * @param goal_msg the message to fill
     */
    void constructPickupGoal(const string &arm,
                                const string &object_id,
                                vector<moveit_msgs::Grasp> &grasps,
                                moveit_msgs::PickupGoal &goal);
    /**
     * @brief PickupResultToTrajectory
     * @param arm
     * @param result
     * @param trajectory
     * @return
     */
    bool PickupResultToTrajectory(const string &arm,
                                  moveit_msgs::PickupResultConstPtr &result,
                                  definitions::Trajectory &trajectory);
    /**
     * Convenience method to fill in the trajectory data from the given SDHand into
     * the given Trajectory
     *
     * @param t the trajectory message to fill
     * @param hand the SDHand where the data comes from.
     */
    void fillTrajectory(const string &arm, trajectory_msgs::JointTrajectory &t, const definitions::SDHand &hand);

public:
    PickupPlanner(ros::NodeHandle &nh);
    virtual ~PickupPlanner() {}

    bool planPickup(TrajectoryPlanning::Request &request, TrajectoryPlanning::Response &response);
};


typedef boost::shared_ptr<PickupPlanner> PickupPlannerPtr;

}

#endif // PICKUPPLANNER_H
