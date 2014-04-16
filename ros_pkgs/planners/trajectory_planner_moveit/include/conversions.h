#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <definitions/SDHand.h>
#include <moveit_msgs/RobotTrajectory.h>

// our pacman definitions
#include <pacman/PaCMan/Defs.h>
#include <pacman/PaCMan/ROS.h>

using namespace std;
using namespace pacman;

namespace trajectory_planner_moveit {


/**
 * Extract the SDHand configuration for given arm from given JointState message
 *
 * @param arm which arm to use
 * @param state the JointState from which the configuration is to be extracted
 * @param hand the definitions::SDHand to fill
 *
 * @return false if other value than 'left' or 'right' is provided as arm parameter
 */
bool getSDHandFromJointStatesMsg(const string &arm, const sensor_msgs::JointState &state, definitions::SDHand &hand) {

    int hand_index = 0;

    if(arm == "left") {
        hand_index = 14;
    } else if(arm == "right") {
        hand_index = 21;
    } else {
        ROS_ERROR("Unknown arm '%s'", arm.c_str());
        return false;
    }

    for(int h = 0; h < SchunkDexHand::Config::JOINTS; h++)
    {
        int index = h + hand_index;

        hand.joints.push_back(state.position[index]);
        hand.velocity.push_back(state.velocity[index]);
        hand.acceleration.push_back(0.0);
    }

    return true;

}

void getSDHandFromMoveitTrajectory(moveit_msgs::RobotTrajectory &m_trajectory, definitions::SDHand &d_hand)
{

    if(m_trajectory.joint_trajectory.points.size() > 0) {
        // take the last trajectory point as the target state of the SDHand
        trajectory_msgs::JointTrajectoryPoint point = m_trajectory.joint_trajectory.points.back();

        d_hand.joints.assign(point.positions.begin(), point.positions.end());
        d_hand.velocity.assign(point.velocities.begin(), point.velocities.end());
        d_hand.acceleration.assign(point.accelerations.begin(), point.accelerations.end());

    } else {
        ROS_ERROR("Unable to extract SDHand state from trajectory - trajectory points are empty!");
    }
}



} // namespace trajectory_planner_moveit


#endif // CONVERSIONS_H
