#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <definitions/SDHand.h>
#include <definitions/RobotEddie.h>
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
//        hand.velocity.push_back(state.velocity[index]);
        hand.velocity.push_back(0.01);
        hand.acceleration.push_back(0.0);
    }

    return true;

}

int getJointIndex(const string &joint, const vector<string> &joint_names) {
    for(size_t i = 0; i < joint_names.size(); ++i) {
        if(joint == joint_names[i]) {
            return (int)i;
        }
    }
    return -1;
}

void getSDHandFromMoveitTrajectory(const string &arm, moveit_msgs::RobotTrajectory &m_trajectory, definitions::SDHand &d_hand)
{
    if(m_trajectory.joint_trajectory.points.size() > 0) {
        // take the last trajectory point as the target state of the SDHand
        trajectory_msgs::JointTrajectoryPoint point = m_trajectory.joint_trajectory.points.back();
        d_hand.joints.resize(7);
        d_hand.velocity.resize(7);
        d_hand.acceleration.resize(7);

        // I have to do some nasty searching for the correct indices as moveit seems to rearrange the
        // order of joint names and the corresponding values...
        string joint_name;

        joint_name = arm + "_sdh_knuckle_joint";
        int index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[0] = point.positions[index];
            d_hand.velocity[0] = point.velocities[index];
            d_hand.acceleration[0] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_finger_12_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[1] = point.positions[index];
            d_hand.velocity[1] = point.velocities[index];
            d_hand.acceleration[1] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_finger_13_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[2] = point.positions[index];
            d_hand.velocity[2] = point.velocities[index];
            d_hand.acceleration[2] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_finger_22_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[3] = point.positions[index];
            d_hand.velocity[3] = point.velocities[index];
            d_hand.acceleration[3] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_finger_23_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[4] = point.positions[index];
            d_hand.velocity[4] = point.velocities[index];
            d_hand.acceleration[4] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_thumb_2_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[5] = point.positions[index];
            d_hand.velocity[5] = point.velocities[index];
            d_hand.acceleration[5] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

        joint_name = arm + "_sdh_thumb_3_joint";
        index = getJointIndex(joint_name, m_trajectory.joint_trajectory.joint_names);
        if(index >= 0) {
            d_hand.joints[6] = point.positions[index];
            d_hand.velocity[6] = point.velocities[index];
            d_hand.acceleration[6] = point.accelerations[index];
        } else {
            ROS_ERROR("Trajectory contains no joint with name '%s'", joint_name.c_str());
        }

    } else {
        ROS_ERROR("Unable to extract SDHand state from trajectory - trajectory points are empty!");
    }
}

void getEddiePointFromJointStates(const sensor_msgs::JointState &state, definitions::RobotEddie &robot_point) {

    for(int j = 0; j < pacman::KukaLWR::Config::JOINTS; j++)
    {
        robot_point.armRight.joints.push_back(state.position[j+7]);
        robot_point.armRight.velocity.push_back(state.velocity[j+7]);
        robot_point.armRight.acceleration.push_back(0.0);

        robot_point.armLeft.joints.push_back(state.position[j]);
        robot_point.armLeft.velocity.push_back(state.velocity[j]);
        robot_point.armLeft.acceleration.push_back(0.0);
    }

    for(int h = 0; h < pacman::SchunkDexHand::Config::JOINTS; h++)
    {
        robot_point.handRight.joints.push_back(state.position[h+21]);
        robot_point.handRight.velocity.push_back(state.velocity[h+21]);
        robot_point.handRight.acceleration.push_back(0.0);

        robot_point.handLeft.joints.push_back(state.position[h+14]);
        robot_point.handLeft.velocity.push_back(state.velocity[h+14]);
        robot_point.handLeft.acceleration.push_back(0.0);
    }

    for(int k = 0; k < pacman::KITHead::Config::JOINTS_NECK; k++)
    {
        robot_point.head.joints.push_back(state.position[k+28]);
        robot_point.head.velocity.push_back(state.velocity[k+28]);
        robot_point.head.acceleration.push_back(0.0);
    }

    robot_point.head.jointsLEye = state.position[33];
    robot_point.head.velocityLEye = state.velocity[33];
    robot_point.head.accelerationLEye = 0.0;

    robot_point.head.jointsREye = state.position[34];
    robot_point.head.velocityREye = state.velocity[34];
    robot_point.head.accelerationREye = 0.0;
}

void trajectoryPointToKukaLWR(const trajectory_msgs::JointTrajectoryPoint &point, definitions::KukaLWR &arm) {
    arm.joints.assign(point.positions.begin(), point.positions.end());
    arm.velocity.assign(point.velocities.begin(), point.velocities.end());
    arm.acceleration.assign(point.accelerations.begin(), point.accelerations.end());
}




} // namespace trajectory_planner_moveit


#endif // CONVERSIONS_H
