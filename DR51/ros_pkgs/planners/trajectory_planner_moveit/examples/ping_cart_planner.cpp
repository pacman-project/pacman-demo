#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

//for the messages used in the services
#include "definitions/TrajectoryPlanning.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ping_path_planner");
    ros::NodeHandle nh;

    // planning service
    std::string planning_service_name("/trajectory_planning_srv");
    if ( !ros::service::waitForService(planning_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  planning_service_name.c_str());
      return (-1);     
    }

    // create a test goal state
    definitions::RobotEddie goal_state;
    goal_state.handRight.joints.resize(7);
    goal_state.handLeft.joints.resize(7);
    ros::Duration five_seconds(5.0);

    // FIRST plan for the right
    definitions::TrajectoryPlanning trajectory_planning_srv;
    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL;
    trajectory_planning_srv.request.arm = "right";

    // first, one simple motion: try to move to the safe position first without checking
    goal_state.handRight.wrist_pose.pose.position.x = 0.1624;
    goal_state.handRight.wrist_pose.pose.position.y = -0.2599;
    goal_state.handRight.wrist_pose.pose.position.z = 0.6642;
    goal_state.handRight.wrist_pose.pose.orientation.x = 0.404885;
    goal_state.handRight.wrist_pose.pose.orientation.y = 0.86333;
    goal_state.handRight.wrist_pose.pose.orientation.z = -0.139283;
    goal_state.handRight.wrist_pose.pose.orientation.w = 0.267076;
    goal_state.handRight.wrist_pose.header.frame_id = "world_link";
    goal_state.handRight.joints[0] = 0.0;
    goal_state.handRight.joints[1] = -0.8;
    goal_state.handRight.joints[2] = 0.8;
    goal_state.handRight.joints[3] = -0.8;
    goal_state.handRight.joints[4] = 0.8;
    goal_state.handRight.joints[5] = -0.8;
    goal_state.handRight.joints[6] = 0.8;

    trajectory_planning_srv.request.eddie_goal_state = goal_state;

    // call the planning service with the instance
    ROS_INFO("Calling the planning service for the %s arm", trajectory_planning_srv.request.arm.c_str());
    if ( !ros::service::call( planning_service_name, trajectory_planning_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", planning_service_name.c_str());  
        return (-1);
    }   

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.OTHER_ERROR)
    {   
        ROS_ERROR("Unable to plan a trajectory: OTHER_ERROR");
        return (-1);
    }

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.NO_FEASIBLE_TRAJECTORY_FOUND)
    {   
        ROS_ERROR("Unable to plan a trajectory:: NO_FEASIBLE_TRAJECTORY_FOUND");
        return (-1);
    }

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
    { 
        ROS_INFO("Trajectory Planning OK...\n");
    }

//     ros::Duration(5).sleep();
// 
//     // NOW plan for the left arm
//     trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL;
//     trajectory_planning_srv.request.arm = "left";
// 
//     // first, one simple motion: try to move to the safe position first without checking
//     goal_state.handLeft.wrist_pose.pose.position.x = 0.201;
//     goal_state.handLeft.wrist_pose.pose.position.y = 1.451;
//     goal_state.handLeft.wrist_pose.pose.position.z = 0.750;
//     goal_state.handLeft.wrist_pose.pose.orientation.x = 0.270;
//     goal_state.handLeft.wrist_pose.pose.orientation.y = 0.051;
//     goal_state.handLeft.wrist_pose.pose.orientation.z = 0.363;
//     goal_state.handLeft.wrist_pose.pose.orientation.w = 0.891;
//     goal_state.handLeft.wrist_pose.header.frame_id = "world_link";
//     goal_state.handLeft.joints[0] = 0.0;
//     goal_state.handLeft.joints[1] = -0.8;
//     goal_state.handLeft.joints[2] = 0.8;
//     goal_state.handLeft.joints[3] = -0.8;
//     goal_state.handLeft.joints[4] = 0.8;
//     goal_state.handLeft.joints[5] = -0.8;
//     goal_state.handLeft.joints[6] = 0.8;
// 
//     trajectory_planning_srv.request.eddie_goal_state = goal_state;
// 
//     // call the planning service with the instance
//     ROS_INFO("Calling the planning service for the %s arm", trajectory_planning_srv.request.arm.c_str());
//     if ( !ros::service::call( planning_service_name, trajectory_planning_srv) )
//     { 
//         ROS_ERROR("Call to the service %s failed.", planning_service_name.c_str());  
//         return (-1);
//     }   
// 
//     if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.OTHER_ERROR)
//     {   
//         ROS_ERROR("Unable to plan a trajectory: OTHER_ERROR");
//         return (-1);
//     }
// 
//     if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.NO_FEASIBLE_TRAJECTORY_FOUND)
//     {   
//         ROS_ERROR("Unable to plan a trajectory:: NO_FEASIBLE_TRAJECTORY_FOUND");
//         return (-1);
//     }
// 
//     if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
//     { 
//         ROS_INFO("Trajectory Planning OK...\n");
//     }

    return 0;
}