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
    goal_state.armRight.joints.resize(7);
    goal_state.handRight.joints.resize(7);
    goal_state.armLeft.joints.resize(7);
    goal_state.handLeft.joints.resize(7);
    ros::Duration five_seconds(5.0);

    // FIRST plan for the right
    definitions::TrajectoryPlanning trajectory_planning_srv;
    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_STATE_GOAL;
    trajectory_planning_srv.request.arm = "right";

    // first, one simple motion: try to move to the safe position first without checking
    // first, one simple motion: try to move to the safe position first without checking
    // goal_state.armRight.joints[0] = 0.90358;
    // goal_state.armRight.joints[1] = 1.07305;
    // goal_state.armRight.joints[2] = 1.16986;
    // goal_state.armRight.joints[3] = 0.89418;
    // goal_state.armRight.joints[4] = 0.74461;
    // goal_state.armRight.joints[5] = 0.01476;
    // goal_state.armRight.joints[6] = -0.48848;
    // goal_state.handRight.joints[0] = 0.0;
    // goal_state.handRight.joints[1] = -0.8;
    // goal_state.handRight.joints[2] = 0.8;
    // goal_state.handRight.joints[3] = -0.8;
    // goal_state.handRight.joints[4] = 0.8;
    // goal_state.handRight.joints[5] = -0.8;
    // goal_state.handRight.joints[6] = 0.8;

    goal_state.armRight.joints[0] = 0.0;
    goal_state.armRight.joints[1] = 0.0;
    goal_state.armRight.joints[2] = 0.0;
    goal_state.armRight.joints[3] = 0.0;
    goal_state.armRight.joints[4] = 0.0;
    goal_state.armRight.joints[5] = 0.0;
    goal_state.armRight.joints[6] = 0.0;
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

    ros::Duration(5).sleep();

    // NOW plan for the left arm
    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_STATE_GOAL;
    trajectory_planning_srv.request.arm = "left";

    // first, one simple motion: try to move to the safe position first without checking     goal_state.handAright.joints[0] = 0.90358;
    goal_state.armLeft.joints[0] = 0.98819;
    goal_state.armLeft.joints[1] = -1.01639;
    goal_state.armLeft.joints[2] = 2.00266;
    goal_state.armLeft.joints[3] = 0.98314;
    goal_state.armLeft.joints[4] = 0.0;
    goal_state.armLeft.joints[5] = 0.0;
    goal_state.armLeft.joints[6] = 1.25715;
    goal_state.handLeft.joints[0] = 0.0;
    goal_state.handLeft.joints[1] = -0.8;
    goal_state.handLeft.joints[2] = 0.8;
    goal_state.handLeft.joints[3] = -0.8;
    goal_state.handLeft.joints[4] = 0.8;
    goal_state.handLeft.joints[5] = -0.8;
    goal_state.handLeft.joints[6] = 0.8;

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

    return 0;
}