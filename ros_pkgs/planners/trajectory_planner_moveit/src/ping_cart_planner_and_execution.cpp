#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

//for the messages used in the services
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"

// the safe state in cartesian
definitions::UIBKRobot safe_state;
ros::Duration five_seconds(5.0);

// create the planning and execution service instance
definitions::TrajectoryPlanning trajectory_planning_srv;
definitions::TrajectoryExecution trajectory_execution_srv;

// planning and execution service names
std::string planning_service_name("/trajectory_planning_srv");
std::string execution_service_name("/trajectory_execution_srv");

void tryGoal(int id, definitions::UIBKRobot &goal_state)
{
    ROS_INFO("Trying Goal %d...", id);
    trajectory_planning_srv.request.goal_state = goal_state;
    ros::service::call( planning_service_name, trajectory_planning_srv);

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
    { 
        ROS_INFO("Found a plan to Goal %d...", id);
        trajectory_execution_srv.request.trajectory = trajectory_planning_srv.response.trajectory[0];
        ros::Duration(5).sleep(); 
        ros::service::call( execution_service_name, trajectory_execution_srv);
        
        ros::Duration(5).sleep(); 
        
        ROS_INFO("Plan back to safe position...");
        trajectory_planning_srv.request.goal_state = safe_state;;
        ros::service::call( planning_service_name, trajectory_planning_srv);

        if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
        {
            ROS_INFO("Found a plan to safe position...");
            trajectory_execution_srv.request.trajectory = trajectory_planning_srv.response.trajectory[0];
            ros::Duration(5).sleep(); 
            ros::service::call( execution_service_name, trajectory_execution_srv);
        }
        else
        {
            ROS_INFO("Could not plan to safe position...\n");
        }
    }
    else
    {
        ROS_INFO("Could not plan for Goal %d...\n", id);
    }

    ros::Duration(5).sleep(); 
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ping_path_planner_and_execution");
    ros::NodeHandle nh;

    safe_state.hand.wrist_pose.pose.position.x = 0.1624;
    safe_state.hand.wrist_pose.pose.position.y = -0.2599;
    safe_state.hand.wrist_pose.pose.position.z = 0.6642;
    safe_state.hand.wrist_pose.pose.orientation.x = 0.404885;
    safe_state.hand.wrist_pose.pose.orientation.y = 0.86333;
    safe_state.hand.wrist_pose.pose.orientation.z = -0.139283;
    safe_state.hand.wrist_pose.pose.orientation.w = 0.267076;
    safe_state.hand.wrist_pose.header.frame_id = "world_link";

    if ( !ros::service::waitForService(planning_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  planning_service_name.c_str());
      return (-1);     
    }

    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL;
    trajectory_planning_srv.request.arm = "right";

    trajectory_planning_srv.request.goal_state = safe_state;

    // call the planning service with the instance
    ROS_INFO("Calling the planning service for the right arm");
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
        ROS_INFO("Trajectory Planning OK, now execute the trajectory...\n");
    }
    ros::Duration(5).sleep();
    if ( !ros::service::waitForService(execution_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  execution_service_name.c_str());
      return (-1);     
    }

    // create the execution service instance
    trajectory_execution_srv.request.trajectory = trajectory_planning_srv.response.trajectory[0];

    // call the execution service with the instance
    ROS_INFO("Calling the execution service");
    if ( !ros::service::call( execution_service_name, trajectory_execution_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", execution_service_name.c_str());  
        return (-1);
    }   

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.OTHER_ERROR)
    {   
        ROS_ERROR("Unable to execute the trajectory: OTHER_ERROR");
        return (-1);
    }

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.SUCCESS)
    { 
        ROS_INFO("Trajectory execution OK... moved to safe position succesfully!\n");
        ROS_INFO("Now, we will try several goal positions for planning...\n");
    }

    definitions::UIBKRobot goal_state;

    // GOAL 1
    goal_state.hand.wrist_pose.pose.position.x = 0.207500562072;
    goal_state.hand.wrist_pose.pose.position.y = 0.558501422405;
    goal_state.hand.wrist_pose.pose.position.z = 0.469974249601;
    goal_state.hand.wrist_pose.pose.orientation.x = 0.630408406258;
    goal_state.hand.wrist_pose.pose.orientation.y = -0.77248442173;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.0736171901226;
    goal_state.hand.wrist_pose.pose.orientation.w = 0.0208686068654;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";
    tryGoal(1, goal_state);
    
    // GOAL 2
    goal_state.hand.wrist_pose.pose.position.x = 0.285089105368;
    goal_state.hand.wrist_pose.pose.position.y = 0.627856254578;
    goal_state.hand.wrist_pose.pose.position.z = 0.46203365922;
    goal_state.hand.wrist_pose.pose.orientation.x = -0.0602076053619;
    goal_state.hand.wrist_pose.pose.orientation.y = -0.997133851051;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.032443780452;
    goal_state.hand.wrist_pose.pose.orientation.w = 0.0323820859194;
    tryGoal(2, goal_state);

    // GOAL 3
    goal_state.hand.wrist_pose.pose.position.x = 0.257347553968;
    goal_state.hand.wrist_pose.pose.position.y = 0.483040869236;
    goal_state.hand.wrist_pose.pose.position.z = 0.460594981909;
    goal_state.hand.wrist_pose.pose.orientation.x = 0.995404422283;
    goal_state.hand.wrist_pose.pose.orientation.y = -0.0931990146637;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.0217804089189;
    goal_state.hand.wrist_pose.pose.orientation.w = -0.00340245850384;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";
    tryGoal(3, goal_state);

    // GOAL 4
    goal_state.hand.wrist_pose.pose.position.x = 0.342958420515;
    goal_state.hand.wrist_pose.pose.position.y = 0.542339384556;
    goal_state.hand.wrist_pose.pose.position.z = 0.466535836458;
    goal_state.hand.wrist_pose.pose.orientation.x = 0.772745728493;
    goal_state.hand.wrist_pose.pose.orientation.y = 0.634146571159;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.00403589010239;
    goal_state.hand.wrist_pose.pose.orientation.w = -0.0266053937376;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";
    tryGoal(4, goal_state);

    // GOAL 5
    goal_state.hand.wrist_pose.pose.position.x = 0.0494939684868;
    goal_state.hand.wrist_pose.pose.position.y = 0.752711713314;
    goal_state.hand.wrist_pose.pose.position.z = 0.465163171291;
    goal_state.hand.wrist_pose.pose.orientation.x = -0.251597672701;
    goal_state.hand.wrist_pose.pose.orientation.y = 0.964012026787;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.0859108045697;
    goal_state.hand.wrist_pose.pose.orientation.w = 0.00193554908037;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";
    tryGoal(5, goal_state);

    // GOAL 6
    goal_state.hand.wrist_pose.pose.position.x = 0.0494939684868;
    goal_state.hand.wrist_pose.pose.position.y = 0.752711713314;
    goal_state.hand.wrist_pose.pose.position.z = 0.465163171291;
    goal_state.hand.wrist_pose.pose.orientation.x = -0.251597672701;
    goal_state.hand.wrist_pose.pose.orientation.y = 0.964012026787;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.0859108045697;
    goal_state.hand.wrist_pose.pose.orientation.w = 0.00193554908037;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";
    tryGoal(6, goal_state);

    return 0;
}