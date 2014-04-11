#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

//for the messages used in the services
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"

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
    definitions::UIBKRobot goal_state;
    ros::Duration five_seconds(5.0);

    // create the service instance
    definitions::TrajectoryPlanning trajectory_planning_srv;
    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL;
    trajectory_planning_srv.request.arm = "right";

    // first, one simple motion: try to move to the safe position first without checking
    goal_state.hand.wrist_pose.pose.position.x = 0.1624;
    goal_state.hand.wrist_pose.pose.position.y = -0.2599;
    goal_state.hand.wrist_pose.pose.position.z = 0.6642;
    goal_state.hand.wrist_pose.pose.orientation.x = 0.404885;
    goal_state.hand.wrist_pose.pose.orientation.y = 0.86333;
    goal_state.hand.wrist_pose.pose.orientation.z = -0.139283;
    goal_state.hand.wrist_pose.pose.orientation.w = 0.267076;
    goal_state.hand.wrist_pose.header.frame_id = "world_link";

    trajectory_planning_srv.request.goal_state = goal_state;

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

    // execution service
    std::string execution_service_name("/trajectory_execution_srv");
    if ( !ros::service::waitForService(execution_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  execution_service_name.c_str());
      return (-1);     
    }

    // create the service instance
    definitions::TrajectoryExecution trajectory_execution_srv;
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
        ROS_INFO("Now, we will try several grasps for planning...\n");
    }

    // // now, we test with grasps obtained from real data
    // trajectory_planning_srv.request.ordered_grasp.clear();

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.207500562072;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.558501422405;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.469974249601;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = 0.630408406258;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = -0.77248442173;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.0736171901226;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = 0.0208686068654;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.285089105368;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.627856254578;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.46203365922;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = -0.0602076053619;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = -0.997133851051;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.032443780452;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = 0.0323820859194;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.257347553968;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.483040869236;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.460594981909;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = 0.995404422283;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = -0.0931990146637;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.0217804089189;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = -0.00340245850384;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.342958420515;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.542339384556;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.466535836458;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = 0.772745728493;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = 0.634146571159;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.00403589010239;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = -0.0266053937376;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.0494939684868;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.752711713314;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.465163171291;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = -0.251597672701;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = 0.964012026787;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.0859108045697;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = 0.00193554908037;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.0494939684868;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.752711713314;
    // grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.465163171291;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = -0.251597672701;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = 0.964012026787;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = -0.0859108045697;
    // grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = 0.00193554908037;
    // trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // // call the planning service
    // ROS_INFO("Calling the planning service");
    // if ( !ros::service::call( planning_service_name, trajectory_planning_srv) )
    // { 
    //     ROS_ERROR("Call to the service %s failed.", planning_service_name.c_str());  
    //     return (-1);
    // }   

    // if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.OTHER_ERROR)
    // {   
    //     ROS_ERROR("Unable to plan a trajectory: OTHER_ERROR");
    //     return (-1);
    // }

    // if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.NO_FEASIBLE_TRAJECTORY_FOUND)
    // {   
    //     ROS_ERROR("Unable to plan a trajectory:: NO_FEASIBLE_TRAJECTORY_FOUND");
    //     return (-1);
    // }

    // if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
    // { 
    //     ROS_INFO("Trajectory Planning OK, now test execution after 5 seconds...\n");
    // }

    // // wait 10 seconds before calling the execution service
    // ros::Duration(5).sleep();

    // // execution service
    // if ( !ros::service::waitForService(execution_service_name, ros::Duration().fromSec(1.0)) )
    // { 
    //   ROS_ERROR("After one second, the service %s hasn't shown up...",  execution_service_name.c_str());
    //   return (-1);     
    // }

    // // fill with new data
    // trajectory_execution_srv.request.trajectory = trajectory_planning_srv.response.trajectory[0];

    // // call the execution service
    // ROS_INFO("Calling the execution service");
    // if ( !ros::service::call( execution_service_name, trajectory_execution_srv) )
    // { 
    //     ROS_ERROR("Call to the service %s failed.", execution_service_name.c_str());  
    //     return (-1);
    // }   

    // if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.OTHER_ERROR)
    // {   
    //     ROS_ERROR("Unable to execute the trajectory: OTHER_ERROR");
    //     return (-1);
    // }

    // if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.SUCCESS)
    // { 
    //     ROS_INFO("Trajectory execution OK... pinging done succesfully!\n");
    // }

    return 0;
}