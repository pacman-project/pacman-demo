#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

//for the messages used in the services
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"

// utility function to simplify grasp trajectory generation without too many complications
void initializeGraspTrajectory(definitions::SDHand &traj, double posx, double posy, double posz, double qx, double qy, double qz, double qw, std::string frame_id, double j0, double j1, double j2, double j3, double j4, double j5, double j6)
{
	// wrist pose
	traj.wrist_pose.pose.position.x = posx;
	traj.wrist_pose.pose.position.y = posy;
	traj.wrist_pose.pose.position.z = posz;
	traj.wrist_pose.pose.orientation.x = qx;
	traj.wrist_pose.pose.orientation.y = qy;
	traj.wrist_pose.pose.orientation.z = qz;
	traj.wrist_pose.pose.orientation.w = qw;
	traj.wrist_pose.header.frame_id = frame_id;
	// hand joint vector
	traj.joints.resize(7);
	traj.joints[0] = j0;
	traj.joints[1] = j1;
	traj.joints[2] = j2;
	traj.joints[3] = j3;
	traj.joints[4] = j4;
	traj.joints[5] = j5;
	traj.joints[6] = j6;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ping_pick_planner");
    ros::NodeHandle nh;

    // planning and execution service
    std::string planning_service_name("/trajectory_planning_srv");
	std::string execution_service_name("/trajectory_execution_srv");

    if ( !ros::service::waitForService(planning_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  planning_service_name.c_str());
      return (-1);     
    }

    // create a test goal state
    std::vector<definitions::Grasp> test_grasps;
    definitions::Grasp test_grasp;
    definitions::SDHand single_grasp;

    initializeGraspTrajectory(single_grasp,0.1624,-0.2599,0.6642,0.404885,0.86333,-0.139283,0.267076,"world_link",0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    test_grasp.grasp_trajectory.push_back(single_grasp);
    initializeGraspTrajectory(single_grasp,0.1624,-0.2599,0.5642,0.404885,0.86333,-0.139283,0.267076,"world_link",0.0,-0.8,0.8,-0.8,0.8,-0.8,0.8);
    test_grasp.grasp_trajectory.push_back(single_grasp);
    initializeGraspTrajectory(single_grasp,0.1624,-0.2599,0.4642,0.404885,0.86333,-0.139283,0.267076,"world_link",0.0,-0.8,0.8,-0.8,0.8,-0.8,0.8);
    test_grasp.grasp_trajectory.push_back(single_grasp);
    initializeGraspTrajectory(single_grasp,0.1624,-0.2599,0.3642,0.404885,0.86333,-0.139283,0.267076,"world_link",0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    test_grasp.grasp_trajectory.push_back(single_grasp);
//     initializeGraspTrajectory(single_grasp,0.1624,-0.2599,0.3042,0.404885,0.86333,-0.139283,0.267076,"world_link",0.0,0.0,0.0,0.0,0.0,0.0,0.0);
//     test_grasp.grasp_trajectory.push_back(single_grasp);

    test_grasps.push_back(test_grasp);

    // FIRST plan for the right
    definitions::TrajectoryPlanning trajectory_planning_srv;
    trajectory_planning_srv.request.type = definitions::TrajectoryPlanning::Request::PICK;
    trajectory_planning_srv.request.arm = "right";

    trajectory_planning_srv.request.ordered_grasp = test_grasps;

    ros::Time now = ros::Time::now();

    // call the picking service with the instance
    ROS_INFO("Calling the picking service for the %s hand", trajectory_planning_srv.request.arm.c_str());
    if ( !ros::service::call( planning_service_name, trajectory_planning_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", planning_service_name.c_str());  
        return (-1);
    }   

    std::cout << "Time to do the pick planning: " << ros::Time::now() - now << std::endl;

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
    
    // print the whole trajectory
    // std::cout << "trajectory[0]:" << std::endl << trajectory_planning_srv.response.trajectory[0] << std::endl;
    
    // create the execution service instance
    definitions::TrajectoryExecution trajectory_execution_srv;
	trajectory_execution_srv.request.trajectory = trajectory_planning_srv.response.trajectory[0];

    // call the execution service with the instance
    ROS_INFO("Calling the execution service");
    
    now = ros::Time::now();
    
    if ( !ros::service::call( execution_service_name, trajectory_execution_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", execution_service_name.c_str());  
        return (-1);
    }   

    std::cout << "Time to do the execution: " << ros::Time::now() - now << std::endl;

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.OTHER_ERROR)
    {   
        ROS_ERROR("Unable to execute the trajectory: OTHER_ERROR");
        return (-1);
    }

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.SUCCESS)
    { 
        ROS_INFO("Trajectory execution OK... moved to pick position succesfully!\n");
    }


    return 0;
}



// #include <moveit/move_group_interface/move_group.h>
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
//   // start a ROS spinning thread
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   // this connecs to a running instance of the move_group node
//   move_group_interface::MoveGroup group("right_arm");
//   // specify that our target will be a random one
//   group.setRandomTarget();
//   // plan the motion and then move the group to the sampled target 
//   moveit::planning_interface::MoveGroup::Plan pre_grasp_plan;
//   group.move();
//   ROS_INFO("Planned!");
//   ros::waitForShutdown();
// }