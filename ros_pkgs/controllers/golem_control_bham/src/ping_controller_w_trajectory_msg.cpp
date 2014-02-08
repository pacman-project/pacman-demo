#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>

//for the messages used in the services
#include "definitions/TrajectoryExecution.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ping_golem_controller_w_trajector_msg");
    ros::NodeHandle nh;

    // execution service
    std::string execution_service_name("/trajectory_execution_srv");
    if ( !ros::service::waitForService(execution_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  execution_service_name.c_str());
      return (-1);     
    }

    // create a test trajectory
    definitions::Trajectory trajectory;
    ros::Duration five_seconds(5.0);

    // resize with the number of waypoints you want to test
    trajectory.robot_path.resize(3);
    trajectory.time_from_previous.resize(3);

    // start at home
    for (int j = 0; j < 7; j++)
    {
      trajectory.robot_path[0].arm.joints.push_back(0);
      trajectory.robot_path[0].hand.joints.push_back(0);
    }
    trajectory.time_from_previous[0] = ros::Duration().fromSec(0);

    // second way point
    trajectory.robot_path[1] = trajectory.robot_path[0];
    trajectory.robot_path[1].arm.joints[1] = trajectory.robot_path[0].arm.joints[0] + 0.5;
    trajectory.time_from_previous[1] = five_seconds;

    // third way point
    trajectory.robot_path[2] = trajectory.robot_path[0];
    trajectory.time_from_previous[2] = five_seconds;

    // create the service instance
    definitions::TrajectoryExecution trajectory_execution_srv;
    trajectory_execution_srv.request.trajectory.push_back(trajectory);

    // call the service with the instance
    ROS_INFO("Calling the service");
    if ( !ros::service::call( execution_service_name, trajectory_execution_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", execution_service_name.c_str());  
        return (-1);
    }   

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.OTHER_ERROR)
    {   
        ROS_ERROR("Unable to execute the trajectory");
        return (-1);
    }

    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.SUCCESS)
    { 
        ROS_INFO("Trajectory Execution OK...\n");
        return 0;
    }

    return 0;
}