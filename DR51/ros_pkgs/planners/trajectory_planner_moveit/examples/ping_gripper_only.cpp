#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>

//for the messages used in the services
#include "definitions/TrajectoryExecution.h"
#include "conversions.h"

using namespace std;
using namespace pacman;

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

    // wait for a joint state message to retrieve the current configuration of the robot
    sensor_msgs::JointStateConstPtr start_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/golem/joint_states", ros::Duration(3.0));
//    const sensor_msgs::JointState &start_state = *start_state_ptr;

    if (!start_state_ptr)
    {
        ROS_WARN("No real start state available, since no joint state recevied");
        ROS_WARN("Did you forget to start a controller?");
        ROS_WARN("The resulting trajectory might be invalid!");

        return -1;
    }

    // create a test trajectory
    definitions::Trajectory trajectory;

    // fill robot point with initial data
    definitions::RobotEddie robot_point;
    trajectory_planner_moveit::getEddiePointFromJointStates(*start_state_ptr, robot_point);

    trajectory.eddie_path.push_back(robot_point);
    trajectory.time_from_previous.push_back(ros::Duration(0.0));

    // pre_grasp
    robot_point.handRight.joints[0] = 0.0;
    robot_point.handRight.joints[1] = (-M_PI / 4);
    robot_point.handRight.joints[2] = (M_PI / 9);
    robot_point.handRight.joints[3] = (-M_PI / 4);
    robot_point.handRight.joints[4] = (M_PI / 9);
    robot_point.handRight.joints[5] = (-M_PI / 4);
    robot_point.handRight.joints[6] = (M_PI / 9);

    trajectory.eddie_path.push_back(robot_point);
    trajectory.time_from_previous.push_back(ros::Duration(2.0));

    // grasp
    robot_point.handRight.joints[0] = (0.0);
    robot_point.handRight.joints[1] = (-M_PI / 14);
    robot_point.handRight.joints[2] = (M_PI / 6);
    robot_point.handRight.joints[3] = (-M_PI / 14);
    robot_point.handRight.joints[4] = (M_PI / 6);
    robot_point.handRight.joints[5] = (-M_PI / 14);
    robot_point.handRight.joints[6] = (M_PI / 6);

    trajectory.eddie_path.push_back(robot_point);
    trajectory.time_from_previous.push_back(ros::Duration(2.0));

    // home
    robot_point.handRight.joints[0] = (0.0);
    robot_point.handRight.joints[1] = (0.0);
    robot_point.handRight.joints[2] = (0.0);
    robot_point.handRight.joints[3] = (0.0);
    robot_point.handRight.joints[4] = (0.0);
    robot_point.handRight.joints[5] = (0.0);
    robot_point.handRight.joints[6] = (0.0);


    trajectory.eddie_path.push_back(robot_point);
    trajectory.time_from_previous.push_back(ros::Duration(2.0));



    // create the service instance
    definitions::TrajectoryExecution trajectory_execution_srv;
    trajectory_execution_srv.request.trajectory = trajectory;

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
