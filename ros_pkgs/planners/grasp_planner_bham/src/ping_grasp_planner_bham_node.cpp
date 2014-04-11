#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <eigen_conversions/eigen_msg.h>

//for the messages used in the services
#include "definitions/GraspPlanning.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ping_grasp_planner_bham_node");
  ros::NodeHandle nh;

  std::vector<definitions::Object> my_detected_objects;
  my_detected_objects.resize(1);
  std::vector<definitions::Grasp> my_calculated_grasp;
  definitions::GraspPlanning grasp_planning_srv;

  // GRASP PLANNING
  std::string grasp_service_name("/grasp_planner_srv");
  if ( !ros::service::waitForService(grasp_service_name, ros::Duration().fromSec(1.0)) )
  { 
  ROS_ERROR("After one second, the service %s hasn't shown up, so continue to capture the scene...",  grasp_service_name.c_str());
  return (-1);     
  }

  // build a test pose
  geometry_msgs::Pose pose;

  // container_2 test 
  pose.position.x = 0.22;
  pose.position.y = 0.55;
  pose.position.z = 0.22;

  pose.orientation.x = -0.345244538583;
  pose.orientation.y = 0.629125777427;
  pose.orientation.z = -0.592736526524;
  pose.orientation.w = 0.365610572594;

  std_msgs::String name;
  name.data = "container_2";

  // fill the test detected object
  my_detected_objects[0].pose = pose;
  my_detected_objects[0].name.data = name.data;

  // fill the request for the grasp planner service
  grasp_planning_srv.request.ordered_objects = my_detected_objects;
  grasp_planning_srv.request.object_id = 0;

  // call the service 
  ROS_INFO("Calling the service");
  if (!ros::service::call(grasp_service_name, grasp_planning_srv) )
  { 
    ROS_ERROR("Call to grasp planning service failed.");  
    return (-1);
  } 

  if (grasp_planning_srv.response.result == grasp_planning_srv.response.NO_FEASIBLE_GRASP_FOUND)
  { 
    ROS_ERROR("No feasible grasp found.");
    return (-1);
  }    

  if (grasp_planning_srv.response.result == grasp_planning_srv.response.OTHER_ERROR)
  {   
    ROS_ERROR("Other error ocurred during the service.");
    return (-1);
  }

  if (grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS)
  { 
    ROS_INFO("Grasp Planning OK...\n");
    for (int i =0; i < grasp_planning_srv.response.grasp_list.size(); i++) 
    {
      //Filtering heuristic: z value has to be bigger than a certain amount and rotation has to look along a certain direction (top grasp), define maximum angle on quaternion
      std::cout << "Grasp Nr." << i <<  std::endl;
      std::cout << grasp_planning_srv.response.grasp_list[i].grasp_trajectory[2].wrist_pose <<  std::endl;   
    }

    return 0;
  }
}