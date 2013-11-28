#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// for the messages used in the services
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
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    // name the test object
    std_msgs::String name;
    name.data = "pacman_mug1";

    // // load a test pcd
    // sensor_msgs::PointCloud2 cloud_from_mesh;
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("pacman_mug1.pcd", *cloud_ptr) == -1) //* load the file
    // {
    //     ROS_ERROR ("Couldn't read file pcd file for pinging \n");
    //     return (-1);
    // }

    // ROS_INFO("Point cloud read of size: %d", cloud_ptr->points.size ());

    // pcl::toROSMsg(*cloud_ptr, cloud_from_mesh);

    // ROS_INFO("Point cloud sent with width: %d, height: %d", cloud_from_mesh.width, cloud_from_mesh.height);

    // fill the test detected object
    my_detected_objects[0].pose = pose;
    my_detected_objects[0].name.data = name.data;
    //my_detected_objects[0].cloud_from_mesh = cloud_from_mesh;
    
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
        ROS_ERROR("Other error ocurred during the serivice.");
        return (-1);
    }

    if (grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS)
    { 
        ROS_INFO("Grasp Planning OK...\n");
        ROS_INFO("grasp_planning_srv.response.grasp_list[0].grasp_trajectory[2].wrist_pose.pose.position.x =  %f", 
            grasp_planning_srv.response.grasp_list[0].grasp_trajectory[0].wrist_pose.pose.position.x);

        return 0;
    }
}