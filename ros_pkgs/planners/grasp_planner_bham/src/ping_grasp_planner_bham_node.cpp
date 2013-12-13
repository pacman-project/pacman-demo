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

//Rotation angle between 2 Frames (along z-axis):

double rotationAngleZ(const Eigen::Matrix4f &t1, const Eigen::Matrix4f &t2)
{

double arc = t1.col(2).head(3).dot(t2.col(2).head(3));
 
return fabs(acos(arc));
}

void poseToMatrix4f(geometry_msgs::Pose &pose, Eigen::Matrix4f &mat)
{
    mat = Eigen::Matrix4f::Identity(4,4);
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);      
    Eigen::Matrix3f rotation ( q.toRotationMatrix() );
    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            mat(i,j) = rotation(i,j);
        }
    }
    mat(0,3) = pose.position.x;
    mat(1,3) = pose.position.y;
    mat(2,3) = pose.position.z;   

}

bool checkWorkspaceHeuristic(geometry_msgs::Pose &pose, double maxAngle, double minZdist) 
{
  //Directional approach vector(s), here along z-axis ~ Identity
  Eigen::Matrix4f approachDirection;
  
  approachDirection << 1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1;
    
  //convert geometry msgs:
  Eigen::Matrix4f actualPose;
  poseToMatrix4f(pose, actualPose);
  
  //Calculate rotation angle difference
  
  double diffAngle = fabs(rotationAngleZ(approachDirection, actualPose)-M_PI)*180/M_PI;
  std::cout << "[Debug] Current rotation Angle with respect to Z-axis:" << diffAngle << std::endl;
  
  double zDist = actualPose(2,3);
  std::cout << "[Debug] Current distance to Z-axis:" << zDist << std::endl;
  
  if (diffAngle < maxAngle && zDist > minZdist)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}


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
    
     pose.position.x = 0.0762727856636;
     pose.position.y = 0.498210191727;
     pose.position.z = 0.248801708221;
 
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
	for (int i =0; i < grasp_planning_srv.response.grasp_list.size(); i++) {

	      //Filtering heuristic: z value has to be bigger than a certain amount and rotation has to look along a certain direction (top grasp), define maximum angle on quaternion
	        std::cout << "Grasp Nr." << i <<  std::endl;
		std::cout << grasp_planning_srv.response.grasp_list[i].grasp_trajectory[2].wrist_pose <<  std::endl;   
	  }

        return 0;
    }
}