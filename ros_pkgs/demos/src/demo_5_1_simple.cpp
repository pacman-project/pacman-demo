/*********************************************************************
*
* Copyright (c) 2009, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


// The structure is more or less the following:
//
// 1) Start the main loop
//    2) Calls the pose_estimation service
//        Input: point cloud of the scene
//        Output: ordered list of detected objects 
//    3) Calls the grasp_planning service
//        Input: ordered list of detected objects 
//        Output: list of grasps for the first object of the scene 
//    4)  Calls the trajectory_planning service
//        Input: ordered list of detected objects
//               lists of detected grasps for the first object of the scene 
//        Output: trajectory for both the arm and the hand
//    5) Calls the trajectory_execution service
//        Input: trajectory for both the arm and the hand
//        Output: seccess/failure
//    

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <vector>

// for the messages used in the services
#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"




class DemoSimple
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::ServiceClient pose_client;
    ros::ServiceClient grasp_client;
    ros::ServiceClient trajectory_planner_client;
    
    //Stuff
    std::string pose_estimation_service_name;
    std::string grasp_service_name;
    std::string trajectory_planning_service_name;
    
    //Variables messages
    bool status_robot, status_ros;
    std::vector<definitions::Object> my_detected_objects;
    std::vector<definitions::Grasp> my_calculated_grasp;
    std::vector<definitions::Trajectory> my_calculated_trajectory;
    
    //Service variables
    definitions::PoseEstimation estimation_srv;
    definitions::GraspPlanning grasp_planning_srv;
    definitions::TrajectoryPlanning trajectory_planning_srv;
    definitions::TrajectoryExecution trajectory_execution_srv;

  public:

    void initRobot(); //e.g checking joint states cmd mode and hand initialization
    
    void goToStartPos();
    
    void doPoseEstimation();
    
    void planGrasps();
    
    void userCheck();
    
    void executeMovement();
    
    void goToPlacingPos();
    
    // constructor
    DemoSimple(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        pose_estimation_service_name = "/pose_estimation_uibk/estimate_poses";
	grasp_service_name = "/grasp_planner_srv";
	trajectory_planning_service_name = "/trajectory_planner_srv",
	
	pose_client = nh_.serviceClient<definitions::PoseEstimation>(pose_estimation_service_name);
	grasp_client = nh_.serviceClient<definitions::GraspPlanning>(grasp_service_name);
	trajectory_planner_client = nh_.serviceClient<definitions::TrajectoryPlanning>(trajectory_planning_service_name);
    }

    //! Empty stub
    ~DemoSimple() {}

};


void DemoSimple::goToStartPos(){}

void DemoSimple::doPoseEstimation()
{
   if( !pose_client.call(estimation_srv))
   {   
     ROS_INFO("pose estimation service failed. wait...");
     ros::Duration(0.5).sleep();
   }   
   my_detected_objects = estimation_srv.response.detected_objects; 
}

void DemoSimple::planGrasps()
{
  grasp_planning_srv.request.ordered_objects = my_detected_objects;
  grasp_planning_srv.request.object_id = 0;
  if( !grasp_client.call(grasp_planning_srv) )
  {
    ROS_INFO("grasp_planner service call failed. wait...");
    ros::Duration(0.5).sleep();
  }
  if ( grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS)
  { 
    ROS_INFO("Grasp Planning OK");
    my_calculated_grasp = grasp_planning_srv.response.grasp_list;
   }
   else
   {
     ROS_INFO("grasp_planner did not succeed.");
   }
   for (int i =0; i < grasp_planning_srv.response.grasp_list.size(); i++) 
   {
     std::cout << "Grasp Nr." << i <<  std::endl;
     std::cout << grasp_planning_srv.response.grasp_list[i].grasp_trajectory[2].wrist_pose <<  std::endl;   
  }

}

void DemoSimple::executeMovement()
{
  trajectory_planning_srv.request.ordered_grasp = grasp_planning_srv.response.grasp_list;
  trajectory_planning_srv.request.object_list = estimation_srv.response.detected_objects;
  trajectory_planning_srv.request.object_id = 0;
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else
  {
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demoSimple");
  ros::NodeHandle nh;
  
  // Set the freq of the main loop, because now we have a socket client
  ros::Rate loop_rate(15);
  ros::Timer delay();
  
  DemoSimple demo(nh);
  demo.doPoseEstimation();
  demo.planGrasps();
  demo.executeMovement();
/*
 
    // GRASP PLANNING
    std::string grasp_service_name("/grasp_planner_srv");
    if ( !ros::service::waitForService(grasp_service_name, ros::Duration().fromSec(1.0)) && nh.ok() )
    { ROS_ERROR("After one second, the service %s hasn't shown up, so continue to capture the scene...",  grasp_service_name.c_str());
      ros::Duration(2).sleep();  
      continue;     
    }

    grasp_planning_srv.request.ordered_objects = my_detected_objects;
    bool grasp_service_error=false;
    bool trajectory_planning_service_error=false;
    
    int objects_list_size=my_detected_objects.size();
    objects_list_size=10;
  
    while(object_to_grasp_id<objects_list_size && !grasp_service_error && !trajectory_planning_service_error)
    { grasp_planning_srv.request.object_id=object_to_grasp_id;

      if (!ros::service::call(grasp_service_name, grasp_planning_srv) )
      { ROS_ERROR("Call to grasp planning service failed. Continue...");
        grasp_service_error=true;
        ros::Duration(2).sleep();  
        break;
      } 
     
      // pose estimation response validation   
      if (grasp_planning_srv.response.result==grasp_planning_srv.response.SUCCESS)
      { ROS_INFO("Grasp Planning OK");
        my_calculated_grasp = grasp_planning_srv.response.grasp_list;
        
        // TRAJECTORY PLANNING
        std::string trajectory_planning_service_name("/trajectory_planner_srv");
        if ( !ros::service::waitForService(trajectory_planning_service_name, ros::Duration().fromSec(1.0)) && nh.ok() )
        { ROS_ERROR("After one second, the service %s hasn't shown up, so continue to capture the scene...", trajectory_planning_service_name.c_str());
          ros::Duration(2).sleep();  
          trajectory_planning_service_error=true;
          break;
        }
       
        trajectory_planning_srv.request.object_list = my_detected_objects;
        trajectory_planning_srv.request.object_id = object_to_grasp_id;
        trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
                   
        if (!ros::service::call(trajectory_planning_service_name, trajectory_planning_srv) )
        { ROS_ERROR("Call to trajectory planning service failed. Continue...");
          ros::Duration(2).sleep();  
          trajectory_planning_service_error=true;
          break;          
        }
         // trajectory planning response validation
        if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
        { ROS_INFO("Trajectory Planning OK");
          break;
        } 
        
        if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.OTHER_ERROR)
        { ROS_ERROR("trajectory planning service returned error %d. Continue...", trajectory_planning_srv.response.result);
          ros::Duration(2).sleep();  
          trajectory_planning_service_error=true;
          break;          
        }
        
        if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.NO_FEASIBLE_TRAJECTORY_FOUND)        
        {
          ROS_ERROR("No feasible trajectory found, try another object");
          ros::Duration(2).sleep();  
          object_to_grasp_id++; 
        }  
      }  
      
      else
      { if (grasp_planning_srv.response.result==grasp_planning_srv.response.OTHER_ERROR)
        { grasp_service_error=true;
          break;
        }

        //try another obejct  
        if (grasp_planning_srv.response.result==grasp_planning_srv.response.NO_FEASIBLE_GRASP_FOUND)
        { ROS_ERROR("No feasible grasp found, try another object");
          ros::Duration(2).sleep();  
          object_to_grasp_id++;
        }      
      }  
    }

    if (grasp_service_error==true || object_to_grasp_id==objects_list_size)
    { ROS_ERROR("No possible grasping found` ..... ");
       ros::Duration(2).sleep();  
      continue;  
    }

    if (trajectory_planning_service_error==true)
    { ROS_ERROR("There is a problem in the trajectory planning ..... ");
       ros::Duration(2).sleep();  
      continue;  
    }

    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
                        
    // TRAJECTORY EXECUTION
    std::string trajectory_execution_service_name("/trajectory_execution_srv");
    if ( !ros::service::waitForService(trajectory_execution_service_name, ros::Duration().fromSec(1.0)) && nh.ok() )
    { ROS_ERROR("After one second, the service %s hasn't shown up, so continue to capture the scene...", trajectory_execution_service_name.c_str());
       ros::Duration(2).sleep();  
      continue;
    }
    
    trajectory_execution_srv.request.trajectory= my_calculated_trajectory;
    if (!ros::service::call(trajectory_execution_service_name, trajectory_execution_srv))
    { ROS_ERROR("Call to trajectory execution service failed. Continue...");
      ros::Duration(2).sleep();  
      continue;
    }
    
    // trajectory execution response validation
    if (trajectory_execution_srv.response.result == trajectory_execution_srv.response.SUCCESS)
    { ROS_INFO("Trajectory Execution OK");
      ros::Duration(2).sleep();  
    } 
    else 
    { ROS_ERROR("Error in the execution of the trajectory...");
      ros::Duration(2).sleep();  
      continue;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  return 0;
}   