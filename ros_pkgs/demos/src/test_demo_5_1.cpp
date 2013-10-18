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

//Testing file for demo_5_1.cpp set the parameters into the launch file

#include <ros/ros.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

// for the messages used in the services
#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"



namespace demos {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class test_demo_5_1
{
  private:

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_pose_estimation;
    ros::ServiceServer srv_grasp_planning;
    ros::ServiceServer srv_trajectory_planning;
    ros::ServiceServer srv_trajectory_execution;

  public:

  bool setClassParameters_pose(definitions::PoseEstimation::Request &request, 
                              definitions::PoseEstimation::Response &response);

  bool setClassParameters_grasp(definitions::GraspPlanning::Request &request, 
                              definitions::GraspPlanning::Response &response);
  bool setClassParameters_trajectory(definitions::TrajectoryPlanning::Request &request, 
                              definitions::TrajectoryPlanning::Response &response);
  bool setClassParameters_execution(definitions::TrajectoryExecution::Request &request, 
                              definitions::TrajectoryExecution::Response &response);
   
  // constructor
    test_demo_5_1(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // pose_estimation service
      srv_pose_estimation = 
         nh_.advertiseService(nh_.resolveName("/pose_estimation_uibk"), 
                                                       &test_demo_5_1::setClassParameters_pose, this);
      
      // grasp_planning service
      srv_grasp_planning = 
         nh_.advertiseService(nh_.resolveName("/grasp_planner_srv"), 
                                                        &test_demo_5_1::setClassParameters_grasp, this);
      //trajectory_planning service
      srv_trajectory_planning = 
         nh_.advertiseService(nh_.resolveName("/trajectory_planner_srv"), 
                                                        &test_demo_5_1::setClassParameters_trajectory, this);
      //trajectory execution service
      srv_trajectory_execution = 
         nh_.advertiseService(nh_.resolveName("/trajectory_execution_srv"), 
                                                        &test_demo_5_1::setClassParameters_execution, this);
    }

    //! Empty stub
    ~test_demo_5_1() {}

};

bool test_demo_5_1::setClassParameters_pose(definitions::PoseEstimation::Request &request, 
                                          definitions::PoseEstimation::Response &response)
{   ROS_INFO("Pose Estimation service up");
    nh_.param<int>("pose_estimation_response", response.result, response.result);
    return true;  
}

bool test_demo_5_1::setClassParameters_grasp(definitions::GraspPlanning::Request &request, 
                                          definitions::GraspPlanning::Response &response)
{   ROS_INFO("Grasp planning service up");
    nh_.param<int>("grasp_planning_response", response.result, response.result);
    return true;  
}

bool test_demo_5_1::setClassParameters_trajectory(definitions::TrajectoryPlanning::Request &request, 
                                                definitions::TrajectoryPlanning::Response &response)
{   ROS_INFO("Trajectory planning service up");
    nh_.param<int>("trajectory_planning_response", response.result, response.result);
    return true;  
}

bool test_demo_5_1::setClassParameters_execution(definitions::TrajectoryExecution::Request &request, 
                                                definitions::TrajectoryExecution::Response &response)
{   ROS_INFO("Trajectory execution service up");
    nh_.param<int>("trajectory_execution_response", response.result, response.result);
    return true;  
}


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation_uibk_up");
    ros::NodeHandle nh;


    demos::test_demo_5_1 node(nh);



    while(ros::ok())
    {
    //node.doSomething();
    ros::spinOnce();
    }

    return 0;
}
