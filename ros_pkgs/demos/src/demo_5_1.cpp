// The structure is more or less the following:
//
// 1) Start the main loop
// 2) Calls the pose_estimation service
// Input: point cloud of the scene
// Output: ordered list of detected objects
// 3) Calls the grasp_planning service
// Input: ordered list of detected objects
// Output: list of grasps for the first object of the scene
// 4) Calls the trajectory_planning service
// Input: ordered list of detected objects
// lists of detected grasps for the first object of the scene
// Output: trajectory for both the arm and the hand
// 5) Calls the trajectory_execution service
// Input: trajectory for both the arm and the hand
// Output: success/failure//

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <vector>
#include <algorithm>  
// for the messages used in the services
#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"
#include "definitions/ObjectCloudReader.h"

//hand trajectory sdh messages

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

enum stateEval
{
  Fail,
  Success,
  Idle
};

enum States
{
  Start_State,
  PoseEstimate_State,
  PickObject_State,
  PlanGrasp_State,
  PreGraspTraj_State,
  GraspTraj_State,
  PostGraspTraj_State,
  PickGrasp_State
};

enum Event
{
  Start,
  EstimatePose,
  PlanGrasp,
  PlanTrajectory,
  Retreat,
  Stop,
  PickObject,
  PickGrasp
};

const int StatesNum = 8;
const int TransNum = 19;

int transTab[TransNum][StatesNum] = 
{
  {Success,Idle,Idle,Idle,Idle,Idle,Idle,Idle}, // EstimatePose
  {Success,Success,Success,Idle,Idle,Idle,Idle,Idle}, // PlanGrasp
  {Success,Success,Success,Success,Idle,Idle,Idle,Success}, // PlanTrajectory- pre-grasp
  {Success,Success,Success,Success,Success,Idle,Idle,Success}, // PlanTrajectory- grasp
  {Success,Success,Success,Success,Success,Success,Idle,Success}, // PlanTrajectory- post-grasp
  {Success,Success,Success,Success,Success,Fail,Idle,Success}, // PlanTrajectory- post-grasp
  {Success,Success,Success,Success,Success,Success,Success,Success}, // re-start
  {Success,Success,Success,Success,Success,Success,Fail,Success}, // re-start
  {Success,Success,Success,Success,Success,Fail,Fail,Success}, // Retreat
  {Success,Success,Success,Success,Success,Fail,Success,Success}, // Retreat
  {Idle,Idle,Idle,Idle,Idle,Idle,Idle,Idle}, // re-start
  {Success,Fail,Idle,Idle,Idle,Idle,Idle,Idle}, // Stop
  {Fail,Idle,Idle,Idle,Idle,Idle,Idle,Idle}, // Stop
  {Fail,Success,Success,Success,Success,Fail,Fail,Success}, // Stop
  {Fail,Success,Success,Success,Success,Fail,Success,Success}, // Stop
  {Success,Success,Fail,Idle,Idle,Idle,Idle,Idle}, // Stop -- table cleaned
  {Success,Success,Success,Fail,Idle,Idle,Idle,Idle}, // PickNextObject
  {Success,Success,Success,Success,Fail,Idle,Idle,Success}, // PickNextGrasp
  {Success,Success,Success,Success,Fail,Idle,Idle,Fail}, // PickNextObject
};

Event mapEvent[TransNum] = 
{
  EstimatePose,PlanGrasp,PlanTrajectory,PlanTrajectory,PlanTrajectory,PlanTrajectory,
  Start,Start,Retreat,Retreat,Start,Stop,Stop,Stop,Stop,Stop,PickObject,PickGrasp,PickObject
};

const int max_trial_ = 1;
using namespace std;
//Mimicing the hardware grasp_types, replace later by inbuild message
std::vector<float> getTargetAnglesFromGraspType(std::string grasp_type, float close_ratio) {
 
std::vector<float> hand_pose;
int grasp;
if(grasp_type=="cylindrical")
grasp=1;
else if(grasp_type=="parallel")
grasp=2;
else if(grasp_type=="centrical")
grasp=3;
else if(grasp_type=="spherical")
grasp=4;
else if(grasp_type=="rim_open")
grasp=5;
else if(grasp_type=="rim_close")
grasp=6;
else if(grasp_type=="rim_pre_grasp")
grasp=7;
 
if (close_ratio > 1.0)
close_ratio = 1.0;
else if (close_ratio < 0.0)
close_ratio = 0.0;
 
switch(grasp) {
case 1: // Cylindrical
hand_pose.push_back(0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
return hand_pose;
break;
case 2: // Parallel
hand_pose.push_back(0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
return hand_pose;
 
case 3: // Centrical
hand_pose.push_back((60)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
return hand_pose;
 
case 4: // Spherical
hand_pose.push_back((60)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
return hand_pose;

case 5: //Parallel grasp for rim open
hand_pose.push_back(0.);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
return hand_pose;

case 6: // parallel grasp for rim close
hand_pose = std::vector<float>(7,0.15);
return hand_pose;

case 7: //grasp for pre-grasp
hand_pose.push_back(0.0);
hand_pose.push_back(-0.2);
hand_pose.push_back(0.1);
hand_pose.push_back(-0.25);
hand_pose.push_back(0.15);
hand_pose.push_back(-0.25);
hand_pose.push_back(0.15);
return hand_pose;

default:
hand_pose.push_back(0.);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
return hand_pose;
}
}

class DemoSimple
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::ServiceClient pose_client;
    ros::ServiceClient grasp_client;
    ros::ServiceClient trajectory_planner_client;
    ros::ServiceClient trajectory_execution_client;
    ros::Publisher sdh_joint_angles_pub;
    ros::ServiceClient reader_client;
    
    //Stuff
    std::string pose_estimation_service_name;
    std::string grasp_service_name;
    std::string trajectory_planning_service_name;
    std::string trajectory_execution_service_name;
    std::string object_reader_service_name;
    
    //Variables messages
    bool status_robot, status_ros;
    
    std::vector<std::string> joint_names_str;
    
    std::vector<definitions::Object> my_detected_objects;
    std::vector<definitions::Grasp> my_calculated_grasp;
    std::vector<definitions::Trajectory> my_calculated_trajectory;
    
    std::vector<float> my_current_sdh_joints_pregrasp;
    std::vector<float> my_current_sdh_joints;
    std::vector<float> my_current_sdh_joints_pre_;
    
    //Service variables
    definitions::PoseEstimation estimation_srv;
    definitions::GraspPlanning grasp_planning_srv;
    definitions::TrajectoryPlanning trajectory_planning_srv;
    definitions::TrajectoryExecution trajectory_execution_srv;
    definitions::ObjectCloudReader reader_srv;
    geometry_msgs::Pose poseMid;  
    geometry_msgs::Pose robotpose;
    int objectsNum_;
    int grasp_id_;
    int object_id_;
    vector<int> object_count_;

  public:

    int curState_[StatesNum];
    Event event_; 
    stateEval eval_;
    
    bool grasp_success_;
    string arm_;    
    
    bool goToStartPos(bool beginning);
    
    void reconstruct_scene();
    
    int doPoseEstimation();
    
    bool planGrasps(string arm);    
    
    bool executeMovement(bool pre_grasp,int &grasp_id);    
    
    void publishSdhJoints(std::vector<float> positions);
    bool post_grasp(int grasp_id);
    void order_grasp();
    // -------------------------------------------------- //
    Event evaluate_cur_state();
    void perform_event(Event event);
    bool plan_trajectory();
    void goToNextObject();
    void goToNextGrasp();
    
    // constructor
    DemoSimple(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        pose_estimation_service_name = "/pose_estimation_uibk/estimate_poses";
        grasp_service_name = "/grasp_planner_srv";
        trajectory_planning_service_name = "/trajectory_planner_srv";
        trajectory_execution_service_name = "/trajectory_execution_srv";
        object_reader_service_name = "/object_reader";

        pose_client = nh_.serviceClient<definitions::PoseEstimation>(pose_estimation_service_name);
        grasp_client = nh_.serviceClient<definitions::GraspPlanning>(grasp_service_name);
        trajectory_planner_client = nh_.serviceClient<definitions::TrajectoryPlanning>(trajectory_planning_service_name);
        
        trajectory_execution_client = nh_.serviceClient<definitions::TrajectoryExecution>(trajectory_execution_service_name);
	
	if(!nh_.getParam("arm_name",arm_)) 
	  arm_ = "right";
	
        //Grasp execution
	if( arm_ == "right" )
          sdh_joint_angles_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/real/right_sdh/follow_joint_trajectory/goal", 1);
	else
	  sdh_joint_angles_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/real/left_sdh/follow_joint_trajectory/goal", 1);
	//sdh_joint_angles_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/simulation/right_sdh/follow_joint_trajectory/goal", 1);
        reader_client = nh_.serviceClient<definitions::ObjectCloudReader>(object_reader_service_name);

        my_current_sdh_joints.resize(7);
        my_current_sdh_joints_pregrasp.resize(7);
	my_current_sdh_joints_pre_.resize(7);
	
	my_current_sdh_joints_pregrasp = getTargetAnglesFromGraspType("rim_open", 0.3);//size should be checked       
      //  my_current_sdh_joints = getTargetAnglesFromGraspType("rim_close", 1.0); 
	my_current_sdh_joints = getTargetAnglesFromGraspType("cylindrical", 1.0); 
	my_current_sdh_joints_pre_ = getTargetAnglesFromGraspType("rim_pre_grasp", 1.0); 
	
	if( arm_ == "left" )
	{
          joint_names_str.push_back("left_sdh_knuckle_joint");
 	  joint_names_str.push_back("left_sdh_thumb_2_joint");
 	  joint_names_str.push_back("left_sdh_thumb_3_joint");
 	  joint_names_str.push_back("left_sdh_finger_12_joint");
 	  joint_names_str.push_back("left_sdh_finger_13_joint");
 	  joint_names_str.push_back("left_sdh_finger_22_joint");
 	  joint_names_str.push_back("left_sdh_finger_23_joint");
	}
	else if( arm_ == "right" )
	{
   	  joint_names_str.push_back("right_sdh_knuckle_joint");
   	  joint_names_str.push_back("right_sdh_thumb_2_joint");
   	  joint_names_str.push_back("right_sdh_thumb_3_joint");
   	  joint_names_str.push_back("right_sdh_finger_12_joint");
   	  joint_names_str.push_back("right_sdh_finger_13_joint");
   	  joint_names_str.push_back("right_sdh_finger_22_joint");
    	  joint_names_str.push_back("right_sdh_finger_23_joint");
	}
	grasp_success_ = false;
	 
         for( int j = 0; j < StatesNum; j++ )
           curState_[j] = Idle;
	 cout <<"init states:" << endl;
	  for( int j = 0; j < StatesNum; j++ )
           cout << curState_[j] << endl;
	event_ = Start;
    }
    //! Empty stub
    ~DemoSimple() {}

};

Event DemoSimple::evaluate_cur_state()
{  
  Event event = Stop;
  int id  = -1;
  for( int i = 0; i < TransNum; i++ )
  {
    bool found = true;
    for( int j = 0; j < StatesNum; j++ )
    {
      if( transTab[i][j] != curState_[j] )
      {
	found = false;
	break;
      }
    }
    if( found )
    {
      id = i;
      break;
    }
  }
  
  cout << "found id is: " << id << endl;
  if( id >= 0 )
    event = mapEvent[id];
  event_ = event;
  return event;
}

void DemoSimple::perform_event(Event event)
{
  cout << "current state:" << endl;
  for( int j = 0; j < StatesNum; j++ )
  {
    cout << curState_[j] << endl;
  }
  
  switch(event_)
  {
    case EstimatePose:
      cout << "event: pose estimation" << endl;
      doPoseEstimation();
      break;
    case PlanGrasp:
      cout << "event: plan grasps" << endl;
      planGrasps(arm_);
      break;
    case PlanTrajectory:
      cout << "event: plan trajectory" << endl;
      plan_trajectory();
      break;  
    case Retreat: 
      cout << "event: retreat" << endl;
      goToStartPos(false);
      break;
    case Start:
      cout << "event: restart" << endl;
      goToStartPos(false);
      break;
    case Stop: 
      cout << "event: stop" << endl;
      break;
    case PickObject:
      cout << "event: next object" << endl;
      goToNextObject();
      break;   
    case PickGrasp:
      cout << "event: next grasp" << endl;
      goToNextGrasp();
      break;
    default:
      cout << "event: stop" << endl;
      event_ = Stop;
      break;
  }
}

void DemoSimple::publishSdhJoints(std::vector<float> positions) 
{        
        
                control_msgs::FollowJointTrajectoryActionGoal actionGoal;
                control_msgs::FollowJointTrajectoryGoal& goal = actionGoal.goal;
                trajectory_msgs::JointTrajectory& traj = goal.trajectory;
		
                traj.joint_names = joint_names_str;
                trajectory_msgs::JointTrajectoryPoint point;
                //conversion to double
		point.positions.resize(positions.size());
		
		for (int i=0; i < positions.size(); i++)
		{
		  
                point.positions[i] = positions.at(i);
		std::cout << "Joint Position  " << i << " "  << positions.at(i) << std::flush;
		
		}
		
		std::cout << "Joint Position" << point << std::flush;
		
		
		std::vector<trajectory_msgs::JointTrajectoryPoint> pointtrajectory;
		pointtrajectory.push_back(point);
                traj.points = pointtrajectory;
		int nr_trial = 3;
		for(int i = 0; i < 3; i++ ){
                sdh_joint_angles_pub.publish(actionGoal);
                //Secure time for finishing of movement, 2 s
                usleep(2000*1000);
		}
        

        ROS_INFO("FollowJointTrajectoryActionGoals published");
        usleep(1000*1000);
        
}   

bool DemoSimple::goToStartPos(bool beginning)
{
  curState_[Start_State] = Fail;
// for testing wo execution
  //curState_[Start_State] = Success;
  
  //open the hand before planning in the beginning of the demo
  if(beginning)
  {
   publishSdhJoints(my_current_sdh_joints_pregrasp);
  }
  
  bool result = false;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1); 

  //right start position
  if( arm_ == "right")
  {
    robotpose.position.x = 0.1624;
    robotpose.position.y = -0.2599;
    robotpose.position.z = 0.6642;
  
    robotpose.orientation.x = 0.404885;
    robotpose.orientation.y = 0.86333;
    robotpose.orientation.z = -0.139283;
    robotpose.orientation.w = 0.267076;
  }
  
  //left arm start position
  else if( arm_ == "left")
  {
    robotpose.position.x = -0.118831;
    robotpose.position.y = 1.70482;
    robotpose.position.z = 0.728295;

    robotpose.orientation.x = -0.411059;
    robotpose.orientation.y = 0.874099;
    robotpose.orientation.z = 0.0389072;
    robotpose.orientation.w = 0.255866;
  }
  
  std::cout << "Initial position: " << robotpose << std::endl;
  current_trajectory.grasp_trajectory[0].wrist_pose.pose = robotpose;
  current_trajectory.grasp_trajectory[0].wrist_pose.header.frame_id = "world_link";
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(current_trajectory);
  
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  trajectory_planning_srv.request.arm = "left";
  std::vector<definitions::Object> noObject;
  
  trajectory_planning_srv.request.object_list = noObject;
  trajectory_planning_srv.request.object_id = 0;
  trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_CART_GOAL;
  trajectory_planning_srv.request.goal_state.hand.wrist_pose = current_trajectory.grasp_trajectory[0].wrist_pose;
  trajectory_planning_srv.request.goal_state.hand.joints = current_trajectory.grasp_trajectory[0].joints;
  
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else	
  {
    
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
      
    
    ROS_INFO("Executing Trajectory");
    
    //User input
    char a;
    std::cout << "Check if trajectory is ok (y/n)" << std::endl;
    std::cin >> a;
    
    if (a=='y')
    {    
    //Arm movement
    std::cout << "Executing arm trajectory" << std::endl;
    std::cout << "#of found trajectories: "  << my_calculated_trajectory.size() << std::endl;
    my_calculated_trajectory[0].trajectory_id=0;
    trajectory_execution_srv.request.trajectory = my_calculated_trajectory[0]; 
      if( !trajectory_execution_client.call(trajectory_execution_srv) )
      {
	      ROS_INFO("trajectory planner service call failed.");
      }
      else
      {
	      ROS_INFO("trajectory execution call succeeded");
	      curState_[Start_State] = Success;
	      result = true;
      }
        
    }
    else
    {
    std::cout << "Trajectory not valid - restart" << std::endl;
    }
  }
  
  //after grasping, open the hand
  if((!beginning)&&(result))
  {
    publishSdhJoints(my_current_sdh_joints_pregrasp);
  }

  if( curState_[GraspTraj_State] == Fail )
  {
    curState_[PickGrasp_State] = Fail;
    if( (grasp_id_ + 1) < my_calculated_grasp.size() )
    {
      grasp_id_ ++;
      curState_[PickGrasp_State] = Success;  
      curState_[PreGraspTraj_State] = Idle;
      curState_[GraspTraj_State] = Idle;
      curState_[PostGraspTraj_State] = Idle;
    }
  }
  else
  {
    for( int i = 1; i < StatesNum; i++ )
      curState_[i] = Idle;
  }
  return result;
  
}

int DemoSimple::doPoseEstimation()
{
   object_id_ = -1;
   curState_[PoseEstimate_State] = Fail; 
   curState_[PickObject_State] = Fail;
   if( !pose_client.call(estimation_srv))
   {
     ROS_INFO("pose estimation service failed. wait...");
     ros::Duration(0.5).sleep();
   }
   my_detected_objects = estimation_srv.response.detected_objects;
   
   std::cout<< "nr of objects: "<<my_detected_objects.size()<<std::endl;
   if( my_detected_objects.size() > 0 ){
     curState_[PickObject_State] = Success;
     object_count_ = vector<int>(my_detected_objects.size(),0);
     object_id_ = 0;
     std::cout << "I am going for object: " << my_detected_objects[0].name << std::endl;
   }
   objectsNum_ = my_detected_objects.size();
   if( estimation_srv.response.result == estimation_srv.response.SUCCESS )
     curState_[PoseEstimate_State] = Success;  
   
   return my_detected_objects.size();
}

void DemoSimple::goToNextObject()
{
  curState_[PickObject_State] = Fail;
  if( ( object_id_ < 0 ) || ( object_id_ >= objectsNum_ ) )
    return;
  if( ( object_count_[object_id_] >= max_trial_ ) && ( (object_id_ + 1)  < objectsNum_ ) )
    object_id_ ++;
  object_count_[object_id_] ++;
  curState_[PickObject_State] = Success; 
}

void DemoSimple::reconstruct_scene()
{ 
   if( !reader_client.call(reader_srv) ) 
   {
     ROS_INFO("object reader service failed. wait...");
     ros::Duration(0.5).sleep();     
   }
   ROS_INFO("object reader done");
}

bool DemoSimple::planGrasps(string arm)
{
  grasp_id_ = 0;
  curState_[PlanGrasp_State] = Fail;
  curState_[PickGrasp_State] = Fail;
  bool success = false;
  grasp_planning_srv.request.ordered_objects = my_detected_objects;
  grasp_planning_srv.request.arm = arm;
  //grasp_planning_srv.request.object_id = 0;
  grasp_planning_srv.request.object_id = object_id_;
  if( !grasp_client.call(grasp_planning_srv) )
  {
    ROS_INFO("grasp_planner service call failed. wait...");
    ros::Duration(0.5).sleep();
  }
  if( ( grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS) && ( grasp_planning_srv.response.grasp_list.size() > 0 ) )
  {
    success = true;
    ROS_INFO("Grasp Planning OK - assigning last wrist pose for arm trajectory planning");
    
    //Removing all but the best grasp and only keeping last trajectory
    my_calculated_grasp.clear();
    std::cout << "grasp planner response size: " << grasp_planning_srv.response.grasp_list.size() << std::endl;
    for( size_t i = 0; i < grasp_planning_srv.response.grasp_list.size(); i++ )
      my_calculated_grasp.push_back(grasp_planning_srv.response.grasp_list[i]);
    my_current_sdh_joints_pregrasp = getTargetAnglesFromGraspType("rim_open", 0.3);//size should be checked

    my_current_sdh_joints = getTargetAnglesFromGraspType("rim_close", 1.0);   
     
   }
   
   else
   {
     ROS_INFO("grasp_planner did not succeed.");
   }
   order_grasp();
   if(my_calculated_grasp.size() == 0 )
   {
     success = false;
   }
   else
   {
      curState_[PlanGrasp_State] = Success;
      curState_[PickGrasp_State] = Success; 
   }
  return success;
}

void DemoSimple::order_grasp()
{
   cout << "to order the grasps" << endl;
   std::vector<definitions::Grasp> my_calculated_grasp_tmp;
   double threshold = 0.2;
   vector<double> euc_dists;  
   for( size_t i = 0; i <  my_calculated_grasp.size(); i++ )
   {
     geometry_msgs::Pose pre_grasp_cur = my_calculated_grasp[i].grasp_trajectory[0].wrist_pose.pose;
     geometry_msgs::Pose grasp_cur = my_calculated_grasp[i].grasp_trajectory[1].wrist_pose.pose;
     if( ( pre_grasp_cur.position.z < threshold ) || ( grasp_cur.position.z < threshold ) )
       continue;
     my_calculated_grasp_tmp.push_back(my_calculated_grasp[i]);
     
     vector<double> diff(3,0);
     diff[0] = grasp_cur.position.x - robotpose.position.x;
     diff[1] = grasp_cur.position.y - robotpose.position.y;
     diff[2] = grasp_cur.position.z - robotpose.position.z;
     double dist = sqrt(pow(diff[0],2)+pow(diff[1],2)+pow(diff[2],2));
     euc_dists.push_back(dist);
   }
   if( my_calculated_grasp.size() - my_calculated_grasp_tmp.size() > 0 )
     
     cout << "grasps are rejected: " << my_calculated_grasp.size() - my_calculated_grasp_tmp.size() << endl;
   my_calculated_grasp.clear();
   my_calculated_grasp = my_calculated_grasp_tmp;
  
   
   vector<double> euc_dists_sort = euc_dists;
   sort(euc_dists_sort.begin(),euc_dists_sort.end());
   
   std::vector<definitions::Grasp> my_calculated_grasp_sort;
   set<int> seen_ids;
   for( size_t i = 0; i < euc_dists_sort.size(); i++ )
   {
     for( size_t j = 0; j < euc_dists.size(); j++ )
     {
       if( seen_ids.find(j) != seen_ids.end() )
	 continue;
       if( euc_dists[j] == euc_dists_sort[i] )
       {
	 my_calculated_grasp_sort.push_back(my_calculated_grasp[j]);
	 seen_ids.insert(j);
	 break;
       }
     }
   }
   
   my_calculated_grasp.clear();
   my_calculated_grasp = my_calculated_grasp_sort;
   
   for( size_t i = 0; i < my_calculated_grasp.size(); i++ )
   {
     std::cout << "pre:grasp position:" << my_calculated_grasp[i].grasp_trajectory[0].wrist_pose.pose << std::endl;
     std::cout << "grasp position:" << my_calculated_grasp[i].grasp_trajectory[2].wrist_pose.pose << std::endl;
     cout << "distance to start position: " << euc_dists_sort[i] << endl;
   }
}

void DemoSimple::goToNextGrasp()
{
  curState_[PickGrasp_State] = Fail;
  if( (grasp_id_ + 1) < my_calculated_grasp.size() )
  {
    grasp_id_ ++;
    curState_[PickGrasp_State] = Success;  
    curState_[PreGraspTraj_State] = Idle; 
  }
}

bool DemoSimple::plan_trajectory()
{
  bool result = false;
  if( curState_[PreGraspTraj_State] == Idle )
  {
    cout << "to do pre grasp" << endl;
    curState_[PreGraspTraj_State] = Fail;
    // for testing wo execution 
    // curState_[PreGraspTraj_State] = Success;
    executeMovement(true,grasp_id_);
    if( grasp_success_ )
      curState_[PreGraspTraj_State] = Success;
  }
  else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] == Idle ) )
  {
    cout << "to do grasp " << endl;
    curState_[GraspTraj_State] = Fail;
    // for testing wo execution
    //curState_[GraspTraj_State] = Success;
    executeMovement(false,grasp_id_);
    if( grasp_success_ )
      curState_[GraspTraj_State] = Success;
  }
  else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] != Idle ) )
  {
    cout << "to do post-grasp" << endl;
    curState_[PostGraspTraj_State] = Fail;
    // for testing wo execution
    //curState_[PostGraspTraj_State] = Success;
    post_grasp(grasp_id_);
    if( grasp_success_ )
      curState_[PostGraspTraj_State] = Success;    
  }
  result = grasp_success_;
  return result;
}

bool DemoSimple::post_grasp(int grasp_id)
{
  bool success = false;
  double offset_z;
  
  if(my_detected_objects[0].name.data.find("cuttlery") == std::string::npos)
    offset_z  = 0.2; // 0.15
  else
    offset_z  = 0.25;
  
  my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose.pose.position.z += offset_z; 
  my_calculated_grasp[grasp_id].grasp_trajectory[1] = my_calculated_grasp[grasp_id].grasp_trajectory[0];
  my_calculated_grasp[grasp_id].grasp_trajectory[2] = my_calculated_grasp[grasp_id].grasp_trajectory[0];
  success = executeMovement(false,grasp_id);
  
  return success;
}

bool DemoSimple::executeMovement(bool pre_grasp,int &grasp_id)
{
  bool succeed = false;
  grasp_success_ = false;
  if( grasp_id >= my_calculated_grasp.size() )   
  {
    return succeed;
  }
  if( !pre_grasp ) 
  {    
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = 0;
    reconstruct_scene();
    my_calculated_grasp[grasp_id].grasp_trajectory[0] = my_calculated_grasp[grasp_id].grasp_trajectory[2];
  }
  else
  {    
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = -1;
    reconstruct_scene(); 
  }
  std::cout << "grasp id is: " << grasp_id << " : " << "grasp size is: " << my_calculated_grasp.size() << std::endl;
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(my_calculated_grasp[grasp_id]);
  
  //trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
  trajectory_planning_srv.request.arm = "left";
  trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_CART_GOAL;
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  trajectory_planning_srv.request.object_list = estimation_srv.response.detected_objects;
  trajectory_planning_srv.request.object_id = 0;
  trajectory_planning_srv.request.goal_state.hand.wrist_pose = my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose;
  trajectory_planning_srv.request.goal_state.hand.joints = my_calculated_grasp[grasp_id].grasp_trajectory[0].joints;
  //trajectory_planning_srv.request.object_id = grasp_id;
  
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
    return false;
  }
  else	
  {
    ROS_INFO("trajectory planner call succeeded");
    
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
    
    int nr_found_trajectories = my_calculated_trajectory.size();
    
    if(nr_found_trajectories > 0) 
    {
       ROS_INFO("Executing Pick and place");
    
       //User input
      char a;
      std::cout << "Check if trajectory is ok (y/n)" << std::endl;
      std::cin >> a;
    
      if (a=='y')
      {    
       
      //Hand Pre-grasp
      //publishSdhJoints(my_current_sdh_joints_pregrasp);
      
      //Arm movement
        std::cout << "Executing arm trajectory" << std::endl;
        my_calculated_trajectory[0].trajectory_id=0;
      
       // trajectory_execution_srv.request.trajectory = my_calculated_trajectory; //execute 0 trajectory id
        trajectory_execution_srv.request.trajectory = my_calculated_trajectory[0];
	
        if( !trajectory_execution_client.call(trajectory_execution_srv) )
        {
	      ROS_INFO("trajectory planner service call failed.");
        }
        else
        {
	// succeed = true;
	      grasp_success_ = true;
	      ROS_INFO("trajectory execution call succeeded");
	      succeed = true;
	      if( !pre_grasp )
	        publishSdhJoints(my_current_sdh_joints);
	      //else // pacman intermediate joints 
		//publishSdhJoints(my_current_sdh_joints_pre_);
	      return true;
        }
    
    //GraspPlanning
    
   
    
    //TODO: Move to Place positions, to be done after testing, query trajectory planning service again.
    
      }
      else if (a=='n')
      {
        std::cout << "Trajectory not valid - restart" << std::endl;
    
        bool grasp_success = planGrasps(arm_);
    
        if( !grasp_success )
          return false;
    
        executeMovement(pre_grasp,grasp_id);
    
      }
     /* else if( (a=='g') && ( (grasp_id+1) < (my_calculated_grasp.size()) ) && (pre_grasp) )
      {
        grasp_id++;
        std::cout << "user want another grasp, pre-grasp id: " <<grasp_id<< std::endl;
        executeMovement(pre_grasp,grasp_id);	
      }*/
      else
        return false;
    }
    else if( ( (grasp_id+1) < (my_calculated_grasp.size()) ) && (pre_grasp) )
    { 
      
      grasp_id++;
      std::cout << "Plan another pre-grasp id: " <<grasp_id<< std::endl;
      executeMovement(pre_grasp,grasp_id);
    }
    else if( ( (grasp_id+1) >= (my_calculated_grasp.size()) ) && (pre_grasp) )
    {
      std::cout << "No plan found for pre-grasp id: " <<grasp_id<< std::endl;
      return false;
    }
    else if( !pre_grasp )
    {
      std::cout << "No plan found for grasp, therefore open hand " << std::endl;
     // publishSdhJoints(my_current_sdh_joints_pregrasp);
      return false;
    }
  }
  
  std::cout << "Everything succeded inside execute movement " << std::endl;
//  succeed = true; 
  return succeed;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demoSimple");
  ros::NodeHandle nh;
  
  // Set the freq of the main loop, because now we have a socket client
  ros::Rate loop_rate(15);
  ros::Timer delay();
  
  DemoSimple demo(nh);
  
  cout << "arm name is: " << demo.arm_ << "." << endl;

  while( demo.event_ != Stop )
  {
    Event event = demo.evaluate_cur_state();
    demo.perform_event(event);
  }
  
  return 0;
}
