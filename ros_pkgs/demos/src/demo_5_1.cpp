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
#include <definitions/Grasp.h>

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
  Restart,
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
  {Idle,Idle,Idle,Idle,Idle,Idle,Idle,Idle}, // start
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
  Restart,Restart,Retreat,Retreat,Start,Stop,Stop,Stop,Stop,Stop,PickObject,PickGrasp,PickObject
};

const int max_trial_ = 1;
const int max_trial_start_ = 5;
using namespace std;

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
    
    //Service variables
    definitions::PoseEstimation estimation_srv;
    definitions::GraspPlanning grasp_planning_srv;
    definitions::TrajectoryPlanning trajectory_planning_srv;
    definitions::TrajectoryExecution trajectory_execution_srv;
    definitions::ObjectCloudReader reader_srv;
    ros::Publisher pub_cur_grasp_;
    geometry_msgs::Pose poseMid;  
    geometry_msgs::Pose robotpose;

    geometry_msgs::Pose robotpose_right;
    geometry_msgs::Pose robotpose_left;
    vector<float> robot_start_joints_right;
    vector<float> robot_place_joints_right;
    vector<float> robot_start_joints_left;
    vector<float> robot_place_joints_left;    

    vector<float> robot_start_joints_;
    vector<float> robot_place_joints_;
    
    int objectsNum_;
    int grasp_id_;
    int object_id_;
    vector<int> object_count_;
    vector<float> hand_pose_start_;
    string available_arm_;
  public:

    int curState_[StatesNum];
    Event event_; 
    stateEval eval_;
    
    bool grasp_success_;
    string arm_;    
    
    bool goToStartPos();
    
    void reconstruct_scene();
    
    int doPoseEstimation();
    
    bool planGrasps(string arm);    
    
    bool executeMovement(bool pre_grasp,int &grasp_id,int traj_id=2);    
    
    bool post_grasp(int grasp_id);
    void order_grasp();

    Event evaluate_cur_state();
    void perform_event(Event event);
    bool plan_trajectory();
    void goToNextObject();
    void goToNextGrasp();
    bool restart(bool place,bool drop);
    void select_arm();
    
    // constructor
    DemoSimple(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        pose_estimation_service_name = "/pose_estimation_uibk/estimate_poses";
        grasp_service_name = "/grasp_planner_srv";
       // trajectory_planning_service_name = "/trajectory_planner_srv";
        trajectory_planning_service_name = "/trajectory_planning_srv";
        trajectory_execution_service_name = "/trajectory_execution_srv";
        object_reader_service_name = "/object_reader";

        pose_client = nh_.serviceClient<definitions::PoseEstimation>(pose_estimation_service_name);
        grasp_client = nh_.serviceClient<definitions::GraspPlanning>(grasp_service_name);
        trajectory_planner_client = nh_.serviceClient<definitions::TrajectoryPlanning>(trajectory_planning_service_name);
        
        trajectory_execution_client = nh_.serviceClient<definitions::TrajectoryExecution>(trajectory_execution_service_name);
	
	      if(!nh_.getParam("arm_name",arm_)) 
	        arm_ = "right";
	      available_arm_ = arm_;
        //Grasp execution
        reader_client = nh_.serviceClient<definitions::ObjectCloudReader>(object_reader_service_name);
	      grasp_success_ = false;

        pub_cur_grasp_ = nh_.advertise<definitions::Grasp>(nh_.resolveName("/grasp_planner/cur_grasp"), 1);
	 
      for( int j = 0; j < StatesNum; j++ )
           curState_[j] = Idle;
	      cout <<"init states:" << endl;
	     for( int j = 0; j < StatesNum; j++ )
           cout << curState_[j] << endl;
	     event_ = Start;

      // * open hand at the begining * // 
       hand_pose_start_.push_back(0.); hand_pose_start_.push_back(-0.5); hand_pose_start_.push_back(0.5);
       hand_pose_start_.push_back(-0.5); hand_pose_start_.push_back(0.5); hand_pose_start_.push_back(-0.5);
       hand_pose_start_.push_back(0.5);
       
       robot_start_joints_ = vector<float>(7,0);
       robot_place_joints_ = vector<float>(7,0);

       robot_start_joints_right = vector<float>(7,0);
       robot_place_joints_right = vector<float>(7,0);
       robot_start_joints_left = vector<float>(7,0);
       robot_place_joints_left = vector<float>(7,0);              
       cout << "arm name is: " << available_arm_ << endl;
       // ** start position ** //
       if( ( available_arm_ == "right") || (available_arm_ == "both") )
       {
        cout << "to initialize right arm" << endl;
     /*    robotpose_right.position.x = 0.1624;
         robotpose_right.position.y = -0.2599;
         robotpose_right.position.z = 0.6642;
  
         robotpose_right.orientation.x = 0.404885;
         robotpose_right.orientation.y = 0.86333;
         robotpose_right.orientation.z = -0.139283;
         robotpose_right.orientation.w = 0.267076;*/

         robotpose_right.position.x = 0.064062;
         robotpose_right.position.y = -0.17281;
         robotpose_right.position.z = 0.9236;
  
         robotpose_right.orientation.x = 0.43361;
         robotpose_right.orientation.y = 0.40981;
         robotpose_right.orientation.z = 0.26315;
         robotpose_right.orientation.w = 0.75815;         

	       robot_start_joints_right[0] = 0.90358; robot_start_joints_right[1] = 1.07305; robot_start_joints_right[2] = 1.16986; robot_start_joints_right[3] = 0.89418;
	       robot_start_joints_right[4] = 0.74461; robot_start_joints_right[5] = 0.01476; robot_start_joints_right[6] = -0.48848;
	 
	      // robot_place_joints_[0] = 0.9035800099372864; robot_place_joints_[1] = 1.0730500221252441; robot_place_joints_[2] = 1.1698600053787231; robot_place_joints_[3] = 0.8941799998283386;
	       //robot_place_joints_[4] = 1.8446100115776062; robot_place_joints_[5] = 0.814759999699890614; robot_place_joints_[6] = -0.48848000168800354;	 

         robot_place_joints_right[0] = 0.9035800099372864; robot_place_joints_right[1] = 0.4730500221252441; robot_place_joints_right[2] = 1.1698600053787231; 
         robot_place_joints_right[3] = 0.3941799998283386; robot_place_joints_right[4] = 1.8446100115776062; robot_place_joints_right[5] = 1.514759999699890614; 
         robot_place_joints_right[6] = -0.48848000168800354;  
	
       }
  
      //left arm start position
      if( ( available_arm_ == "left") || ( available_arm_ == "both" ) )
      {	
        cout << "to initialize left arm" << endl;
        /*robotpose_left.position.x = -0.118831;
        robotpose_left.position.y = 1.70482;
        robotpose_left.position.z = 0.728295;

        robotpose_left.orientation.x = -0.411059;
        robotpose_left.orientation.y = 0.874099;
        robotpose_left.orientation.z = 0.0389072;
        robotpose_left.orientation.w = 0.255866;  */

        robotpose_left.position.x = 0.0043042;
        robotpose_left.position.y = 1.5781;
        robotpose_left.position.z = 0.87247;

        robotpose_left.orientation.x = -0.092165;
        robotpose_left.orientation.y = 0.59065;
        robotpose_left.orientation.z = 0.28223;
        robotpose_left.orientation.w = 0.75032;          

	      robot_start_joints_left[0] = 0.98819; robot_start_joints_left[1] = -1.01639; robot_start_joints_left[2] = 2.00266; robot_start_joints_left[3] = 0.98314;
	      robot_start_joints_left[4] = 0.0; robot_start_joints_left[5] = 0.0; robot_start_joints_left[6] = 1.25715;

      // 	robot_place_joints_left[0] = 0.98819; robot_place_joints_left[1] = -1.01639; robot_place_joints_left[2] = 2.00266; robot_place_joints_left[3] = 0.98314;
	      //robot_place_joints_left[4] = 0.0; robot_place_joints_left[5] = 0.0; robot_place_joints_left[6] = 1.25715 + 0.3;

        robot_place_joints_left[0] = 0.98819; robot_place_joints_left[1] = -0.4730500221252441; robot_place_joints_left[2] = 2.00266; 
        robot_place_joints_left[3] = 0.3941799998283386; robot_place_joints_left[4] = 1.5; robot_place_joints_left[5] = -1.1; 
        robot_place_joints_left[6] = 1.25715; 
      }
      
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
  int count;
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
      count = 1;
      goToStartPos();
      while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) ){
	      count++;
        goToStartPos();
      }
      break;
    case Start:
      cout << "event: start" << endl;
      if( available_arm_ == "both" )
      {
        arm_ = "right";
        count = 1;
        do{
          goToStartPos();
          count ++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );    

        if( curState_[Start_State] == Success )
        {
          arm_ = "left";
          count = 1;
          do{
            goToStartPos();
            count ++;
          }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) ); 
        }              
      }
      else
      {
        count = 1;
        do{
          goToStartPos();
          count ++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );       
      }
      break;
    case Restart:
      cout << "event: restart" << endl;
      cout << "go to start pose with grasp joint" << endl;
      restart(false,false);
      count = 1;
      while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) )
      {
	      count ++;
	      restart(false,false);
      }
      count = 1;
      if( curState_[Start_State] == Success )
      {
	      cout << "go to place position!!" << endl;
        do{
          restart(true,false);
          count++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );
      }
      if( curState_[Start_State] == Success )
      {
        cout << "to drop object!!!" << endl;
        restart(true,true);
      }
      count = 1;
      if( curState_[Start_State] == Success )
      {
        cout << "come back to start position after droping!!" << endl;
        do
        {
          goToStartPos();
          count++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );
      }      
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

bool DemoSimple::goToStartPos()
{
  curState_[Start_State] = Fail;
  // ** to test wo execution //
  //curState_[Start_State] = Success;

  if( arm_ == "right" ){
    robotpose = robotpose_right;
    robot_start_joints_ = robot_start_joints_right;
  }
  else if( arm_ == "left" )
  {
    robotpose = robotpose_left;
    robot_start_joints_ = robot_start_joints_left;    
  }

  bool result = false;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1); 
  
  std::cout << "Initial position: " << robotpose << std::endl;
  current_trajectory.grasp_trajectory[0].wrist_pose.pose = robotpose;
  current_trajectory.grasp_trajectory[0].wrist_pose.header.frame_id = "world_link";
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(current_trajectory);
  
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  //trajectory_planning_srv.request.arm = "right";
  trajectory_planning_srv.request.arm = arm_;
  std::vector<definitions::Object> noObject;
  
  trajectory_planning_srv.request.object_list = noObject;
  trajectory_planning_srv.request.object_id = 0;
  trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_STATE_GOAL;
  if( arm_ == "right" )
  {
    trajectory_planning_srv.request.eddie_goal_state.armRight.joints = robot_start_joints_;
    trajectory_planning_srv.request.eddie_goal_state.handRight.joints = hand_pose_start_;
  }
  else if( arm_ == "left" )
  {
    trajectory_planning_srv.request.eddie_goal_state.armLeft.joints = robot_start_joints_;
    trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = hand_pose_start_;
  }
  
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else	
  {
    
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
    
    if( my_calculated_trajectory.size() > 0 )
    {
    
      ROS_INFO("Executing Trajectory");
    
      //User input
      string answer;
      std::cout << "Check if trajectory is ok (y/n)" << std::endl;
      std::cin >> answer;
    
      if (answer == "y")
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
    else
      std::cout << "no trajectory found" << std::endl;
  }
   // ** retreat ** //
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

bool DemoSimple::restart(bool place,bool drop)
{
  curState_[Start_State] = Fail;
  // ** to test wo execution **//
 // curState_[Start_State] = Success;

  if( arm_ == "right" ){
    robotpose = robotpose_right;
    robot_start_joints_ = robot_start_joints_right;
    robot_place_joints_ = robot_place_joints_right;
  }
  else if( arm_ == "left" )
  {
    robotpose = robotpose_left;
    robot_start_joints_ = robot_start_joints_left;  
    robot_place_joints_ = robot_place_joints_left;  
  }

  bool result = false;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1); 
  
  std::cout << "Initial position: " << robotpose << std::endl;
  current_trajectory.grasp_trajectory[0].wrist_pose.pose = robotpose;
  current_trajectory.grasp_trajectory[0].wrist_pose.header.frame_id = "world_link";
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(current_trajectory);
  
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  //trajectory_planning_srv.request.arm = "right";
  trajectory_planning_srv.request.arm = arm_;
  std::vector<definitions::Object> noObject;
  
  trajectory_planning_srv.request.object_list = noObject;
  trajectory_planning_srv.request.object_id = 0;
  trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_STATE_GOAL;
  if( !place )
  {
    if( arm_ == "right" )
    {
      trajectory_planning_srv.request.eddie_goal_state.armRight.joints = robot_start_joints_;
      trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;
    }
    else if( arm_ == "left" )
    {
      trajectory_planning_srv.request.eddie_goal_state.armLeft.joints = robot_start_joints_;
      trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;
    }  
  }
  else
  {
    if( arm_ == "right" )
      trajectory_planning_srv.request.eddie_goal_state.armRight.joints = robot_place_joints_;
    else if( arm_ == "left" )
      trajectory_planning_srv.request.eddie_goal_state.armLeft.joints = robot_place_joints_;

    if( drop )
    {
      if( arm_ == "right" )
        trajectory_planning_srv.request.eddie_goal_state.handRight.joints = hand_pose_start_;
      else if( arm_ == "left" )
        trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = hand_pose_start_;      
    }
    else
    {
      if( arm_ == "right" )
        trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;
      else if( arm_ == "left" )
        trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;      
    }
  }
  
  
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else	
  {
    
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
      
    if( my_calculated_trajectory.size() > 0 ) 
    {
      ROS_INFO("Executing Trajectory");
    
    //User input
      string answer;
      std::cout << "Check if trajectory is ok (y/n)" << std::endl;
      std::cin >> answer;
    
      if (answer == "y")
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
    else
      cout << "no trajectory found" << endl;
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
   if( ( estimation_srv.response.result == estimation_srv.response.SUCCESS ) && ( my_detected_objects.size() > 0 ) )
     curState_[PoseEstimate_State] = Success;  
  
   if( available_arm_ == "both" )
     select_arm(); 

   return my_detected_objects.size();
}

void DemoSimple::goToNextObject()
{
  curState_[PickObject_State] = Fail;
  //curState_[PoseEstimate_State] = Fail; 
  if( ( object_id_ < 0 ) || ( object_id_ >= objectsNum_ ) )
    return;
  if( ( object_count_[object_id_] >= max_trial_ ) &&  ( (object_id_ + 1)  < objectsNum_ ) )
    object_id_ ++;
  else if( ( object_count_[object_id_] >= max_trial_ ) &&  ( (object_id_ + 1)  >= objectsNum_ ) ) 
    return;
  object_count_[object_id_] ++;
  cout << "In next object: I am going for object: " << my_detected_objects[object_id_].name << endl;
  curState_[PickObject_State] = Success;
 // curState_[PoseEstimate_State] = Success; 
  for( int i = (PickObject_State + 1 ); i < StatesNum; i++ )
    curState_[i] = Idle;

  if( available_arm_ == "both" )
    select_arm();
}

void DemoSimple::select_arm()
{
  geometry_msgs::Pose obj_pose = my_detected_objects[object_id_].pose;
  double dist_right = sqrt(pow((obj_pose.position.x - robotpose_right.position.x),2)+
                      pow((obj_pose.position.y - robotpose_right.position.y),2)+
                      pow((obj_pose.position.z - robotpose_right.position.z),2));


  double dist_left = sqrt(pow((obj_pose.position.x - robotpose_left.position.x),2)+
                      pow((obj_pose.position.y - robotpose_left.position.y),2)+
                      pow((obj_pose.position.z - robotpose_left.position.z),2));
  if( dist_left < dist_right )
    arm_ = "left";
  else 
    arm_ = "right";
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
   // std::cout << "grasp planner response size: " << grasp_planning_srv.response.grasp_list.size() << std::endl;
    for( size_t i = 0;((i < grasp_planning_srv.response.grasp_list.size() )&& ( i < 5 ) ); i++ )
      my_calculated_grasp.push_back(grasp_planning_srv.response.grasp_list[i]);    
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
  if( arm_ == "right" )
    robotpose = robotpose_right;
  else if( arm_ == "left" )
    robotpose = robotpose_left;   

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
   
   /*for( size_t i = 0; i < my_calculated_grasp.size(); i++ )
   {
     std::cout << "pre:grasp position:" << my_calculated_grasp[i].grasp_trajectory[0].wrist_pose.pose << std::endl;
     std::cout << "grasp position:" << my_calculated_grasp[i].grasp_trajectory[2].wrist_pose.pose << std::endl;
     cout << "distance to start position: " << euc_dists_sort[i] << endl;
   }*/
}

void DemoSimple::goToNextGrasp()
{
  curState_[PickGrasp_State] = Fail;
//  curState_[PlanGrasp_State] = Fail;
  if( (grasp_id_ + 1) < my_calculated_grasp.size() )
  {
    grasp_id_ ++;
    curState_[PickGrasp_State] = Success;  
    curState_[PreGraspTraj_State] = Idle; 
  //  curState_[PlanGrasp_State] = Success;
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

    executeMovement(false,grasp_id_,1);
    if( grasp_success_ )
    {
      executeMovement(false,grasp_id_);
      if( grasp_success_ )
        curState_[GraspTraj_State] = Success;
    }
    /*executeMovement(false,grasp_id_);
    if( grasp_success_ )
      curState_[GraspTraj_State] = Success;*/
  }
  else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] != Idle ) )
  {
    cout << "to do post-grasp" << endl;
    curState_[PostGraspTraj_State] = Fail;
    // for testing wo execution
   // curState_[PostGraspTraj_State] = Success;
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

bool DemoSimple::executeMovement(bool pre_grasp,int &grasp_id,int traj_id)
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
    reader_srv.request.object_id = object_id_;
    reconstruct_scene();
    usleep(1000*1000);
   // my_calculated_grasp[grasp_id].grasp_trajectory[0] = my_calculated_grasp[grasp_id].grasp_trajectory[2];
    my_calculated_grasp[grasp_id].grasp_trajectory[0] = my_calculated_grasp[grasp_id].grasp_trajectory[traj_id];
  }
  else
  {    
    pub_cur_grasp_.publish(my_calculated_grasp[grasp_id]);
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = -1;
    reconstruct_scene();
    usleep(1000*1000); 
  }
  std::cout << "grasp id is: " << grasp_id << " : " << "grasp size is: " << my_calculated_grasp.size() << std::endl;
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(my_calculated_grasp[grasp_id]);

 // std::cout << "my_calculated_grasp[grasp_id]" << my_calculated_grasp[grasp_id] << std::endl;

  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
  //trajectory_planning_srv.request.arm = "right";
  trajectory_planning_srv.request.arm = arm_;
  trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_CART_GOAL;
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  trajectory_planning_srv.request.object_list = estimation_srv.response.detected_objects;
  trajectory_planning_srv.request.object_id = 0;
  if( arm_ == "right" )
  {
    trajectory_planning_srv.request.eddie_goal_state.handRight.wrist_pose = my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose;
    trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id].grasp_trajectory[0].joints;
  }
  else if( arm_ == "left" )
  {
    trajectory_planning_srv.request.eddie_goal_state.handLeft.wrist_pose = my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose;
    trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id].grasp_trajectory[0].joints;    
  }
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
      string answer;
      std::cout << "Check if trajectory is ok (y/n)" << std::endl;
      std::cin >> answer;
    
      if (answer == "y")
      {    
      
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
	      return true;
        }
      }
      else if (answer == "n")
      {
        std::cout << "Trajectory not valid - restart" << std::endl;
    
        bool grasp_success = planGrasps(arm_);
    
        if( !grasp_success )
          return false;
    
        executeMovement(pre_grasp,grasp_id);
    
      }
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
