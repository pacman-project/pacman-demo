// system headers
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <ros/message_operations.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <algorithm> 

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

// generated headers from the definitions package
#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"
#include "definitions/ObjectCloudReader.h"
#include "definitions/Grasp.h"
//#include "definitions/StateMachineList.h"

// STATE MACHINE
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
  PickGrasp_State,
  N_STATES
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
  PickGrasp,
  N_EVENTS
};

const int StatesNum = N_STATES;
const int TransNum = 20;

int transTab[TransNum][StatesNum] = 
{
  {Success,Idle,Idle,Idle,Idle,Idle,Idle,Idle}, // EstimatePose
  {Success,Success,Success,Idle,Idle,Idle,Idle,Idle}, // PlanGrasp
  {Success,Success,Success,Success,Idle,Idle,Idle,Success}, // PlanTrajectory- pre-grasp
  {Success,Success,Success,Success,Success,Idle,Idle,Success}, // PlanTrajectory- grasp
  {Success,Success,Success,Success,Success,Success,Idle,Success}, // PlanTrajectory- post-grasp
  {Success,Success,Success,Success,Success,Fail,Idle,Success}, // Retreat
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
  {Success,Success,Success,Fail,Idle,Idle,Idle,Fail}, // PickNextObject
  {Success,Success,Success,Success,Fail,Idle,Idle,Success}, // PickNextGrasp
  {Success,Success,Success,Success,Fail,Idle,Idle,Fail}, // PickNextObject
};

Event mapEvent[TransNum] = 
{
  EstimatePose,
  PlanGrasp,
  PlanTrajectory,
  PlanTrajectory,
  PlanTrajectory,
  Retreat,
  Restart,
  Restart,
  Retreat,
  Retreat,
  Start,
  Stop,
  Stop,
  Stop,
  Stop,
  Stop,
  PickObject,
  PickObject,
  PickGrasp,
  PickObject
};
// STATE MACHINE

const int max_trial_ = 1;
const int max_trial_start_ = 5;
const int max_pose_estimation_trial_ = 2;

using namespace std;

class DemoSimple
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    
    // Service clients
    ros::ServiceClient pose_client_;
    ros::ServiceClient grasp_client_;
    ros::ServiceClient trajectory_planner_client_;
    ros::ServiceClient trajectory_execution_client_;
    ros::ServiceClient reader_client_;
    
    // Service names
    std::string pose_estimation_service_name;
    std::string grasp_service_name;
    std::string trajectory_planning_service_name;
    std::string trajectory_execution_service_name;
    std::string object_reader_service_name;
    
    // Service variables
    definitions::PoseEstimation estimation_srv;
    definitions::GraspPlanning grasp_planning_srv;
    definitions::TrajectoryPlanning trajectory_planning_srv;
    definitions::TrajectoryExecution trajectory_execution_srv;
    definitions::ObjectCloudReader reader_srv;

    // Vector of results of states
    std::vector<definitions::Object> my_detected_objects;
    std::vector<definitions::Grasp> my_calculated_grasp;
    std::vector<definitions::Trajectory> my_calculated_trajectory;
   
    // Predefined poses for the demo
    string available_arm_;
    geometry_msgs::Pose robotpose;
    geometry_msgs::Pose robotpose_right_;
    geometry_msgs::Pose robotpose_left_;
    vector<float> robot_start_joints_right;
    vector<float> robot_place_joints_right;
    vector<float> robot_start_joints_left;
    vector<float> robot_place_joints_left;    
    vector<float> robot_start_joints_;
    vector<float> robot_place_joints_;
    vector<float> hand_pose_start_;
    
    // Misc
    ros::Publisher pub_cur_grasp_; 
    ros::Publisher pub_clear_;
    int objectsNum_;
    int grasp_id_;
    int object_id_;
    vector<int> object_count_;

    // Logging variables
    log4cxx::LoggerPtr my_logger;
    ofstream ofs;
    string log_path_;

    // State machine variables
    ros::Publisher pub_states_;
    ros::Publisher pub_events_;
    map<int,string> event_map_;
    States cur_state_;
    bool grasp_ordering_;
    string select_arm_factor;

  public:

    // State machine
    int curState_[StatesNum];
    Event event_; 
    stateEval eval_;
    int pose_trial_;
    
    bool grasp_success_;
    string arm_;    
    
    // State functions
    bool goToStartPos(bool user_debug=true);
    int doPoseEstimation();
    void reconstruct_scene();
    void goToNextObject();
    bool planGrasps(string arm);
    void order_grasp();
    bool post_grasp(int grasp_id);
    void goToNextGrasp();
    bool plan_trajectory();
    bool executeMovement(bool pre_grasp,bool post_grasp,int &grasp_id,int traj_id=2,bool user_debug=true);    
    bool restart(bool place,bool drop,bool user_debug=true);

    // State Machine evaluations
    Event evaluate_cur_state();
    void perform_event(Event event);

    // helper functions
    // arm selection based on grasp
    void select_arm();
    // arm selection based on object poistion
    void select_arm_object();

    // constructor
    DemoSimple(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      ROS_DEBUG("DEMO Started!");

      // logging init
      if(!nh_.getParam("log_path",log_path_)) 
        log_path_ = "/tmp/";
      stringstream ss_log;
      ss_log << log_path_ << "demo_" << ros::Time::now() << ".log";
      string log_path_str = ss_log.str();
      ofs.open(log_path_str.c_str());
      my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
      my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

      // service names init
      pose_estimation_service_name = "/pose_estimation_uibk/estimate_poses";
      grasp_service_name = "/grasp_planner_srv";
      trajectory_planning_service_name = "/trajectory_planning_srv";
      trajectory_execution_service_name = "/trajectory_execution_srv";
      object_reader_service_name = "/object_reader";

      // service client init
      pose_client_ = nh_.serviceClient<definitions::PoseEstimation>(pose_estimation_service_name);
      grasp_client_ = nh_.serviceClient<definitions::GraspPlanning>(grasp_service_name);
      trajectory_planner_client_ = nh_.serviceClient<definitions::TrajectoryPlanning>(trajectory_planning_service_name);
      trajectory_execution_client_ = nh_.serviceClient<definitions::TrajectoryExecution>(trajectory_execution_service_name);
      reader_client_ = nh_.serviceClient<definitions::ObjectCloudReader>(object_reader_service_name);

      if(!nh_.getParam("arm_name",arm_)) 
        arm_ = "right";
      available_arm_ = arm_;
      //Grasp execution
      grasp_success_ = false;

      if(!nh_.getParam("order_grasp_scene",grasp_ordering_)) 
        grasp_ordering_ = true;     

      if(!nh_.getParam("select_arm_factor",select_arm_factor)) 
        select_arm_factor = "position_based";   

      // publisher init
      pub_cur_grasp_ = nh_.advertise<definitions::Grasp>(nh_.resolveName("/grasp_planner/cur_grasp"), 1);
      pub_clear_ = nh_.advertise<std_msgs::String>(nh_.resolveName("/visualization/clear_all"), 1);
      pub_states_ = nh_.advertise<std_msgs::String>(nh_.resolveName("/pacman_visualisation/html_info"),1);
      pub_events_ = nh_.advertise<std_msgs::String>(nh_.resolveName("/pacman_visualisation/toast_info"),1,true);

      for( int j = 0; j < StatesNum; j++ )
        curState_[j] = Idle;
        //cout <<"init states:" << endl;
      for( int j = 0; j < StatesNum; j++ )
        //cout << curState_[j] << endl;

      event_ = Start;
      cur_state_ = Start_State;

      event_map_[Start] = "Start";
      event_map_[EstimatePose] = "Estimate Pose";
      event_map_[PlanGrasp] = "Plan Grasp";
      event_map_[PlanTrajectory] = "Plan Trajectory";
      event_map_[Retreat] = "Retreat";
      event_map_[Restart] = "Restart";
      event_map_[Stop] = "Stop";
      event_map_[PickObject] = "Select Object";
      event_map_[PickGrasp] = "Select Grasp";

      pose_trial_ = 0;
      // predefined poses init //
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

      cout << "Arm to be used: " << available_arm_ << endl;
      // right arm start position
      if( ( available_arm_ == "right") || (available_arm_ == "both") )
      {
       cout << "Setting up the right arm..." << endl;

       robotpose_right_.position.x = 0.064062;
       robotpose_right_.position.y = -0.17281;
       robotpose_right_.position.z = 0.9236;

       robotpose_right_.orientation.x = 0.43361;
       robotpose_right_.orientation.y = 0.40981;
       robotpose_right_.orientation.z = 0.26315;
       robotpose_right_.orientation.w = 0.75815;         

       robot_start_joints_right[0] = 0.90358; robot_start_joints_right[1] = 1.07305; robot_start_joints_right[2] = 1.16986; robot_start_joints_right[3] = 0.89418;
       robot_start_joints_right[4] = 0.74461; robot_start_joints_right[5] = 0.01476; robot_start_joints_right[6] = -0.48848;

       robot_place_joints_right[0] = 0.9035800099372864; robot_place_joints_right[1] = 0.4730500221252441; robot_place_joints_right[2] = 1.1698600053787231; 
       robot_place_joints_right[3] = 0.3941799998283386; robot_place_joints_right[4] = 1.8446100115776062; robot_place_joints_right[5] = 1.514759999699890614; 
       robot_place_joints_right[6] = -0.48848000168800354;  

      }
  
      // left arm start position
      if( ( available_arm_ == "left") || ( available_arm_ == "both" ) )
      {	
        cout << "Setting up the left arm..." << endl;

        robotpose_left_.position.x = 0.0043042;
        robotpose_left_.position.y = 1.5781;
        robotpose_left_.position.z = 0.87247;

        robotpose_left_.orientation.x = -0.092165;
        robotpose_left_.orientation.y = 0.59065;
        robotpose_left_.orientation.z = 0.28223;
        robotpose_left_.orientation.w = 0.75032;          

        robot_start_joints_left[0] = 0.98819; robot_start_joints_left[1] = -1.01639; robot_start_joints_left[2] = 2.00266; robot_start_joints_left[3] = 0.98314;
        robot_start_joints_left[4] = 0.0; robot_start_joints_left[5] = 0.0; robot_start_joints_left[6] = 1.25715;

        robot_place_joints_left[0] = 0.98819; robot_place_joints_left[1] = -0.4730500221252441; robot_place_joints_left[2] = 2.00266; 
        robot_place_joints_left[3] = 0.3941799998283386; robot_place_joints_left[4] = 1.5; robot_place_joints_left[5] = -1.1; 
        robot_place_joints_left[6] = 1.25715; 
      }
      std_msgs::String msg;
      msg.data = "state_machine";
      pub_states_.publish(msg);
      usleep(1000*1000);

      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

    }
    //! Empty stub
    ~DemoSimple() 
    {
      ofs.close();
    }

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

  if( id >= 0 )
    event = mapEvent[id];
  event_ = event;

  if( id >= 0 )
  {
     if( curState_[cur_state_] == Success )
     {
        std_msgs::String msg;
        msg.data = "green_light";
        pub_states_.publish(msg);
     } 
     else if( curState_[cur_state_] == Fail )
     {
        std_msgs::String msg;
        msg.data = "red_light";
        pub_states_.publish(msg);
     }      
  }

  return event;
}

void DemoSimple::perform_event(Event event)
{  
  int count;
  std_msgs::String msg;
  stringstream ss;

  switch(event_)
  {
    case EstimatePose:
      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

      ROS_DEBUG("Event: pose estimation");
      doPoseEstimation();
      break;
    case PlanGrasp:
      cur_state_ = PlanGrasp_State;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);

      ROS_DEBUG("Event: plan grasps");
      planGrasps(arm_);
      break;
    case PlanTrajectory:

  
    if( curState_[PreGraspTraj_State] == Idle )
    {
      cur_state_ = PreGraspTraj_State;
      stringstream ss;
      std_msgs::String msg;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);
      usleep(1000*1000);
    }
    else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] == Idle ) )
    {
      cur_state_ = GraspTraj_State;
      stringstream ss;
      std_msgs::String msg;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);
    }
    else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] != Idle ) )
    {
      cur_state_ = PostGraspTraj_State;
      stringstream ss;
      std_msgs::String msg;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);
      usleep(1000*1000);   
    }

      ROS_DEBUG("Event: plan trajectory");
      plan_trajectory();
      break;  
    case Retreat: 
      cur_state_ = Start_State;

      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);

      ROS_DEBUG("Event: retreat");
      
    /*  msg.data = "clear_all";
      pub_clear_.publish(msg);  */

      // ** detach object for safety ** //
      /*reader_srv.request.object_id = object_id_;
      reader_srv.request.retreat = false;
      reader_srv.request.arm_name = arm_;
      usleep(1000*1000);
      reconstruct_scene();*/

      count = 1;
      goToStartPos();
      while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) ){
	      count++;
        goToStartPos();
      }     
      break;
    case Start:
      pose_trial_ = 0;
      msg.data = "clear_all";
      pub_clear_.publish(msg);
      usleep(1000*1000);

      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

      cur_state_ = Start_State;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);

      ROS_DEBUG("Event: start");
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
      pose_trial_ = 0;
      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

      cur_state_ = Start_State;
      cur_state_ = Start_State;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg); 
      usleep(1000*1000);

      ROS_DEBUG("Event: restart");
      
      msg.data = "clear_all";
      pub_clear_.publish(msg);

      ROS_INFO("go to start pose with grasp joint");
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
	      ROS_INFO("go to place position!!");
        do{
          restart(true,false);
          count++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );
      }
      /*reader_srv.request.object_id = 0;
      reader_srv.request.retreat = false;
      reader_srv.request.arm_name = arm_;
      reconstruct_scene();   
      usleep(1000*1000);*/

      if( curState_[Start_State] == Success )
      {
        ROS_INFO("to drop object!!!");
        //restart(true,true);
        restart(true,true,false);    
      }
      count = 1;
      if( curState_[Start_State] == Success )
      {
        ROS_INFO("come back to start position after droping!!");
        do
        {
          goToStartPos();
          count++;
        }while( ( count < max_trial_start_ ) && ( curState_[Start_State] == Fail ) );
      }                  
      break;      
    case Stop: 
      msg.data = "clear_screen";
      pub_states_.publish(msg);
      usleep(1000*1000);

      msg.data = "state_stop";
      pub_states_.publish(msg); 
      usleep(1000*1000);

      msg.data = "clear_all";
      pub_clear_.publish(msg);
      usleep(1000*1000);

      ROS_DEBUG("Event: stop");
      break;
    case PickObject:
      cur_state_ = PickObject_State;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);

      ROS_DEBUG("Event: next object");
      goToNextObject();
      break;   
    case PickGrasp:
      cur_state_ = PickGrasp_State;
      ss << "state_" << cur_state_;
      msg.data = ss.str();
      pub_states_.publish(msg);  
      usleep(1000*1000);

      ROS_DEBUG("Event: next grasp");
      goToNextGrasp();
      break;
    default:
      ROS_DEBUG("Event: stop");
      event_ = Stop;
      break;
  }
}

bool DemoSimple::goToStartPos(bool user_debug)
{
  curState_[Start_State] = Fail;
  // ** to test wo execution //
  //curState_[Start_State] = Success;

  if( arm_ == "right" ){
    robotpose = robotpose_right_;
    robot_start_joints_ = robot_start_joints_right;
  }
  else if( arm_ == "left" )
  {
    robotpose = robotpose_left_;
    robot_start_joints_ = robot_start_joints_left;    
  }

  bool result = false;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1); 
  
  //std::cout << "Initial position: " << robotpose << std::endl;
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
  
  if( !trajectory_planner_client_.call(trajectory_planning_srv) )
  {
    ROS_ERROR("trajectory planner service call failed.");
    ofs << ros::Time::now() << " trajectory planner service call failed." << endl;     
  }
  else	
  {
    
    //ROS_INFO("trajectory planner call succeeded");
    ofs << ros::Time::now() << " trajectory planner service call succeeded for joint: " <<           
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;    
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("No. of found trajectories: %d",(int)my_calculated_trajectory.size());
    
    if( my_calculated_trajectory.size() > 0 )
    {
    
      //ROS_INFO("Executing Trajectory");
    
      //User input
      string answer;
      if( user_debug )
      {
        std::cout << "Check if trajectory is ok (y/n)" << std::endl;
        std::cin >> answer;        
      }
    
      if ((answer == "y") || ( !user_debug) )
      {    
        //Arm movement
        //std::cout << "Executing arm trajectory" << std::endl;
        //std::cout << "#of found trajectories: "  << my_calculated_trajectory.size() << std::endl;
        my_calculated_trajectory[0].trajectory_id=0;
        trajectory_execution_srv.request.trajectory = my_calculated_trajectory[0]; 
        if( !trajectory_execution_client_.call(trajectory_execution_srv) )
        {
	        ROS_ERROR("trajectory execution service call failed.");
          ofs << ros::Time::now() << " trajectory execution service call failed." << endl;
        }
        else
        {
	        ROS_INFO("Succesfully executed the trajectory");
          ofs << ros::Time::now() << " trajectory execution call succeeded for joint: " <<           
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;
	        curState_[Start_State] = Success;
	        result = true;
        }
        
      }
      else if ((answer != "y") && ( user_debug) )
      {
        ROS_WARN("Trajectory discarded by the user - restart");
        ofs << ros::Time::now() << " Trajectory not valid by user -restart: joint: " << 
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;        
      }
    }
    else
    {
      ROS_ERROR("No trajectory found");
      ofs << ros::Time::now() << " no trajectory found for joint: " << 
          robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << robot_start_joints_[3] <<
          " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;
    }
       
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

bool DemoSimple::restart(bool place,bool drop,bool user_debug)
{
  curState_[Start_State] = Fail;
  // ** to test wo execution **//
 // curState_[Start_State] = Success;

  if( arm_ == "right" ){
    robotpose = robotpose_right_;
    robot_start_joints_ = robot_start_joints_right;
    robot_place_joints_ = robot_place_joints_right;
  }
  else if( arm_ == "left" )
  {
    robotpose = robotpose_left_;
    robot_start_joints_ = robot_start_joints_left;  
    robot_place_joints_ = robot_place_joints_left;  
  }

  bool result = false;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1); 
  
  //std::cout << "Initial position: " << robotpose << std::endl;
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

  size_t N = my_calculated_grasp[grasp_id_].grasp_trajectory.size() - 1;
  if( !place )
  {
    if( arm_ == "right" )
    {
      trajectory_planning_srv.request.eddie_goal_state.armRight.joints = robot_start_joints_;
      trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[N].joints;
    }
    else if( arm_ == "left" )
    {
      trajectory_planning_srv.request.eddie_goal_state.armLeft.joints = robot_start_joints_;
      trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[N].joints;
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
        trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[N].joints;
      else if( arm_ == "left" )
        trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[N].joints;      
    }
  }
  
  
  if( !trajectory_planner_client_.call(trajectory_planning_srv) )
  {
    ROS_ERROR("trajectory planner service call failed.");
    ofs << ros::Time::now() << " trajectory planner service call failed." << endl;
  }
  else	
  {
    
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());   
    if( !place )
    {
      ofs << ros::Time::now() << " trajectory planner service call succeeded for joint: " <<           
        robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
        robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;            
    }
    else
    {
      ofs << ros::Time::now() << " trajectory planner service call succeeded for joint: " <<           
        robot_place_joints_[0]  << " " << robot_place_joints_[1] << " " << robot_place_joints_[2]  << " " << 
        robot_place_joints_[3] << " " << robot_place_joints_[4]  << " " << robot_place_joints_[5] << " " << robot_place_joints_[6] << endl;            
    }

    if( my_calculated_trajectory.size() > 0 ) 
    {
      ROS_INFO("Executing Trajectory");
    
      //User input
      string answer;
      if( user_debug )
      {
        std::cout << "Check if trajectory is ok (y/n)" << std::endl;
        std::cin >> answer;
      }
    
      if ( (answer == "y") || ( !user_debug ) )
      {    
        //Arm movement
        //std::cout << "Executing arm trajectory" << std::endl;
        //std::cout << "#of found trajectories: "  << my_calculated_trajectory.size() << std::endl;
        my_calculated_trajectory[0].trajectory_id=0;
        trajectory_execution_srv.request.trajectory = my_calculated_trajectory[0]; 
        if( !trajectory_execution_client_.call(trajectory_execution_srv) )
        {
	        ROS_ERROR("trajectory execution service call failed.");
          ofs << ros::Time::now() << " trajectory execution service call failed." << endl;
        }
        else
        {
	        ROS_INFO("trajectory execution call succeeded");
          if( !place )
          {
            ofs << ros::Time::now() << " trajectory execution call succeeded for joint: " <<           
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;            
          }
          else
          {
            ofs << ros::Time::now() << " trajectory execution call succeeded for joint: " <<           
              robot_place_joints_[0]  << " " << robot_place_joints_[1] << " " << robot_place_joints_[2]  << " " << 
              robot_place_joints_[3] << " " << robot_place_joints_[4]  << " " << robot_place_joints_[5] << " " << robot_place_joints_[6] << endl;            
          }
	        curState_[Start_State] = Success;
	        result = true;
        }
        
      }
      else if( ( answer != "y" ) && ( user_debug ) )
      {
        ROS_WARN("Trajectory not valid - restart");
        if( !place )
        {
            ofs << ros::Time::now() << " Trajectory not valid by user- restart for joint: " <<           
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;            
        }
        else
        {
            ofs << ros::Time::now() << " Trajectory not valid by user- restart for joint: " <<           
              robot_place_joints_[0]  << " " << robot_place_joints_[1] << " " << robot_place_joints_[2]  << " " << 
              robot_place_joints_[3] << " " << robot_place_joints_[4]  << " " << robot_place_joints_[5] << " " << robot_place_joints_[6] << endl;            
        }        
      }
    }
    else
    {
      ROS_ERROR("no trajectory found");
      if( !place )
      {
            ofs << ros::Time::now() << " no trajectory found for joint: " <<           
              robot_start_joints_[0]  << " " << robot_start_joints_[1] << " " << robot_start_joints_[2]  << " " << 
              robot_start_joints_[3] << " " << robot_start_joints_[4]  << " " << robot_start_joints_[5] << " " << robot_start_joints_[6] << endl;            
      }
      else
      {
            ofs << ros::Time::now() << " no trajectory found for joint: " <<           
              robot_place_joints_[0]  << " " << robot_place_joints_[1] << " " << robot_place_joints_[2]  << " " << 
              robot_place_joints_[3] << " " << robot_place_joints_[4]  << " " << robot_place_joints_[5] << " " << robot_place_joints_[6] << endl;            
      }          
    }
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
  cur_state_ = PoseEstimate_State;
  stringstream ss;
  std_msgs::String msg;
  ss << "state_" << cur_state_;
  msg.data = ss.str();
  pub_states_.publish(msg);

  curState_[PoseEstimate_State] = Fail; 
  curState_[PickObject_State] = Fail;

  if( !pose_client_.call(estimation_srv))
  {
   ROS_ERROR("pose estimation service failed. wait...");
   ofs << ros::Time::now() << " pose estimation service failed. wait..." << endl;
   ros::Duration(0.5).sleep();
  }
  my_detected_objects = estimation_srv.response.detected_objects;

  ROS_INFO("No. of found objects: %d",(int)my_detected_objects.size());

  if( my_detected_objects.size() > 0 ){
   curState_[PickObject_State] = Success;
   object_count_ = vector<int>(my_detected_objects.size(),0);
   object_id_ = 0;
   ROS_INFO_STREAM("I am going for object: " << my_detected_objects[object_id_].name.data << endl);
   ss << "<font size=" << "20"<<  " color=red>" <<"No. of found objects: " << my_detected_objects.size() << "</font>" << "<br>";
   ss << "<font size=" << "20"<<  " color=red>" <<"I am going for object: " << 
         my_detected_objects[object_id_].name.data << "</font>" << "<br>";
   msg.data = ss.str();
   pub_states_.publish(msg); 

  }
  objectsNum_ = my_detected_objects.size();
  if( ( estimation_srv.response.result == estimation_srv.response.SUCCESS ) && ( my_detected_objects.size() > 0 ) )
  // if( estimation_srv.response.result == estimation_srv.response.SUCCESS )
  {
    curState_[PoseEstimate_State] = Success;  
    ofs << ros::Time::now() << " pose estimation succeded" << endl;
  }
  else
    ofs << ros::Time::now() << " pose estimation failed" << endl;
  
   if( ( available_arm_ == "both" ) && ( curState_[PoseEstimate_State] == Success ) && 
       (select_arm_factor.find("position_based") != string::npos ) )
        select_arm_object(); 

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
  {
    if( ( my_detected_objects.size() > 0 ) && (pose_trial_ < max_pose_estimation_trial_ ) )   
    {
      cout << "All grasps for all objects failed, let's try again." << endl;
      pose_trial_ ++;
      curState_[0] = Success;
      for( int i = 1; i < StatesNum; i++ )
        curState_[i] = Idle;      
     }    
     return;
  }
 /* else if( ( object_count_[object_id_] >= max_trial_ ) &&  ( (object_id_ + 1)  >= objectsNum_ ) ) 
    return;*/
  object_count_[object_id_] ++;
  ROS_INFO_STREAM("In next object: I am going for object: " << my_detected_objects[object_id_].name);

  stringstream ss;
  std_msgs::String msg;

  msg.data = "clear_screen";
  pub_states_.publish(msg);
  usleep(1000*1000);

  ss << "<font size=" << "20"<<  " color=red>" <<"No. of found objects: " << my_detected_objects.size() << "</font>" << "<br>"; 
  ss << "<font size=" << "20"<<  " color=red>" <<"I am going for object: " << 
         my_detected_objects[object_id_].name.data << "</font>" << "<br>";
  msg.data = ss.str();
  pub_states_.publish(msg); 

  curState_[PickObject_State] = Success;
 // curState_[PoseEstimate_State] = Success; 
  for( int i = (PickObject_State + 1 ); i < StatesNum; i++ )
    curState_[i] = Idle;

  if(( available_arm_ == "both" ) && (select_arm_factor.find("position_based") != string::npos ))
     select_arm_object();
}

void DemoSimple::select_arm()
{
  geometry_msgs::Pose obj_pose = my_calculated_grasp[grasp_id_].grasp_trajectory[0].wrist_pose.pose;

  Eigen::Quaternionf q_obj(obj_pose.orientation.w,obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z);
  Eigen::Matrix3f m;
  m = q_obj.toRotationMatrix();
  // the projection of the z-axis of the grasp pose on the y-axis of the world boils down to check 
  // the y-component of the z-axis of the grasp pose
  // if it is positive, it means we better try with the right arm, and negative with the left arm
  // the y-component is m(1,2)

  // the distance is self-explanatory, the arm is chose accoding to the euclidean distance
  double dist_right = sqrt(pow((obj_pose.position.x - robotpose_right_.position.x),2)+
                      pow((obj_pose.position.y - robotpose_right_.position.y),2)+
                      pow((obj_pose.position.z - robotpose_right_.position.z),2));


  double dist_left = sqrt(pow((obj_pose.position.x - robotpose_left_.position.x),2)+
                      pow((obj_pose.position.y - robotpose_left_.position.y),2)+
                      pow((obj_pose.position.z - robotpose_left_.position.z),2));

  // so in the end, we have a weighted sum to select the arm giving slightly more weight to the orientation
  double w_orien = 0.6;
  double w_trans = 1 - w_orien;
  double sum = w_orien*(m(1,2)) + w_trans*(dist_left - dist_right);

  if( sum < 0 )
    arm_ = "left";
  else 
    arm_ = "right"; 
  ROS_INFO("Using arm: %s", arm_.c_str() ); 
}

void DemoSimple::select_arm_object()
{
  geometry_msgs::Pose obj_pose = my_detected_objects[object_id_].pose;

  // the distance is self-explanatory, the arm is chose accoding to the euclidean distance
  double dist_right = sqrt(pow((obj_pose.position.x - robotpose_right_.position.x),2)+
                      pow((obj_pose.position.y - robotpose_right_.position.y),2)+
                      pow((obj_pose.position.z - robotpose_right_.position.z),2));


  double dist_left = sqrt(pow((obj_pose.position.x - robotpose_left_.position.x),2)+
                      pow((obj_pose.position.y - robotpose_left_.position.y),2)+
                      pow((obj_pose.position.z - robotpose_left_.position.z),2));

  if( dist_left < dist_right )
    arm_ = "left";
  else 
    arm_ = "right"; 
  ROS_INFO_STREAM("Using arm" << arm_ << " considering object position!" ); 
}

void DemoSimple::reconstruct_scene()
{ 
   if( !reader_client_.call(reader_srv) ) 
   {
     ROS_ERROR("object reader service failed. wait...");
     ros::Duration(0.5).sleep();     
   }
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
  if( !grasp_client_.call(grasp_planning_srv) )
  {
    ROS_ERROR("grasp_planner service call failed. wait...");
    ros::Duration(0.5).sleep();
  }
  if( ( grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS) && ( grasp_planning_srv.response.grasp_list.size() > 0 ) )
  {
    success = true;
    ROS_INFO("Grasp Planning OK - assigning last wrist pose for arm trajectory planning");
    ofs << ros::Time::now() << " grasp_planner succeed" << endl;
    
    //Removing all but the best grasp and only keeping last trajectory
    my_calculated_grasp.clear();
   // std::cout << "grasp planner response size: " << grasp_planning_srv.response.grasp_list.size() << std::endl;
    for( size_t i = 0;((i < grasp_planning_srv.response.grasp_list.size() )&& ( i < 5 ) ); i++ )
      my_calculated_grasp.push_back(grasp_planning_srv.response.grasp_list[i]);    
   }
   
   else
   {
     ROS_ERROR("Grasp_planner did not succeed.");
     ofs << ros::Time::now() << " grasp_planner did not succeed." << endl;
     return success;
   }
   
   if( grasp_ordering_ )
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

  if( ( available_arm_ == "both" ) && (select_arm_factor.find("grasp_based") != string::npos ) )
      select_arm(); 

  return success;
}

void DemoSimple::order_grasp()
{
  if( arm_ == "right" )
    robotpose = robotpose_right_;
  else if( arm_ == "left" )
    robotpose = robotpose_left_;   

   ROS_INFO("Ordering grasp list");
   std::vector<definitions::Grasp> my_calculated_grasp_tmp;
   //double threshold = 0.2;
   double threshold = 0.15;
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
     
   ROS_INFO_STREAM("No. of Grasps rejected: " << my_calculated_grasp.size() - my_calculated_grasp_tmp.size());
   if(my_calculated_grasp.size() - my_calculated_grasp_tmp.size() > 0)
   {
      std_msgs::String msg;
      stringstream ss; 
      ss << "<font size=" << "20" << " color=red>" << "No. of Grasps rejected: " << my_calculated_grasp.size() - my_calculated_grasp_tmp.size() 
         << "</font>" << "<br>";
      msg.data = ss.str();
      pub_states_.publish(msg);
   }

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
}

void DemoSimple::goToNextGrasp()
{
  curState_[PickGrasp_State] = Fail;
  // curState_[PlanGrasp_State] = Fail;
  if( (grasp_id_ + 1) < my_calculated_grasp.size() )
  {
    grasp_id_ ++;
    curState_[PickGrasp_State] = Success;  
    curState_[PreGraspTraj_State] = Idle; 

    // curState_[PlanGrasp_State] = Success;

    if( ( available_arm_ == "both" ) && (select_arm_factor.find("grasp_based") != string::npos ) )
      select_arm(); 
  }
}

bool DemoSimple::plan_trajectory()
{
  bool result = false;
  if( curState_[PreGraspTraj_State] == Idle )
  {
    cur_state_ = PreGraspTraj_State;

    ROS_INFO_STREAM("Plan for pre-grasp id: " << grasp_id_);
    curState_[PreGraspTraj_State] = Fail;

    stringstream ss;
    std_msgs::String msg;

    msg.data = "clear_screen";
    pub_states_.publish(msg);
    usleep(1000*1000);

    ss << "<font size=" << "20"<<  " color=red>" <<"No. of found objects: " << my_detected_objects.size() << "</font>" << "<br>"; 
    ss << "<font size=" << "20"<<  " color=red>" <<"I am going for object: " << 
         my_detected_objects[object_id_].name.data << "</font>" << "<br>";

    ss << "<font size=" << "20" << " color=red>" << "Plan for grasp: " << grasp_id_ << 
        " from "<< my_calculated_grasp.size() << "</font>" << "<br>";
    msg.data = ss.str();
    pub_states_.publish(msg);

    // for testing wo execution 
   // curState_[PreGraspTraj_State] = Success;

    if( arm_ == "right" )
    {
      trajectory_planning_srv.request.eddie_goal_state.handRight.wrist_pose = my_calculated_grasp[grasp_id_].grasp_trajectory[0].wrist_pose;
      trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;
    }
    else if( arm_ == "left" )
    {
      trajectory_planning_srv.request.eddie_goal_state.handLeft.wrist_pose = my_calculated_grasp[grasp_id_].grasp_trajectory[0].wrist_pose;
      trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id_].grasp_trajectory[0].joints;    
    }
    executeMovement(true,false,grasp_id_);
    if( grasp_success_ )
      curState_[PreGraspTraj_State] = Success;
  }
  else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] == Idle ) )
  {
    cur_state_ = GraspTraj_State;

    ROS_INFO("Plan for the grasp");
    curState_[GraspTraj_State] = Fail;
    // for testing wo execution
    //curState_[GraspTraj_State] = Success;

    
    if( executeMovement(false,false,grasp_id_,1) )
    {
      //executeMovement(false,grasp_id_);
      //executeMovement(false,false,grasp_id_,2,false);
      //if( grasp_success_ )
        curState_[GraspTraj_State] = Success;
    }

  }
  else if( ( curState_[PreGraspTraj_State] == Success ) && ( curState_[GraspTraj_State] != Idle ) )
  {
    cur_state_ = PostGraspTraj_State;

    ROS_INFO("Plan for the post-grasp");
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

  size_t N = my_calculated_grasp[grasp_id_].grasp_trajectory.size()-1;
  
  if( arm_ == "right" )
  {
    trajectory_planning_srv.request.eddie_goal_state.handRight.wrist_pose = my_calculated_grasp[grasp_id].grasp_trajectory[N].wrist_pose;
    trajectory_planning_srv.request.eddie_goal_state.handRight.wrist_pose.pose.position.z += offset_z;
    trajectory_planning_srv.request.eddie_goal_state.handRight.joints = my_calculated_grasp[grasp_id].grasp_trajectory[N].joints;
  }
  else if( arm_ == "left" )
  {
    trajectory_planning_srv.request.eddie_goal_state.handLeft.wrist_pose = my_calculated_grasp[grasp_id].grasp_trajectory[N].wrist_pose;
    trajectory_planning_srv.request.eddie_goal_state.handLeft.wrist_pose.pose.position.z += offset_z;
    trajectory_planning_srv.request.eddie_goal_state.handLeft.joints = my_calculated_grasp[grasp_id].grasp_trajectory[N].joints;    
  }

  success = executeMovement(false,true,grasp_id);
  
  return success;
}

bool DemoSimple::executeMovement(bool pre_grasp, bool post_grasp, int &grasp_id, int traj_id, bool user_debug)
{
  bool succeed = false;
  grasp_success_ = false;

  if( grasp_id >= my_calculated_grasp.size() )   
  {
    return succeed;
  }
  if( !pre_grasp && !post_grasp) 
  {    
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = object_id_;
    reader_srv.request.retreat = post_grasp;
    reader_srv.request.arm_name = arm_;
    reconstruct_scene();
    usleep(1000*1000);
    // my_calculated_grasp[grasp_id].grasp_trajectory[0] = my_calculated_grasp[grasp_id].grasp_trajectory[traj_id];
    trajectory_planning_srv.request.type = trajectory_planning_srv.request.PICK;
  }
  else
  {    
    pub_cur_grasp_.publish(my_calculated_grasp[grasp_id]);
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.retreat = post_grasp;
    if (post_grasp)
    {
      reader_srv.request.object_id = object_id_;
    }
    else
    {
      reader_srv.request.object_id = -1;
    }
    reader_srv.request.arm_name = arm_;
    reconstruct_scene();
    usleep(1000*1000); 
    trajectory_planning_srv.request.type = trajectory_planning_srv.request.MOVE_TO_CART_GOAL;
  }
  //std::cout << "grasp id is: " << grasp_id << " : " << "grasp size is: " << my_calculated_grasp.size() << std::endl;
  std::vector<definitions::Grasp> my_calculated_grasp_cur;
  my_calculated_grasp_cur.push_back(my_calculated_grasp[grasp_id]);

 // std::cout << "my_calculated_grasp[grasp_id]" << my_calculated_grasp[grasp_id] << std::endl;

  //trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
  trajectory_planning_srv.request.arm = arm_;
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp_cur;
  trajectory_planning_srv.request.object_list = estimation_srv.response.detected_objects;
  trajectory_planning_srv.request.object_id = 0;

  //trajectory_planning_srv.request.object_id = grasp_id;
  
  if( !trajectory_planner_client_.call(trajectory_planning_srv) )
  {
    ROS_ERROR("trajectory planner service call failed.");
    ofs << ros::Time::now() << " trajectory planner service call failed." << endl;
    return false;
  }
  else	
  {
    //ROS_INFO("trajectory planner call succeeded");
    ofs << ros::Time::now() << " trajectory planner call succeeded for cart position: " << 
         my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
    
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    
    
    
    int nr_found_trajectories = my_calculated_trajectory.size();
    
    if(nr_found_trajectories > 0) 
    {
      ROS_INFO("No. of found trajectories: %d",(int)my_calculated_trajectory.size());
      ROS_INFO("Executing Pick and place");
    
       //User input
      string answer;
      if( user_debug )
      {
        std::cout << "Check if trajectory is ok (y/n)" << std::endl;
        std::cin >> answer;
      }
    
      if( (answer == "y") || ( !user_debug ) )
      {    
      
      //Arm movement
        std::cout << "Executing arm trajectory" << std::endl;
        my_calculated_trajectory[0].trajectory_id=0;
      
       // trajectory_execution_srv.request.trajectory = my_calculated_trajectory; //execute 0 trajectory id
        trajectory_execution_srv.request.trajectory = my_calculated_trajectory[0];
	
        if( !trajectory_execution_client_.call(trajectory_execution_srv) )
        {
	        ROS_ERROR("trajectory planner service call failed.");
          ofs << ros::Time::now() << " trajectory planner service call failed. for cart position: " << 
              my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
        }
        else
        {
	        grasp_success_ = true;
	        ROS_INFO("trajectory execution call succeeded");
          ofs << ros::Time::now() << " trajectory execution call succeeded for cart position: " << 
               my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
	        succeed = true;
	        return true;
        }
      }
      else if( (answer == "n") && ( user_debug ) )
      {
        ROS_WARN("Trajectory not valid - restart");
        ofs << ros::Time::now() << " trajectory not valid by user-restart: for cart position: "
            << my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
    
        /*bool grasp_success = planGrasps(arm_);
    
        if( !grasp_success )
          return false;*/
    
        executeMovement(pre_grasp,post_grasp,grasp_id);
    
      }
      else if( ( answer == "stop" ) && ( user_debug ) )
      {
        ofs << ros::Time::now() << " no trajectory for cart position: " << 
            my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
        return false;
      }
    }
    else if( ( (grasp_id+1) < (my_calculated_grasp.size()) ) && (pre_grasp) )
    { 
      
      grasp_id++;
      ROS_INFO_STREAM("Could not find a plan, so try another pre-grasp id: " << grasp_id);
      ofs << ros::Time::now() << " plan another grasp id: " << grasp_id << endl;
      executeMovement(pre_grasp,post_grasp,grasp_id);
    }
    else if( ( (grasp_id+1) >= (my_calculated_grasp.size()) ) && (pre_grasp) )
    {
      ROS_WARN_STREAM("No plan found for pre-grasp id: " << grasp_id);
      ofs << ros::Time::now() << " No plan found for pre-grasp: " << 
         my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
      return false;
    }
    else if( !pre_grasp )
    {
      ROS_ERROR("No plan found for grasp");
      ofs << ros::Time::now() << " No plan found for cart position: " << 
         my_calculated_grasp[grasp_id].grasp_trajectory[0].wrist_pose << endl;
      return false;
    }
  }
  
  //std::cout << "Everything succeded inside execute movement " << std::endl;
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

  while( demo.event_ != Stop )
  {
    Event event = demo.evaluate_cur_state();
    demo.perform_event(event);
  }
  
  return 0;
}
