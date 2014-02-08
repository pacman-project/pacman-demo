// system headers
#include <exception>
#include <boost/thread.hpp>

// ros headers
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/JointState.h>

// local headers
#include <definitions/TrajectoryExecution.h>
#include <pacman/Bham/Control/Control.h>
#include <pacman/PaCMan/Defs.h>

using namespace pacman;

namespace golem_control_bham{

  class GolemController{
    private:
      // the node handle
      ros::NodeHandle nh_;

      // node handle in the private namespace
      ros::NodeHandle priv_nh_;

      // frequency for refresh, wait for messages, callbacks
      double frequency_;
      
      // the controller object
      BhamControl::Ptr controller_;

      // the robot type and data structures fro state and comman
      RobotUIBK uibk_robot_;
      RobotUIBK::State uibk_robot_state_;
      RobotUIBK::Command::Seq uibk_robot_command_;

      // variables where conversions are saved to
      //BhamControl trajectory_;
      //BhamControl test_trajectory_;

      // configuration files
      std::string config_file_;

      // test files
      //std::string test_trajectory_file_;

      // ros variables where conversions are saved to
      sensor_msgs::JointState joint_states_;
      //actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
      std::string action_name_;
      std::string arm_name_;
      std::string hand_name_;

      // service servers offered by this node
      ros::ServiceServer srv_trajectory_execution_;
      //ros::ServiceServer srv_get_controller_time_;
      ros::ServiceServer srv_test_controller_;

      // publishers of this node
      ros::Publisher pub_robot_state_;

      // conversion functions
      // void convertTrajFromJointTrajectoryMsg(moveit_msgs::RobotTrajectory &trajectory);
      // void convertTrajFromDefinitionsMsg(const definitions::Trajectory &trajectory)
      void convertStateToJointStateMsg(const RobotUIBK::State &state);

    public:
    
      // loop function to update the robot state
      void publishRobotState();

      // service function to request trajectory execution
      //bool executeTrajectoryFromMoveItGUI(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res);
      //bool executeTrajectoryFromDemoNode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res);
      bool executeTrajectory();


      // helper function to get the time stamp of the controllers
      // void getTimeStamp(time);

      // service function to test the controller
      bool testController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      // constructor
      GolemController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        // load configuration file from the launch file
        priv_nh_.param<std::string>("config_file", config_file_, "");

        // define the names passed in the urdf files corresponding to the current move group
        priv_nh_.param<std::string>("arm_name", arm_name_, "right");
        priv_nh_.param<std::string>("hand_name", hand_name_, "right_sdh");
        
        // // load test files from the launch file
        // priv_nh_.param<std::string>("test_trajectory_file", test_trajectory_file_, "");

        // create the controller object using the config file
        controller_ = BhamControl::create(RobotType::ROBOT_UIBK, config_file_);

        // load the test trajectory file (if function exist?)
        // load(test_trajectory_file_, test_trajectory_);

        // advertise the node services
        //srv_trajectory_execution_ = nh_.advertiseService(nh_.resolveName("/trajectory_execution_srv"),&GolemController::executeTrajectory, this);
        srv_test_controller_ = nh_.advertiseService(nh_.resolveName("/test_controller_srv"),&GolemController::testController, this);

        // advertise the node topics
        pub_robot_state_ = nh_.advertise<sensor_msgs::JointState>(nh_.resolveName("/golem/joint_states"), 10);

        // initialize the joint state topic
        joint_states_.name.resize(RobotUIBK::Config::JOINTS);
        joint_states_.position.resize(RobotUIBK::Config::JOINTS);
        joint_states_.velocity.resize(RobotUIBK::Config::JOINTS);
        joint_states_.effort.resize(RobotUIBK::Config::JOINTS);

        // initialze the command 
        uibk_robot_command_.resize(1);


      }

      //! Empty stub
      ~GolemController() {}
  };
  
// void GolemController::convertTrajFromMoveIt()
// {
//   return;
// }

// void GolemController::convertTrajFromDefinitions()
// {
//   return;
// }

void GolemController::convertStateToJointStateMsg(const RobotUIBK::State &state) 
{

  joint_states_.header.stamp = ros::Time::now();

  // the joint mapping needs to be hardcoded to match Golem.xml and Ros.urdf structures
  // and let the party begin... first the arm:
  joint_states_.name[0] = arm_name_ + "_arm_0_joint";
  joint_states_.position[0] = state.pos.arm.c[0];

  joint_states_.name[1] = arm_name_ + "_arm_1_joint";
  joint_states_.position[1] = state.pos.arm.c[1];

  joint_states_.name[2] = arm_name_ + "_arm_2_joint"; 
  joint_states_.position[2] = state.pos.arm.c[2]; 

  joint_states_.name[3] = arm_name_ + "_arm_3_joint";
  joint_states_.position[3] = state.pos.arm.c[3];

  joint_states_.name[4] = arm_name_ + "_arm_4_joint";
  joint_states_.position[4] = state.pos.arm.c[4];

  joint_states_.name[5] = arm_name_ + "_arm_5_joint";
  joint_states_.position[5] = state.pos.arm.c[5];

  joint_states_.name[6] = arm_name_ + "_arm_6_joint";
  joint_states_.position[6] = state.pos.arm.c[6];

  // and continue with the hand:
  joint_states_.name[7] = hand_name_ + "_knuckle_joint";
  joint_states_.position[7] = state.pos.hand.rotation;

  joint_states_.name[8] = hand_name_ + "_finger_12_joint";
  joint_states_.position[8] = state.pos.hand.left[0];

  joint_states_.name[9] = hand_name_ + "_finger_13_joint";
  joint_states_.position[9] = state.pos.hand.left[1];

  joint_states_.name[10] = hand_name_ + "_finger_22_joint";
  joint_states_.position[10] = state.pos.hand.right[0];

  joint_states_.name[11] = hand_name_ + "_finger_23_joint";
  joint_states_.position[11] = state.pos.hand.right[1];

  joint_states_.name[12] = hand_name_ + "_thumb_2_joint";
  joint_states_.position[12] = state.pos.hand.middle[0];

  joint_states_.name[13] = hand_name_ + "_thumb_3_joint";
  joint_states_.position[13] = state.pos.hand.middle[1];

  return;
}


void GolemController::publishRobotState()
{
  try 
  {  
    controller_->lookupState(controller_->time(), &uibk_robot_state_);
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to read the robot state: %s\n", ex.what());
  }

  // convert from pacman to ros joint states
  convertStateToJointStateMsg(uibk_robot_state_);

  // publish the joint states, this functionality could be ported to another node to increase speed and versatility.
  pub_robot_state_.publish(joint_states_);

  return;
}

// bool GolemController::executeTrajectoryFromMoveItGUI(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res)
// {
//   uibk_robot_command_.clear();
//   convertTrajFromMoveIt(trajectory, uibk_robot_command_);
//   executeTrajectory(uibk_robot_command_);

//   return true;
// }

// bool GolemController::executeTrajectoryFromDemoNode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res)
// {
//   uibk_robot_command_.clear();
//   convertTrajFromDefinitions(trajectory, uibk_robot_command_);
//   executeTrajectory(uibk_robot_command_);

//   return true;
// }

bool GolemController::executeTrajectory()
{
  try 
  {  
    controller_->send(uibk_robot_command_.data(), uibk_robot_command_.size());
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to execute the given trajectory: %s\n", ex.what());
    return false;
  }
  return true;
}

// this function is called to test that the controller client works well with some basic motions
bool GolemController::testController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  try {
    // Read current state
    RobotUIBK::State begin;
    controller_->lookupState(controller_->time(), &begin);

    // Prepare trajectory
    RobotUIBK::Command::Seq commands(9);
    // Initial pose
    commands[0].t = controller_->time();
    commands[0].pos = begin.pos;
    // test arm
    // Pose #1
    commands[1].t = commands[0].t + pacman::float_t(5.0);
    commands[1].pos = begin.pos;
    commands[1].pos.arm.c[0] += pacman::float_t(0.75);
    //commands[1].pos.hand.rotation += pacman::float_t(0.75);
    // Pose #2
    commands[2].t = commands[1].t + pacman::float_t(5.0);
    commands[2].pos = begin.pos;
    commands[2].pos.arm.c[1] -= pacman::float_t(0.75);
    // Pose #3
    commands[3].t = commands[2].t + pacman::float_t(5.0);
    commands[3].pos = begin.pos;
    commands[3].pos.arm.c[2] += pacman::float_t(0.75);
    // Pose #4
    commands[4].t = commands[3].t + pacman::float_t(5.0);
    commands[4].pos = begin.pos;
    commands[4].pos.arm.c[3] -= pacman::float_t(0.75);
    // Pose #5
    commands[5].t = commands[4].t + pacman::float_t(5.0);
    commands[5].pos = begin.pos;
    commands[5].pos.arm.c[4] += pacman::float_t(0.75);
    // Pose #6
    commands[6].t = commands[5].t + pacman::float_t(5.0);
    commands[6].pos = begin.pos;
    commands[6].pos.arm.c[5] -= pacman::float_t(0.75);
    // Pose #7
    commands[7].t = commands[6].t + pacman::float_t(5.0);
    commands[7].pos = begin.pos;
    commands[7].pos.arm.c[6] += pacman::float_t(0.75);
    // Back to the initial pose
    commands[8].t = commands[7].t + pacman::float_t(5.0);
    commands[8].pos = begin.pos;
    //commands[8].pos.arm.c[3] -= pacman::float_t(0.75);
    // send trajectory
    controller_->send(commands.data(), commands.size());

    // wait for completion
    //controller_->waitForTrajectoryEnd();
  }
  catch (const std::exception& ex) {
    printf("ControlTest exception: %s\n", ex.what());
    return 1;
  }

  return true;
}



} // end namespace golem_control_bham


// // update the states in a different thread such that it is not affected by blocking functions.
// void publishThread_fnc(golem_control_bham::GolemController &nn)
// {
//   while(ros::ok())
//   {
//     // publish joint states
//     nn.publishRobotState();
//   }
//   return;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "golem_control_bham_node");
  ros::NodeHandle nh;

  golem_control_bham::GolemController node(nh);

  ROS_INFO("Controller connected...");
  ROS_INFO("Ready to execute trajectories!");
  
  //boost::thread publish_thread( publishThread_fnc, node );
  //publish_thread.join();

  while(ros::ok())
  {
    // do something

    // publish joint states
    node.publishRobotState();

    // spin
    ros::spinOnce();
  }

  // // detach the thread in case is not ok
  // publish_thread.detach();

  return 0;
}
