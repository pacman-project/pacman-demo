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
#include "definitions/TrajectoryExecution.h"
#include <pacman/Bham/Control/Control.h>
#include <pacman/PaCMan/Defs.h>
#include <pacman/PaCMan/ROS.h>

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

      // roslaunch parameters
      // configuration file for golem
      std::string config_file_;
      std::string arm_name_;

      // ros variables where conversions are saved to
      sensor_msgs::JointState joint_states_;

      // service servers offered by this node
      ros::ServiceServer srv_trajectory_execution_;
      ros::ServiceServer srv_test_controller_;

      // publishers of this node
      ros::Publisher pub_robot_state_;

    public:
    
      // loop function to update the robot state
      void publishRobotState();

      // service function to request trajectory execution
      //bool executeTrajectoryFromMoveItGUI(moveit_msgs::RobotTrajectory::Request &req, moveit_msgs::RobotTrajectory::Response &res);
      bool executeTrajectoryFromCode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res);
      bool executeTrajectory(const RobotUIBK::Command::Seq &command);

      // service function to test the controller
      bool testController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      // constructor
      GolemController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        // load configuration file from the launch file
        priv_nh_.param<std::string>("config_file", config_file_, "");

        // define the names passed in the urdf files corresponding to the current move group
        priv_nh_.param<std::string>("arm_name", arm_name_, "left");

        // create the controller object using the config file
        controller_ = BhamControl::create(config_file_);

        // advertise the node services
        srv_trajectory_execution_ = nh_.advertiseService(nh_.resolveName("/trajectory_execution_srv"),&GolemController::executeTrajectoryFromCode, this);
        srv_test_controller_ = nh_.advertiseService(nh_.resolveName("/test_controller_srv"),&GolemController::testController, this);

        // advertise the node topics
        pub_robot_state_ = nh_.advertise<sensor_msgs::JointState>(nh_.resolveName("/golem/joint_states"), 10);

        // initialze the command 
        uibk_robot_command_.resize(1);

      }

      //! Empty stub
      ~GolemController() {}
  };

void GolemController::publishRobotState()
{
  // read the current data from the controller
  try 
  {  
    controller_->lookupState(controller_->time(), uibk_robot_state_);
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to read the robot state: %s\n", ex.what());
  }

  // convert from pacman to ros joint states 
  pacman::mapStates(uibk_robot_state_, arm_name_, joint_states_, ros::Time::now());

  // publish the joint states, this functionality could be ported to another node to increase speed and versatility.
  pub_robot_state_.publish(joint_states_);

  return;
}

bool GolemController::executeTrajectoryFromCode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res)
{
  ROS_INFO("Exectution service requested...");

  definitions::Trajectory trajectory = req.trajectory;

  // if (req.robot == req.UIBKRobot)
  // {
    //convert the trajectory to the command ToDo: double check that the trajectory has velocity and acceleration
    pacman::convert(trajectory, uibk_robot_command_, controller_->time());

    // excexute the trajectory
    if( executeTrajectory(uibk_robot_command_) )
    {
      ROS_INFO("Exectution finished cleanly...");
      res.result = res.SUCCESS;
    }
    else
    {
      ROS_ERROR("Trajectory could not be executed...");
      res.result = res.OTHER_ERROR;
    }
  // }
  // if (req.robot == req.RobotEddie)
  // {

  // }

  return true;
}

bool GolemController::executeTrajectory(const RobotUIBK::Command::Seq &command)
{
  try 
  {  
    controller_->send(command.data(), command.size());
    // wait for completion
    controller_->waitForTrajectoryEnd();
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to execute the given trajectory:  %s\n", ex.what());
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
    controller_->lookupState(controller_->time(), begin);

    RobotUIBK::Command init;

    init.hand.pos = begin.hand.pos;
    init.arm.pos = begin.arm.pos;

    // Prepare trajectory
    RobotUIBK::Command::Seq commands(12);
    // Initial pose
    commands[0] = init;
    
    commands[0].t = controller_->time() + pacman::float_t(1.0);

    // test arm
    // Pose #1
    commands[1] = init;
    commands[1].t = commands[0].t + pacman::float_t(5.0);
    commands[1].arm.pos.c[0] += pacman::float_t(0.1);
    // Go to home in the hand at pose 1
    commands[1].hand.pos.setToDefault();
    // Pose #2
    commands[2] = init;
    commands[2].t = commands[1].t + pacman::float_t(5.0);
    commands[2].arm.pos.c[1] -= pacman::float_t(0.1);
    // Pose #3
    commands[3] = init;
    commands[3].t = commands[2].t + pacman::float_t(5.0);
    commands[3].arm.pos.c[2] += pacman::float_t(0.1);
    // Pose #4
    commands[4] = init;
    commands[4].t = commands[3].t + pacman::float_t(5.0);
    commands[4].arm.pos.c[3] -= pacman::float_t(0.1);
    // Pose #5
    commands[5] = init;
    commands[5].t = commands[4].t + pacman::float_t(5.0);
    commands[5].arm.pos.c[4] += pacman::float_t(0.1);
    // Pose #6
    commands[6] = init;
    commands[6].t = commands[5].t + pacman::float_t(5.0);
    commands[6].arm.pos.c[5] -= pacman::float_t(0.1);
    // Pose #7
    commands[7] = init;
    commands[7].t = commands[6].t + pacman::float_t(5.0);
    commands[7].arm.pos.c[6] += pacman::float_t(0.1);
    // Back to the initial pose
    commands[8] = init;
    commands[8].t = commands[7].t + pacman::float_t(5.0);
    // Test the hand as well
    // Move knucle
    commands[9] = init;
    commands[9].t = commands[8].t + pacman::float_t(5.0);
    commands[9].hand.pos.rotation += pacman::float_t(0.25);
    // Open
    commands[10] = init;
    commands[10].t = commands[9].t + pacman::float_t(5.0);
    commands[10].hand.pos.middle[0] -= pacman::float_t(0.3);
    commands[10].hand.pos.middle[1] -= pacman::float_t(0.3);
    commands[10].hand.pos.left[0] -= pacman::float_t(0.3);
    commands[10].hand.pos.left[1] -= pacman::float_t(0.3);
    commands[10].hand.pos.right[0] -= pacman::float_t(0.3);
    commands[10].hand.pos.right[1] -= pacman::float_t(0.3);
    // Close to home position
    commands[11] = init;
    commands[11].t = commands[10].t + pacman::float_t(5.0);


    // execute trajectory
    executeTrajectory(commands);
  }
  catch (const std::exception& ex) {
    printf("ControlTest exception: %s\n", ex.what());
    return 1;
  }

  return true;
}


} // end namespace golem_control_bham


void publishThread_fnc(golem_control_bham::GolemController &nn)
{
  while(ros::ok())
  {
    // publish joint states
    nn.publishRobotState();
  }
  return;
}

void controlThread_fnc(golem_control_bham::GolemController &nn)
{
  ros::spin();
  return;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "golem_control_bham_node");
  ros::NodeHandle nh;

  golem_control_bham::GolemController node(nh);

  ROS_INFO("Controller connected...");
  ROS_INFO("Ready to execute trajectories!");

  // this has to be done separately because the execution control is blocking,
  // and we need to continuously pusblish the joint state to visualize whats going on
  // so instead of the typical while loop with ros::spin(), we need to split 
  // the control and the publishing into two threads.
  boost::thread publish_thread( publishThread_fnc, node );
  boost::thread control_thread( controlThread_fnc, node );
  publish_thread.join();
  control_thread.join();


  // detach the threads
  publish_thread.detach();
  control_thread.detach();

  return 0;
}
