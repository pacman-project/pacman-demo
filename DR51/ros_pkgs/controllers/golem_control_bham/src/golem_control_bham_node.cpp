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

      // the robot type and data structures for state and comman
      // RobotUIBK uibk_robot_;
      // RobotUIBK::State uibk_robot_state_;
      // RobotUIBK::Command::Seq uibk_robot_command_;

      // the robot type and data structures for state and comman
      RobotEddie robot_eddie_;
      RobotEddie::State robot_eddie_state_;
      RobotEddie::Command::Seq robot_eddie_command_;

      // roslaunch parameters
      // configuration file for golem
      std::string config_file_;
      std::string arm_name_;

      // ros variables where conversions are saved to
      sensor_msgs::JointState joint_states_;

      // service servers offered by this node
      ros::ServiceServer srv_trajectory_execution_;
      ros::ServiceServer srv_test_uibk_controller_;
      ros::ServiceServer srv_test_eddie_controller_;

      // publishers of this node
      ros::Publisher pub_robot_state_;

    public:
    
      // loop function to update the robot state
      void publishRobotState();

      // service function to request trajectory execution
      bool executeTrajectoryFromCode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res);
      bool executeTrajectory(const RobotEddie::Command::Seq &command);
      bool executeTrajectoryNonBlocking(const RobotEddie::Command::Seq &command);

      // service function to test the controller
      bool testEddieController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      // constructor
      GolemController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        // load configuration file from the launch file
        priv_nh_.param<std::string>("config_file", config_file_, "");

        // define the names passed in the urdf files corresponding to the current move group
        priv_nh_.param<std::string>("arm_name", arm_name_, "right");

        // create the controller object using the config file
        controller_ = BhamControl::create(config_file_);

        // advertise the node services
        srv_trajectory_execution_ = nh_.advertiseService(nh_.resolveName("/trajectory_execution_srv"),&GolemController::executeTrajectoryFromCode, this);
        srv_test_eddie_controller_ = nh_.advertiseService(nh_.resolveName("/test_eddie_controller_srv"),&GolemController::testEddieController, this);


        // advertise the node topics
        pub_robot_state_ = nh_.advertise<sensor_msgs::JointState>(nh_.resolveName("/golem/joint_states"), 10);

        // initialze the command 
        robot_eddie_command_.resize(1);

      }

      //! Empty stub
      ~GolemController() {}
  };

void GolemController::publishRobotState()
{
  // read the current data from the controller
  try 
  {  
    controller_->lookupState(controller_->time(), robot_eddie_state_);
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to read the robot state: %s\n", ex.what());
  }

  // convert from pacman to ros joint states 
  pacman::mapStates(robot_eddie_state_, joint_states_, ros::Time::now());

  // publish the joint states, this functionality could be ported to another node to increase speed and versatility.
  pub_robot_state_.publish(joint_states_);

  return;
}

bool GolemController::executeTrajectoryFromCode(definitions::TrajectoryExecution::Request &req, definitions::TrajectoryExecution::Response &res)
{

  robot_eddie_command_.clear();
  res.result = res.OTHER_ERROR;

  ROS_INFO("Exectution service requested...");

  definitions::Trajectory trajectory = req.trajectory;

  //convert the trajectory to the command ToDo: double check that the trajectory has velocity and acceleration
  pacman::convert(trajectory, robot_eddie_command_, controller_->time());

  // excexute the trajectory
  if(req.nonblocking)
  {
    if( executeTrajectoryNonBlocking(robot_eddie_command_) )
    {
        ROS_INFO("Commands sent correctly.");
        res.result = res.SUCCESS;
    }
    else
    {
      ROS_ERROR("Commands couldn't be sent to the controller.");
      res.result = res.OTHER_ERROR;
      return false;
    }
  }
  else
  {
    if( executeTrajectory(robot_eddie_command_) )
    {
        ROS_INFO("Execution finished cleanly...");
        res.result = res.SUCCESS;
    }
    else
    {
      ROS_ERROR("Trajectory could not be executed...");
      res.result = res.OTHER_ERROR;
      return false;
    }
  }

  return true;
}

bool GolemController::executeTrajectory(const RobotEddie::Command::Seq &command)
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

bool GolemController::executeTrajectoryNonBlocking(const RobotEddie::Command::Seq &command)
{
  try 
  {  
    controller_->send(command.data(), command.size());
  }
  catch (const std::exception& ex) 
  {
    ROS_ERROR("Unable to execute the given trajectory:  %s\n", ex.what());
    return false;
  }
  return true;
}

bool GolemController::testEddieController(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    try
    {
        // RobotEddie
        // Read current state
        RobotEddie::State begin;
        controller_->lookupState(controller_->time(), begin);
        // Initial configuration command
        RobotEddie::Command init;
        init.armLeft.pos = begin.armLeft.pos;
        init.handLeft.pos = begin.handLeft.pos;
        init.armRight.pos = begin.armRight.pos;
        init.handRight.pos = begin.handRight.pos;
        init.head.pos = begin.head.pos;
        // Prepare trajectory
        RobotEddie::Command::Seq commands(7);
        // Pose #0
        commands[0] = init;
        commands[0].t = controller_->time() + pacman::float_t(1.0);
        // Pose #1
        commands[1] = init;
        commands[1].armRight.pos.c[5] += pacman::float_t(0.5);
        commands[1].handRight.pos.middle[0] -= pacman::float_t(0.3);
        commands[1].handRight.pos.middle[1] -= pacman::float_t(0.3);
        commands[1].handRight.pos.left[0] -= pacman::float_t(0.3);
        commands[1].handRight.pos.left[1] -= pacman::float_t(0.3);
        commands[1].handRight.pos.right[0] -= pacman::float_t(0.3);
        commands[1].handRight.pos.right[1] -= pacman::float_t(0.3);
        commands[1].t = commands[0].t + pacman::float_t(2.0);
        // Pose #2
        commands[2] = init;
        commands[2].armRight.pos.c[5] -= pacman::float_t(0.5);
        commands[2].t = commands[1].t + pacman::float_t(2.0);
        // Pose #3
        commands[3] = init;
        commands[3].armRight.pos.c[6] += pacman::float_t(0.5);
        commands[3].armLeft.pos.c[6] += pacman::float_t(0.5);
        commands[3].handLeft.pos.middle[0] -= pacman::float_t(0.3);
        commands[3].handLeft.pos.middle[1] -= pacman::float_t(0.3);
        commands[3].handLeft.pos.left[0] -= pacman::float_t(0.3);
        commands[3].handLeft.pos.left[1] -= pacman::float_t(0.3);
        commands[3].handLeft.pos.right[0] -= pacman::float_t(0.3);
        commands[3].handLeft.pos.right[1] -= pacman::float_t(0.3);
        commands[3].t = commands[2].t + pacman::float_t(2.0);
        // Pose #4
        commands[4] = init;
        commands[4].armLeft.pos.c[6] -= pacman::float_t(0.5);
        commands[4].t = commands[3].t + pacman::float_t(2.0);
        // Pose #5
        commands[5] = init;
        commands[5].head.pos.neck[0] += pacman::float_t(0.1);
        commands[5].head.pos.neck[1] += pacman::float_t(0.1);
        commands[5].head.pos.neck[2] += pacman::float_t(0.1);
        commands[5].head.pos.neck[3] += pacman::float_t(0.1);
        commands[5].head.pos.eyeLeft -= pacman::float_t(0.4);
        commands[5].head.pos.eyeRight += pacman::float_t(0.4);
        commands[5].t = commands[4].t + pacman::float_t(2.0);
        // Pose #6
        commands[6] = init;
        commands[6].t = commands[5].t + pacman::float_t(2.0);

        // execute trajectory
        executeTrajectory(commands);
    }
    catch (const std::exception& ex)
    {
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
