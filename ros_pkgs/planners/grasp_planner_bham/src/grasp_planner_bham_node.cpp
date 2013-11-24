// system headers
#include <exception>

// ros headers
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// local headers
#include "definitions/GraspPlanning.h"
#include "pacman/Bham/Grasp/Grasp.h"


namespace grasp_planner_bham{

  class GraspPlanner{
    private:
    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    
    // the grasp object
    pacman::BhamGrasp* grasp_;

    // variables required to plan the grasp
    pacman::Point3D::Seq object_;
    pacman::RobotUIBK::Config::Seq trajectory_;

    // variable to write the result of the plan
    pacman::BhamGrasp::Trajectory::Seq planned_grasp_;

    // configuration files
    std::string config_file_;

    // test files
    std::string trajectory_file_;
    std::string pcd_file_;

    // services
    ros::ServiceServer srv_grasp_planner_;

    // test the service with the test files with empty request/response (delete when node is finished)
    ros::ServiceServer srv_test_grasp_planner_;

    public:
    
    // service function
    bool planGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res);

    // test service function
    bool test_planGrasp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // constructor
            GraspPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
            {

              // load configuration file from the launch file
              priv_nh_.param<std::string>("config_file", config_file_, "");

              // LOAD TEST FILES FROM THE LAUNCH FILE
              // BIG NOTE: FOR THE DEMO, THIS SHOULD BE PASSED IN THE REQUEST OF THE SERVICE SOMEHOW, THE PCD FILE CAN BE 
              // DONE USING THE POSE ESTIMATION RESULT, HOWEVER, THE TRAJECTORY? IT SEEMS THAT THE TRAJECTORY MUST BE 
              // ALREADY IN THE DATA BASE WITH THE ASSOCIATED OBJECT, RIGHT?
              priv_nh_.param<std::string>("trajectory_file", trajectory_file_, "");
              priv_nh_.param<std::string>("pcd_file", pcd_file_, "");

              // create the grasp object using the config file
              grasp_ = pacman::BhamGrasp::create(config_file_);

              pacman::load(pcd_file_, object_);
              pacman::load(trajectory_file_, trajectory_);
              grasp_->add("pacman_mug1", object_, trajectory_);

              // advertise service
              srv_grasp_planner_ = nh_.advertiseService(nh_.resolveName("grasp_planner_bham_srv"),&GraspPlanner::planGrasp, this);

              srv_test_grasp_planner_ = nh_.advertiseService(nh_.resolveName("test_grasp_planner_bham_srv"),&GraspPlanner::test_planGrasp, this);
            }

            //! Empty stub
            ~GraspPlanner() {}
  };




bool GraspPlanner::planGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res)
{
  //TODO datatype conversion from ordered_objects to PointClouds
  //Conversion   PointCloud = req.ordered_objects[0].cloud_from_mesh;

  //grasp_->estimate(PointCloud,Trajectory);

  //Conversion res.grasp_list = convert(Trajectory)
  //TODO datatype conversion from Trajectory to grasp_list
  return true;
}

bool GraspPlanner::test_planGrasp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  try {
    grasp_->estimate(object_, planned_grasp_);
  }
  catch (const std::exception& ex) {
    printf("GraspTest exception: %s\n", ex.what());
    return false;
  }

  return true;
}

} // end namespace grasp_planner_bham

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_planner_bham");
  ros::NodeHandle nh;

  grasp_planner_bham::GraspPlanner node(nh);

  ROS_INFO("Ready to plan grasp.");
  ros::spin();

  return 0;
}
