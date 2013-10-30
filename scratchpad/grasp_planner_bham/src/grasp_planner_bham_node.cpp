#include <ros/ros.h>
#include "definitions/GraspPlanning.h"
#include <pacman/Bham/Grasp/Grasp.h>


namespace grasp_planner_bham{

  class GraspPlannerSrv{
    private:
    // the node handle
    ros::NodeHandle nh_;
    // Node handle in the private namespace
    pacman::BhamGrasp* grasp_;

    // services
    ros::ServiceServer srv_set_parameters_;

    public:
    bool doGraspPlanning(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res);
    // constructor
            GraspPlannerSrv(ros::NodeHandle nh) : nh_(nh)
            {

	      // create objects of your classes
	      grasp_ = pacman::BhamGrasp::create("something.cfg");
	      pacman::Point3D::Seq object_;
	      RobotUIBK::Config::Seq trajectory_;
          pacman::load("object_file.txt", object_);
          pacman::load("trajectory_file.txt", trajectory_);
          grasp_->add("opis_obiektu_id", object_, trajectory_);

                // advertise service
                srv_set_parameters_ = nh_.advertiseService(nh_.resolveName("grasp_planner_bham"),&GraspPlannerSrv::doGraspPlanning, this);
            }

            //! Empty stub
            ~GraspPlannerSrv() {}
  };


}

  bool grasp_planner_bham::GraspPlannerSrv::doGraspPlanning(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res)
  {
    //TODO datatype conversion from ordered_objects to PointClouds
    //Conversion   PointCloud = req.ordered_objects[0].cloud_from_mesh;

    //grasp_->estimate(PointCloud,Trajectory);

    //Conversion res.grasp_list = convert(Trajectory)
    //TODO datatype conversion from Trajectory to grasp_list
    return true;
  }



int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_planner_bham");
  ros::NodeHandle nh;

  grasp_planner_bham::GraspPlannerSrv node(nh);

  ROS_INFO("Ready to plan grasp.");
  ros::spin();

  return 0;
}
