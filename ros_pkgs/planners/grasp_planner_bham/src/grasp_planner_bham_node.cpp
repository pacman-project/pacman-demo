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
    bool doGraspPlanning(GraspPlanning::Request  &req, GraspPlanning::Response &res);
    // constructor
            GraspPlannerSrv(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
            {
             
	      // create objects of your classes
	      grasp_ = pacman::create("something.cfg");
    
                    
                // advertise service
                srv_set_parameters_ = 
                  nh_.advertiseService(nh_.resolveName("grasp_planner_bham"), 
                                                                &GraspPlannerSrv::doGraspPlanning, this)
            }
    
            //! Empty stub
            ~GraspPlannerSrv() {}    
  };
  
}


bool GraspPlannerSrv::doGraspPlanning(GraspPlanning::Request  &req, GraspPlanning::Response &res)
{
  //TODO datatype conversion from ordered_objects to PointClouds
  //Conversion   PointCloud = req.ordered_objects[0].cloud_from_mesh;
  
  grasp_->estimate(PointCloud,Trajectory);
  
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
