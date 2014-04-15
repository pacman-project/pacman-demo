//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

//// generated headers
#include <definitions/TrajectoryPlanning.h>

//// local headers
#include "CartPlanner.h"
<<<<<<< HEAD
#include "pickupplanner.h"
=======
#include "StatePlanner.h"
>>>>>>> 5051bb3aeab05516c4be4d29c48990a62b74554e


namespace trajectory_planner_moveit {

// template for the full trajectory planner node
class TrajPlanner
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_trajectory_planning_;

    // the trajectory planner helper class
    trajectory_planner_moveit::CartPlanner *my_cart_planner_;
    trajectory_planner_moveit::StatePlanner *my_state_planner_;
     
    // the helper class for planning the pickup phase
    trajectory_planner_moveit::PickupPlannerPtr pickup_planner_;

   

  public:

  	// the service callback 
  	bool planTrajectory(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

    // constructor
    TrajPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
       // advertise the main service
	   srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/trajectory_planning_srv"),&TrajPlanner::planTrajectory, this);
	   
       // init class members
       my_cart_planner_ = new trajectory_planner_moveit::CartPlanner(nh_);
       my_state_planner_ = new trajectory_planner_moveit::StatePlanner(nh_);
       
       pickup_planner_.reset(new PickupPlanner(nh));
       
    }

    //! Empty stub
    ~TrajPlanner() {}

};

bool TrajPlanner::planTrajectory(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	ROS_INFO("Received trajectory planning requestof type %d", request.type);
	int type = request.type;
	switch (type)
	{
		case definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL:
            if ( my_cart_planner_->planTrajectoryFromCode(request, response) )
            {
                ROS_INFO("Succesfully planned a trajectory to the desired MOVE_TO_CART_GOAL");    
            }
            else
            {
                ROS_ERROR("Could not plan a trajectory to the desired MOVE_TO_CART_GOAL"); 
            }
			break;

		case definitions::TrajectoryPlanning::Request::PICK:
            if(pickup_planner_->planPickup(request, response)) {
                ROS_INFO("Succesfully created a motion plan for the desired PICK operation");
            } else {
                ROS_ERROR("Failed to find a valid motion plan for the desired PICK operation");
            }

			break;

		case definitions::TrajectoryPlanning::Request::PLACE:
			ROS_INFO("Succesfully planned a trajectory to the desired PLACE operation");
			break;

		case definitions::TrajectoryPlanning::Request::MOVE_TO_STATE_GOAL:
			if ( my_state_planner_->planTrajectoryFromCode(request, response) )
            {
                ROS_INFO("Succesfully planned a trajectory to the desired MOVE_TO_STATE_GOAL");    
            }
            else
            {
                ROS_ERROR("Could not plan a trajectory to the desired MOVE_TO_STATE_GOAL"); 
            }
			break;

	}	
	return true;
}


} // namespace trajectory_planner_moveit


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh;

    trajectory_planner_moveit::TrajPlanner node(nh);

    ROS_INFO("This node is ready to do trajectory planning!");

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
