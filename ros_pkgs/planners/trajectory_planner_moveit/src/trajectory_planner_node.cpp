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


namespace trajectory_planner_moveit {

// template for the full trajectory planner node
class FakePlanner
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_trajectory_planning_;

  public:

  	// the service callback 
  	bool planTrajectory(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

    // constructor
    FakePlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
		srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/trajectory_planning_srv"),&FakePlanner::planTrajectory, this);
	}

    //! Empty stub
    ~FakePlanner() {}

};

bool FakePlanner::planTrajectory(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
    // create an instance of the helper class
    trajectory_planner_moveit::CartPlanner my_cart_planner(nh_);

	ROS_INFO("Received trajectory planning requestof type %d", request.type);
	int type = request.type;
	switch (type)
	{
		case definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL:
            if ( my_cart_planner.planTrajectoryFromCode(request, response) )
            {
                ROS_INFO("Succesfully planned a trajectory to the desired MOVE_TO_CART_GOAL");    
            }
            else
            {
                ROS_ERROR("Could not plan a trajectory to the desired MOVE_TO_CART_GOAL"); 
            }
			break;

		case definitions::TrajectoryPlanning::Request::PICK:
			ROS_INFO("Succesfully planned a trajectory to the desired PICK operation");
			break;

		case definitions::TrajectoryPlanning::Request::PLACE:
			ROS_INFO("Succesfully planned a trajectory to the desired PLACE operation");
			break;

		case definitions::TrajectoryPlanning::Request::MOVE_TO_STATE_GOAL:
			ROS_INFO("Succesfully planned a trajectory to the desired MOVE_TO_STATE_GOAL operation");
			break;

	}	
	return true;
}


} // namespace trajectory_planner_moveit


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_trajectory_planner_node");
    ros::NodeHandle nh;

    trajectory_planner_moveit::FakePlanner node(nh);

    ROS_INFO("This node is ready to do fake trajectory planning!");

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
