//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

//// local headers
#include <definitions/TrajectoryPlanning.h>

namespace trajectory_planner_moveit {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
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
  	bool planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

    // constructor
    FakePlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
		srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/trajectory_planning_srv"),&FakePlanner::planTrajectoryFromCode, this);
		//srv_test_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/test_trajectory_planning_srv"),&FakePlanner::planTrajectoryFromCode, this);
    }

    //! Empty stub
    ~FakePlanner() {}

};

bool FakePlanner::planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response) 
{
	ROS_INFO("Received trajectory planning requestof type %d", request.type);
	int type = request.type;
	switch (type)
	{
		case definitions::TrajectoryPlanning::Request::MOVE_TO_CART_GOAL:
			ROS_INFO("Succesfully planned a trajectory to the desired MOVE_TO_CART_GOAL");
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
