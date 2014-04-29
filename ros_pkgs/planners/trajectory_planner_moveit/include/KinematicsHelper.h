#ifndef IKHELPER_H
#define IKHELPER_H

//// ros headers
#include <ros/ros.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

//// local headers


#define ATTEMPTS 5
#define TIMEOUT 0.1


using namespace std;
using namespace ros;

namespace trajectory_planner_moveit {

/**
 * @brief The KinematicsHelper class
 *
 * Convenience class that provides easy access to the MoveIt FK and IK functionalies
 */
class KinematicsHelper {

private:
	ros::ServiceClient ik_client_;
	ros::ServiceClient fk_client_;

	bool computeIKInternal(const moveit_msgs::GetPositionIKRequest &request, moveit_msgs::RobotState &solution);

public:

	KinematicsHelper(NodeHandle &nh);
	~KinematicsHelper() {}


	/**
	 * @brief computeIK
	 *
	 * Calculate a valid IK solution for group with given name and for given pose goal
	 * Additionally can be specified, wheather the solution has to be collision free, or not.
	 * The default is set to true.
	 *
	 * Maximum number of attempts and timeout can also be specified.
	 *
	 * @param arm left or right
	 * @param goal
	 * @param solution
	 * @param avoid_collisions
	 * @param attempts
	 * @param timeout
	 * @return true on success
	 */
	bool computeIK(const string &arm,
				   const geometry_msgs::PoseStamped &goal,
				   moveit_msgs::RobotState &solution,
				   const string &plan_for = "",
				   const bool avoid_collisions = true,
				   const int attempts = ATTEMPTS,
				   const double timeout = TIMEOUT);
	/**
	 * @brief computeIK
	 *
	 * Calculate a valid IK solution for group with given name and for given pose goal and given seed state
	 * Additionally can be specified, wheather the solution has to be collision free, or not.
	 * The default is set to true.
	 *
	 * Maximum number of attempts and timeout can also be specified.
	 * @param arm left or right
	 * @param goal
	 * @param seed_state
	 * @param solution
	 * @param avoid_collisions
	 * @param attempts
	 * @param timeout
	 * @return
	 */
	bool computeIK(const string &arm,
				   const geometry_msgs::PoseStamped &goal,
				   const sensor_msgs::JointState &seed_state,
				   moveit_msgs::RobotState &solution,
				   const string &plan_for = "",
				   const bool avoid_collisions = true,
				   const int attempts = ATTEMPTS,
				   const double timeout = TIMEOUT);
	/**
	  Compute the cartesian position for link with given name, using given robot state.
	  Optionally an alternative reference frame id can be specified. Default is set to world_link

	 * @brief computeFK
	 * @param state
	 * @param link_name
	 * @param solution
	 * @param frame_id
	 * @return
	 */
	bool computeFK(const moveit_msgs::RobotState &state,
				   const string &link_name,
				   geometry_msgs::Pose &solution,
				   const string &frame_id = "world_link");

};




}

#endif // IKHELPER_H
