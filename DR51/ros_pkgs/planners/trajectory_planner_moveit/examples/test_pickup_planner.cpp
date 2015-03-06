/*
 * pick_place.cpp
 *
 *  Created on: 17.03.2014
 *      Author: martin
 */

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/Grasp.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <definitions/TrajectoryPlanning.h>
#include <definitions/TrajectoryExecution.h>

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_msgs;

namespace trajectory_planner_moveit {

static const string BASE_LINK = "world_link";
static const double SUPPORT_SURFACE_HEIGHT = 0.08;

static const double CYLINDER_HEIGHT = 0.25;
static const double CYLINDER_RADIUS = 0.04;

/**
 * Sample for testing MoveIt pick and place functionality
 */
class PickPlace {

public:

	ros::ServiceClient planning_service_;
	ros::ServiceClient execution_service_;

	geometry_msgs::Pose start_pose_;
	geometry_msgs::Pose goal_pose_;

	ros::Publisher pub_attach_coll_obj_;

    PickPlace(ros::NodeHandle &nh) {

        planning_service_ = nh.serviceClient<definitions::TrajectoryPlanning>("/trajectory_planning_srv");
        planning_service_.waitForExistence();

		execution_service_ = nh.serviceClient<definitions::TrajectoryExecution>("trajectory_execution_srv");
        // execution_service_.waitForExistence();

		start_pose_ = getStartPose();
		goal_pose_ = getGoalPose();

		// Let everything load
		ros::Duration(2.0).sleep();
	}

	~PickPlace() {}

	bool start() {
		// ---------------------------------------------------------------------------------------------

		bool found = false;
		while (!found && ros::ok()) {

            if (!pick(start_pose_)) {
				ROS_ERROR_STREAM_NAMED("pick_place", "Pick failed. Retrying.");
			} else {
				ROS_INFO_STREAM_NAMED("pick_place",	"Done with pick!");
				found = true;
			}
		}

		return true;
	}

    bool pick(geometry_msgs::Pose& start_pose_) {
        ROS_WARN_STREAM_NAMED("", "picking object");

        definitions::TrajectoryPlanning planning_srv;
        planning_srv.request.arm = "right";
        planning_srv.request.type = planning_srv.request.PICK;

        // Pick grasp
        generateGrasps(start_pose_, planning_srv.request.ordered_grasp);

        if(!planning_service_.call(planning_srv)) {
            ROS_ERROR("Call to planning service failed!");
            return false;
        }

        definitions::TrajectoryPlanning::Response &response = planning_srv.response;

        if(!response.result == definitions::TrajectoryPlanningResponse::SUCCESS) {
            ROS_ERROR("Planning pickup phase failed!");
            return false;
        } else {
            ROS_INFO("Planning pickup phase sucessfully completed!");
        }

        ROS_INFO("Executing pickup...");

        definitions::TrajectoryExecution execution_srv;
        execution_srv.request.trajectory = response.trajectory[0];
        if(!execution_service_.call(execution_srv)) {
            ROS_ERROR("Call to execution service failed!");
            return false;
        }

        return execution_srv.response.result == definitions::TrajectoryExecutionResponse::SUCCESS;
    }

	geometry_msgs::Pose getStartPose() {
		geometry_msgs::Pose start_pose;

		// Position
		start_pose.position.x = 0.0;
		start_pose.position.y = 0.0;
		start_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		start_pose.orientation.x = 0;
		start_pose.orientation.y = 0;
		start_pose.orientation.z = 0;
		start_pose.orientation.w = 1;

		return start_pose;
	}

	geometry_msgs::Pose getGoalPose() {
		geometry_msgs::Pose goal_pose;

		// Position
		goal_pose.position.x = 0.0;
		goal_pose.position.y = -0.2;
		goal_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		goal_pose.orientation.x = 0;
		goal_pose.orientation.y = 0;
		goal_pose.orientation.z = 0;
		goal_pose.orientation.w = 1;

		return goal_pose;
	}

	definitions::SDHand getPreGraspPosture() {

		definitions::SDHand hand;

        hand.joints.push_back(0.0);
        hand.joints.push_back(-M_PI / 4);
        hand.joints.push_back(M_PI / 9);
        hand.joints.push_back(-M_PI / 4);
        hand.joints.push_back(M_PI / 9);
        hand.joints.push_back(-M_PI / 4);
        hand.joints.push_back(M_PI / 9);

        hand.velocity.resize(hand.joints.size());
        hand.acceleration.resize(hand.joints.size());

        for(size_t i = 0; i < hand.joints.size(); ++i) {
            hand.velocity[i] = 0.01;
            hand.acceleration[i] = 0.0;
        }

		return hand;
	}

	definitions::SDHand getGraspPosture() {

		definitions::SDHand hand;

        hand.joints.push_back(0.0);
        hand.joints.push_back(-M_PI / 14);
        hand.joints.push_back(M_PI / 6);
        hand.joints.push_back(-M_PI / 14);
        hand.joints.push_back(M_PI / 6);
        hand.joints.push_back(-M_PI / 14);
        hand.joints.push_back(M_PI / 6);

        hand.velocity.resize(hand.joints.size());
        hand.acceleration.resize(hand.joints.size());

        for(size_t i = 0; i < hand.joints.size(); ++i) {
            hand.velocity[i] = 0.01;
            hand.acceleration[i] = 0.0;
        }

		return hand;
	}

	bool generateGrasps(geometry_msgs::Pose &pose, vector<definitions::Grasp>& grasps) {

		// ---------------------------------------------------------------------------------------------
		// Create a transform from the object's frame to /base_link
		Eigen::Affine3d global_transform;
		tf::poseMsgToEigen(pose, global_transform);

		// ---------------------------------------------------------------------------------------------
		// Grasp parameters

		// -------------------------------
		// Create pre-grasp, middle and grasp posture (Gripper open)
		definitions::SDHand pre_grasp = getPreGraspPosture();;
		definitions::SDHand middle = getGraspPosture();
		definitions::SDHand grasp = getGraspPosture();

		// Create re-usable blank pose
		geometry_msgs::PoseStamped grasp_pose_msg;
		// grasp_pose_msg.header.stamp = ros::Time::now();
		grasp_pose_msg.header.frame_id = BASE_LINK;

		// ---------------------------------------------------------------------------------------------
		// Variables needed for calculations
		double radius = 0.19;
		double xb = 0;
		double yb = 0;
		double zb = 0;
		double theta = M_PI;

		/* Developer Note:
		 * Create angles 90 degrees around the chosen axis at given resolution
		 * We create the grasps in the reference frame of the object, then later convert it to the base link
		 */
		double angle_resolution = 10;

		for (int i = 0; i <= angle_resolution; ++i) {
			// Calculate grasp
			xb = radius * -sin(theta);
			yb = radius * cos(theta);

			Eigen::Affine3d grasp_pose;
			Eigen::Affine3d pre_grasp_pose;

			grasp_pose = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
					* Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

			grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);

			// Calculate pre grasp
			xb = (radius + 0.2) * -sin(theta);
			yb = (radius + 0.2) * cos(theta);

			pre_grasp_pose = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
					* Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

			pre_grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);

			theta -= M_PI / 2 / angle_resolution;


			// PreGrasp and Grasp Postures --------------------------------------------------------------------------

			definitions::SDHand new_pre_grasp = pre_grasp;
			definitions::SDHand new_middle = middle;
			definitions::SDHand new_grasp = grasp;


			// Grasp ------------------------------------------------------------------------------------------------

			// Convert pose to global frame (base_link)
			tf::poseEigenToMsg(global_transform * grasp_pose, grasp_pose_msg.pose);

			// The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
			new_grasp.wrist_pose = grasp_pose_msg;
			middle.wrist_pose = grasp_pose_msg;

			// same for pre grasp pose
			tf::poseEigenToMsg(global_transform * pre_grasp_pose, grasp_pose_msg.pose);
			pre_grasp.wrist_pose = grasp_pose_msg;

			// Add to vector
			definitions::Grasp grasp;

			grasp.grasp_trajectory.push_back(new_pre_grasp);
			grasp.grasp_trajectory.push_back(middle);
			grasp.grasp_trajectory.push_back(new_grasp);
			grasps.push_back(grasp);
		}

		ROS_INFO_STREAM_NAMED("pick_place", "Generated " << grasps.size() << " grasps.");

		return true;
	}
};

} // end trajectory_planner_moveit

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_pick_place");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

    trajectory_planner_moveit::PickPlace t(nh);
	t.start();

	return EXIT_SUCCESS;
}
