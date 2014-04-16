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
static const string EE_PARENT_LINK = "right_arm_7_link";
static const string OBJ_ID = "cylinder1";
static const string OBSTACLE_ID = "obstacle";

static const double SUPPORT_SURFACE_HEIGHT = 0.08;

static const double CYLINDER_HEIGHT = 0.25;
static const double CYLINDER_RADIUS = 0.04;

/**
 * Sample for testing MoveIt pick and place functionality
 */
class PickPlace {

public:
	// our interface with MoveIt
	boost::shared_ptr<PlanningSceneInterface> planning_scene_interface_;

	ros::ServiceClient planning_service_;
	ros::ServiceClient execution_service_;

	geometry_msgs::Pose start_pose_;
	geometry_msgs::Pose goal_pose_;

	ros::Publisher pub_attach_coll_obj_;

	PickPlace(ros::NodeHandle &nh, const string &planner_id) {

		// Create MoveGroup for one of the planning groups
		planning_scene_interface_.reset(new PlanningSceneInterface());

        planning_service_ = nh.serviceClient<definitions::TrajectoryPlanning>("/trajectory_planning_srv");
        planning_service_.waitForExistence();

		execution_service_ = nh.serviceClient<definitions::TrajectoryExecution>("trajectory_execution_srv");
        execution_service_.waitForExistence();

		pub_attach_coll_obj_ = nh.advertise<AttachedCollisionObject>("attached_collision_object", 10);

		start_pose_ = getStartPose();
		goal_pose_ = getGoalPose();

		// Let everything load
		ros::Duration(2.0).sleep();
	}

	~PickPlace() {}

	bool start() {
		// ---------------------------------------------------------------------------------------------

		// create obstacle and object to move...
		// createEnvironment();

		bool found = false;
		while (!found && ros::ok()) {

			if (!pick(start_pose_, OBJ_ID)) {
				ROS_ERROR_STREAM_NAMED("pick_place", "Pick failed. Retrying.");
				//cleanupACO(OBJ_ID);
			} else {
				ROS_INFO_STREAM_NAMED("pick_place",	"Done with pick!");
				found = true;
			}
		}

		ROS_INFO_STREAM_NAMED("simple_pick_place", "Waiting to put...");
		ros::Duration(5.5).sleep();

		bool placed = false;
		while (!placed && ros::ok()) {
			if (!place(goal_pose_, OBJ_ID)) {
				ROS_ERROR_STREAM_NAMED("pick_place", "Place failed.");
			} else {
				ROS_INFO_STREAM_NAMED("pick_place", "Done with place");
				placed = true;
			}
		}

		ROS_INFO_STREAM_NAMED("pick_place", "Pick and place cycle complete");

		return true;
	}

	void createEnvironment() {
		// Remove arbitrary existing objects
		vector<string> object_ids;
		object_ids.push_back(OBSTACLE_ID);
		object_ids.push_back(OBJ_ID);
		planning_scene_interface_->removeCollisionObjects(object_ids);
		
		// create our collision objects:
		std::vector<moveit_msgs::CollisionObject> collision_objects;
		// First, we will define the collision object message for our obstacle.
		moveit_msgs::CollisionObject obstacle;
		obstacle.header.frame_id = BASE_LINK;

		/* The id of the object is used to identify it. */
		obstacle.id = OBSTACLE_ID;

		// create an obstacle
		shape_msgs::SolidPrimitive box;
		box.type = box.BOX;
		box.dimensions.resize(3);
		box.dimensions[0] = 0.3;
		box.dimensions[1] = 0.2;
		box.dimensions[2] = 0.2;

		/* A pose for the box (specified relative to frame_id) */
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.25;
		box_pose.position.y = 0.25;
		box_pose.position.z = 0.1 + SUPPORT_SURFACE_HEIGHT;

		obstacle.primitives.push_back(box);
		obstacle.primitive_poses.push_back(box_pose);
		obstacle.operation = obstacle.ADD;

		collision_objects.push_back(obstacle);

		// create the cyliinder to pick
		moveit_msgs::CollisionObject cylinder_object;

		cylinder_object.header.frame_id = BASE_LINK;
		cylinder_object.id = OBJ_ID;

		// create an obstacle
		shape_msgs::SolidPrimitive cylinder;
		cylinder.type = cylinder.CYLINDER;
		cylinder.dimensions.resize(3);
		cylinder.dimensions[0] = CYLINDER_HEIGHT;
		cylinder.dimensions[1] = CYLINDER_RADIUS;

		/* A pose for the cylinder (specified relative to frame_id) */
		geometry_msgs::Pose cylinder_pose = getStartPose();

		cylinder_object.primitives.push_back(cylinder);
		cylinder_object.primitive_poses.push_back(cylinder_pose);
		cylinder_object.operation = obstacle.ADD;

		collision_objects.push_back(cylinder_object);

		// Now, let's add the collision object into the world
		ROS_INFO("Add an object into the world");
		planning_scene_interface_->addCollisionObjects(collision_objects);

		/* Sleep so we have time to see the object in RViz */
		sleep(2.0);
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

		return hand;
	}

	void cleanupACO(const string &name) {
		// Clean up old attached collision object
		moveit_msgs::AttachedCollisionObject aco;
		aco.object.header.stamp = ros::Time::now();
		aco.object.header.frame_id = BASE_LINK;

		//aco.object.id = name;
		aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

		aco.link_name = EE_PARENT_LINK;

		ros::WallDuration(0.1).sleep();
		pub_attach_coll_obj_.publish(aco);
	}

	bool pick(geometry_msgs::Pose& start_pose_, std::string name) {
		ROS_WARN_STREAM_NAMED("", "picking object "<< name);

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

	bool place(const geometry_msgs::Pose& goal_pose, std::string name) {
//		ROS_WARN_STREAM_NAMED("pick_place", "Placing "<< name);
//
//		std::vector<PlaceLocation> place_locations;
//
//		trajectory_msgs::JointTrajectory post_place_posture = getPreGraspPosture();
//
//		// Re-usable datastruct
//		geometry_msgs::PoseStamped pose_stamped;
//		pose_stamped.header.frame_id = BASE_LINK;
//		// pose_stamped.header.stamp = ros::Time::now();
//
//		// Create 360 degrees of place location rotated around a center
//		for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
//			pose_stamped.pose = goal_pose;
//
//			// Orientation
//			Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
//			pose_stamped.pose.orientation.x = quat.x();
//			pose_stamped.pose.orientation.y = quat.y();
//			pose_stamped.pose.orientation.z = quat.z();
//			pose_stamped.pose.orientation.w = quat.w();
//
//			// Create new place location
//			PlaceLocation place_loc;
//
//			place_loc.place_pose = pose_stamped;
//
//			// Approach
//			GripperTranslation gripper_approach;
//			// gripper_approach.direction.header.stamp = ros::Time::now();
//			gripper_approach.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
//			gripper_approach.min_distance = 0.1;
//			gripper_approach.direction.header.frame_id = EE_PARENT_LINK;
//			gripper_approach.direction.vector.x = 1;
//			gripper_approach.direction.vector.y = 0;
//			gripper_approach.direction.vector.z = 0;
//			place_loc.pre_place_approach = gripper_approach;
//
//			// Retreat
//			GripperTranslation gripper_retreat;
//			// gripper_retreat.direction.header.stamp = ros::Time::now();
//			gripper_retreat.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
//			gripper_retreat.min_distance = 0.1;
//			gripper_retreat.direction.header.frame_id = EE_PARENT_LINK;
//			gripper_retreat.direction.vector.x = 0;
//			gripper_retreat.direction.vector.y = 0;
//			gripper_retreat.direction.vector.z = -1; // Retreat direction (pos z axis)
//			place_loc.post_place_retreat = gripper_retreat;
//
//			// Post place posture - use same as pre-grasp posture (the OPEN command)
//			place_loc.post_place_posture = post_place_posture;
//
//			place_locations.push_back(place_loc);
//		}
//
////		moveit_msgs::OrientationConstraint oc;
////		oc.header.frame_id = BASE_LINK;
////		oc.link_name = EE_PARENT_LINK;
////
////		oc.orientation.x = 0;
////		oc.orientation.y = 0;
////		oc.orientation.z = 0;
////		oc.orientation.w = 1;
////
////		oc.absolute_x_axis_tolerance = 0.3;
////		oc.absolute_y_axis_tolerance = 0.3;
////		oc.absolute_z_axis_tolerance = 0.3;
////
////		oc.weight = 1;
////
////		moveit_msgs::Constraints constraints;
////		constraints.orientation_constraints.push_back(oc);
//
//		PlanningResultPtr plan = plan_execution_->plan_place(name, place_locations);
//
//		if(plan->status == PlanningHelper::SUCCESS) {
//			ROS_INFO("Planning placement phase sucessfully completed!");
//		} else {
//			ROS_WARN("Planning placement phase failed!");
//			return false;
//		}
//
//		ROS_INFO("Executing placement...");
//
//		return plan_execution_->execute(plan);
		return true;
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

	string planner = "LBKPIECEkConfigDefault";
	if(argc > 1) {
		planner = argv[1];
	}

    trajectory_planner_moveit::PickPlace t(nh, planner);
	t.start();

	return EXIT_SUCCESS;
}
