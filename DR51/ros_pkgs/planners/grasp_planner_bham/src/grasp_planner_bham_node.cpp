// system headers
#include <exception>

// ros headers
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// local headers
#include <definitions/GraspPlanning.h>
#include <pacman/Bham/Grasp/Grasp.h>
// #include <pacman/Bham/Grasp/GraspImpl.h>
#include <pacman/PaCMan/Defs.h>
#include <pacman/PaCMan/PCL.h>

// use pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
//#include <pcl/ros/conversions.h>

namespace grasp_planner_bham
{

class GraspPlanner
{
	private:
	// the node handle
	ros::NodeHandle nh_;

	// Node handle in the private namespace
	ros::NodeHandle priv_nh_;
	
	// the grasp object
	pacman::BhamGrasp::Ptr grasp_;

	// variables required to plan the grasp
	pacman::Point3D::Seq object_;
	pacman::RobotUIBK::Config::Seq trajectory_;

	// variable to write the result of the plan
	pacman::BhamGrasp::Trajectory::Seq planned_grasps_;

	// configuration files
	std::string config_file_;
	std::string class_file_;

	// path to the database
	std::string path_to_database_;

	// test files
	// only for testing
	std::string pcd_file_;

	// services
	ros::ServiceServer srv_grasp_planner_;

	// test the service with the test files with empty request/response (delete when node is finished)
	ros::ServiceServer srv_test_grasp_planner_;

	public:
	
	// helper function
	void poseToMatrix4f(geometry_msgs::Pose &pose,Eigen::Matrix4f &mat);
	
	double rotationAngleZ(const Eigen::Matrix4f &t1, const Eigen::Matrix4f &t2);
	
	bool checkWorkspaceHeuristic(geometry_msgs::Pose &pose, double maxAngle, double minZdist);

	// service function
	bool planGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res);

	// test service function
	bool test_planGrasp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	// constructor
	GraspPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{

		// load configuration file from the launch file
		nh_.param<std::string>("config_file", config_file_, "");
		
		// LOAD TEST FILES FROM THE LAUNCH FILE
		nh_.param<std::string>("class_file", class_file_, "");

		nh_.param<std::string>("path_to_DB",path_to_database_,"/home/pacman/Code/pacman/poseEstimation/dataFiles/PCD-MODELS-DOWNSAMPLED/");

		// create the grasp object using the config file
		grasp_ = pacman::BhamGrasp::create(config_file_);

		grasp_->load(class_file_);

		// advertise service
		srv_grasp_planner_ = nh_.advertiseService(nh_.resolveName("/grasp_planner_srv"),&GraspPlanner::planGrasp, this);

		srv_test_grasp_planner_ = nh_.advertiseService(nh_.resolveName("/test_grasp_planner_srv"),&GraspPlanner::test_planGrasp, this);
	}

	//! Empty stub
	~GraspPlanner() {}
};
	
double GraspPlanner::rotationAngleZ(const Eigen::Matrix4f &t1, const Eigen::Matrix4f &t2)
{
	double arc = t1.col(2).head(3).dot(t2.col(2).head(3));
	return fabs(acos(arc));
}


bool GraspPlanner::checkWorkspaceHeuristic(geometry_msgs::Pose &pose, double maxAngle, double minZdist) 
{
	//Directional approach vector(s), here along z-axis ~ Identity
	Eigen::Matrix4f approachDirection;
	
	approachDirection << 1,0,0,0,
						 0,1,0,0,
						 0,0,1,0,
						 0,0,0,1;
		
	//convert geometry msgs:
	Eigen::Matrix4f actualPose;
	poseToMatrix4f(pose, actualPose);
	
	//Calculate rotation angle difference  
	double diffAngle = fabs(rotationAngleZ(approachDirection, actualPose)-M_PI)*180/M_PI;
//   std::cout << "[Debug] Current rotation Angle with respect to Z-axis:" << diffAngle << std::endl;
	
	double zDist = actualPose(2,3);
//   std::cout << "[Debug] Current distance to Z-axis:" << zDist << std::endl;
	
	if (diffAngle < maxAngle && zDist > minZdist)
	{
		return true;
	}
	else
	{
		return false;
	}  
}


void GraspPlanner::poseToMatrix4f(geometry_msgs::Pose &pose,Eigen::Matrix4f &mat)
{
	mat = Eigen::Matrix4f::Identity(4,4);
	Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);      
	Eigen::Matrix3f rotation ( q.toRotationMatrix() );
	for(size_t i=0;i<3;i++)
	{
		for(size_t j=0;j<3;j++)
		{
				mat(i,j) = rotation(i,j);
		}
	}
	mat(0,3) = pose.position.x;
	mat(1,3) = pose.position.y;
	mat(2,3) = pose.position.z;   
}

bool GraspPlanner::planGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res)
{

	// CONVERSION FROM ROS DEFINITIONS TO PACMAN DATATYPE
	// get the object for which we would like to grasp
	definitions::Object object = req.ordered_objects[req.object_id];

	// create the string path to the object
	std::string path_to_object(path_to_database_);
	object.name.data.erase(std::remove(object.name.data.begin(), object.name.data.end(),'\n'), object.name.data.end());
	path_to_object += object.name.data;
	path_to_object += ".pcd";

	// load the pcd file
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (path_to_object.c_str(), *cloud) == -1) //* load the file
	{
		ROS_ERROR("Error at reading point cloud  %s... ", path_to_object.c_str());      
		ros::Duration(2).sleep();    
	}
				
	// transform pointcloud with the detected pose
	Eigen::Matrix4f transform_pose;
	poseToMatrix4f(object.pose, transform_pose);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transformPointCloudWithNormals(*cloud, *transformed_cloud, transform_pose);
	
	// convert to point cloud pacman
	pacman::Point3D::Seq object_pacman;
	pacman::convert(*transformed_cloud, object_pacman);

	ROS_INFO("Point cloud in pacman datatype with size: %ld", object_pacman.size() );

	// ESTIMATE
	// clear the member before calling the estimate again
	planned_grasps_.clear();
	grasp_->estimate(object_pacman, planned_grasps_);
	
	ROS_INFO("Number of planned grasps %ld",planned_grasps_.size());
	
	// check the result
	if(planned_grasps_.size() < 1)
	{
		ROS_INFO("No feasible grasp found");
		res.result = res.NO_FEASIBLE_GRASP_FOUND;
		return false;
	}

	//CONVERSION FROM PACMAN DATA TYPE TO ROS DEFINITIONS

	// create the message we are gonna fill as response
	std::vector<definitions::Grasp> estimated_grasps;
	definitions::Grasp grasp;

	// temp var to fill the eximation and convert to quaternion for the orientation part of the pose
	Eigen::Affine3d wrist_pose_mat;
	
	// the temporary pose
	geometry_msgs::PoseStamped pose_tmp;

	bool good_grasp;

	// std::cout << "pacman interface grasp " << planned_grasps_[0].trajectory[0].pose.R.m11 << " " 
	// << planned_grasps_[0].trajectory[0].pose.R.m12 << " "
	// << planned_grasps_[0].trajectory[0].pose.R.m13 << " "
	// << planned_grasps_[0].trajectory[0].pose.p.x << std::endl;
	// std::cout << "pacman interface grasp " << planned_grasps_[0].trajectory[0].pose.R.m21 << " "
	// << planned_grasps_[0].trajectory[0].pose.R.m22 << " "
	// << planned_grasps_[0].trajectory[0].pose.R.m23 << " "
	// << planned_grasps_[0].trajectory[0].pose.p.y << std::endl;
	// std::cout << "pacman interface grasp" << planned_grasps_[0].trajectory[0].pose.R.m31 << " " 
	// << planned_grasps_[0].trajectory[0].pose.R.m32 << " "
	// << planned_grasps_[0].trajectory[0].pose.R.m33 << " "
	// << planned_grasps_[0].trajectory[0].pose.p.z << std::endl;

	// note: convert (perhaps it is a good idea to have conversions between definitions and Defs.h in one header file inside pacman, 
	// the problem is that the definitions package might change)
	// loop
	// index t for trajectories
	// index w for wrist positions, typical 3 -> pre-grasp, middle point and grasp, but we leave it general for the future
	// remember for trajectory planning, that the grasps are assumed to be the wrist position, hence the value for the ARM_sdh_palm_link
	// and the hand joints
	for (int t = 0; t <  1/*planned_grasps_.size()*/; t++ )
	{
		// resize, this one should be typical to 3 in pacman
		grasp.grasp_trajectory.resize( planned_grasps_[t].trajectory.size() );

		good_grasp = false;
		for (int w = 0; w < planned_grasps_[t].trajectory.size(); w++)
		{
			wrist_pose_mat(0,0) = planned_grasps_[t].trajectory[w].pose.R.m11;
			wrist_pose_mat(0,1) = planned_grasps_[t].trajectory[w].pose.R.m12;
			wrist_pose_mat(0,2) = planned_grasps_[t].trajectory[w].pose.R.m13;
			wrist_pose_mat(0,3) = planned_grasps_[t].trajectory[w].pose.p.x;
			wrist_pose_mat(1,0) = planned_grasps_[t].trajectory[w].pose.R.m21;
			wrist_pose_mat(1,1) = planned_grasps_[t].trajectory[w].pose.R.m22;
			wrist_pose_mat(1,2) = planned_grasps_[t].trajectory[w].pose.R.m23;
			wrist_pose_mat(1,3) = planned_grasps_[t].trajectory[w].pose.p.y;
			wrist_pose_mat(2,0) = planned_grasps_[t].trajectory[w].pose.R.m31;
			wrist_pose_mat(2,1) = planned_grasps_[t].trajectory[w].pose.R.m32;
			wrist_pose_mat(2,2) = planned_grasps_[t].trajectory[w].pose.R.m33;
			wrist_pose_mat(2,3) = planned_grasps_[t].trajectory[w].pose.p.z;
			wrist_pose_mat(3,0) = 0.0;
			wrist_pose_mat(3,1) = 0.0;
			wrist_pose_mat(3,2) = 0.0;
			wrist_pose_mat(3,3) = 1.0;            

			tf::poseEigenToMsg(wrist_pose_mat, pose_tmp.pose);

			// fill the wrist pose
			grasp.grasp_trajectory[w].wrist_pose.pose = pose_tmp.pose;
			
			// for now, we are always planning grasps with respect to the world_link
			grasp.grasp_trajectory[w].wrist_pose.header.frame_id = "world_link";

			// and fill the hand joints
			grasp.grasp_trajectory[w].joints.resize(pacman:: SchunkDexHand::Config::JOINTS);
			grasp.grasp_trajectory[w].joints[0] = planned_grasps_[t].trajectory[w].config.rotation;
			grasp.grasp_trajectory[w].joints[1] = planned_grasps_[t].trajectory[w].config.left[0];
			grasp.grasp_trajectory[w].joints[2] = planned_grasps_[t].trajectory[w].config.left[1];
			grasp.grasp_trajectory[w].joints[3] = planned_grasps_[t].trajectory[w].config.right[0];
			grasp.grasp_trajectory[w].joints[4] = planned_grasps_[t].trajectory[w].config.right[1];
			grasp.grasp_trajectory[w].joints[5] = planned_grasps_[t].trajectory[w].config.middle[0];
			grasp.grasp_trajectory[w].joints[6] = planned_grasps_[t].trajectory[w].config.middle[1];
			
			//if ( (checkWorkspaceHeuristic(pose_tmp.pose, 90, 0.15) && w<1) )
			//{
				//ROS_INFO("Grasp Nr. %d is inside the workspace, hence added to the list of good grasps !", t);
			//	good_grasp = true;
			//}
			//else
			//{
				// only if w = 0 is not a good grasp
			//	if ( w==0 )
			//	{
					//ROS_WARN("Grasp Nr. %d is out of the workspace, hence discarded.", t);
			//		good_grasp = false;
			//	}
			//}
		}
		//if (good_grasp)
		//{
			// push back the current grasp only if it is good
			estimated_grasps.push_back(grasp);
		//}
	}
	// std::cout << "ros grasp " << estimated_grasps[0].grasp_trajectory[0].wrist_pose.pose << std::endl;

	// write the response
	res.grasp_list = estimated_grasps;

	ROS_INFO("Done!");
	res.result = res.SUCCESS;
	return true;
}

bool GraspPlanner::test_planGrasp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	try 
	{
		grasp_->estimate(object_, planned_grasps_);
	}
	catch (const std::exception& ex) 
	{
		printf("Grasp node exception: %s\n", ex.what());
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
