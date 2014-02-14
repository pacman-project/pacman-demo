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

namespace grasp_planner_bham{

  class GraspPlanner{
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

    // test files
    std::string trajectory_file_;
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
	      
              nh_.param<std::string>("trajectory_file", trajectory_file_, "");
              nh_.param<std::string>("pcd_file", pcd_file_, "");

              // create the grasp object using the config file
              grasp_ = pacman::BhamGrasp::create(config_file_);

              pacman::load(pcd_file_, object_);
              pacman::load(trajectory_file_, trajectory_);
              grasp_->add("pacman_object", object_, trajectory_);

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
  std::string path_to_database("/home/pacman/CODE/poseEstimation/dataFiles/PCD-MODELS-DOWNSAMPLED/");
  std::string path_to_object(path_to_database);
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
  
  // Object cloud centroid, debugging
  
//   Eigen::Vector4f centroid_pcl;
//   pcl::compute3DCentroid( *transformed_cloud, centroid_pcl );  
//   std::cout << "[DEBUG] Object Centroid PCL PC: " << centroid_pcl.head(3) << std::endl;
  
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
  definitions::Grasp current_trajectory;

  // resize with the result of the estimation
//   estimated_grasps.resize(planned_grasps_.size());


  // temp var to fill the eximation and convert to quaternion for the orientation part of the pose
  Eigen::Affine3d wrist_pose_TCP;
  Eigen::Matrix4f wrist_pose_mat;
  
  //UIBK robot transform to TCP
  
  Eigen::Matrix4f transToTCPinHand;
  Eigen::Matrix4f transToTCPinWorld;
  Eigen::Matrix4f tempRotMat;
  Eigen::Matrix4f wristPoseTCP;
  //Offset of Hand to KUKA TCP frame + 0.015 m
  transToTCPinHand << 1,0,0,0,
		      0,1,0,0,
		      0,0,1,0.015,
		      0,0,0,1;

  // the temporary pose
  geometry_msgs::PoseStamped pose_tmp;

  // note: convert (perhaps it is a good idea to have conversions between definitions adn Defs.h in one header file inside pacman, 
  // the problem is that the definitions package might change)
  // loop
  // index t for trajectories
  // index w for wrist positions, always 3 pre-grasp, middle point and grasp
  for (int t = 0; t <  planned_grasps_.size(); t++ )
  {
    // resize, this one should be typical to 3 in pacman
     //estimated_grasps[t].grasp_trajectory.resize(planned_grasps_[t].trajectory.size());
     current_trajectory.grasp_trajectory.resize(planned_grasps_[t].trajectory.size());

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
      
      //Transformation to kuka TCP along deltaZ = 0.015 m
      tempRotMat = Eigen::Matrix4f::Identity();
      tempRotMat.block<3,3>(0,0)= wrist_pose_mat.block<3,3>(0,0);
      
      transToTCPinWorld = tempRotMat*transToTCPinHand;
      transToTCPinWorld.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
      
      wristPoseTCP = transToTCPinWorld*wrist_pose_mat;
      
      if ( (t==0) && (w==2) ) {
      std::cout << "[DEBUG] Hand frame before transformation to KUKA TCP:" << wrist_pose_mat << std::endl;
      std::cout << "[DEBUG] Hand frame after transformation (best grasp):" << wristPoseTCP << std::endl;
      
      
	/*Eigen::Vector3f handFramePos2;
	handFramePos2 << wristPoseTCP(0,3), wristPoseTCP(1,3),wristPoseTCP(2,3);
	float dist = (centroid_pcl.head(3)-handFramePos2).norm();
  
        std::cout << "[DEBUG] Distance TCP frame to Centroid" << dist << std::endl;*/
      }
            
      
      wrist_pose_TCP = wristPoseTCP.cast<double>();
       tf::poseEigenToMsg(wrist_pose_TCP, pose_tmp.pose);
      current_trajectory.grasp_trajectory[w].wrist_pose.pose = pose_tmp.pose;
      //Filtering heuristic: z value has to be bigger than a certain amount and rotation has to look along a certain direction (top grasp), define maximum angle on quaternion

    }
    if (checkWorkspaceHeuristic(current_trajectory.grasp_trajectory[2].wrist_pose.pose, 90, 0.15) )
	{
	   std::cout << "Grasp Nr." << t <<  std::endl;
	   std::cout << current_trajectory.grasp_trajectory[2].wrist_pose <<  std::endl;   
        //ROS_INFO("pose.pose.position.x %f", pose.pose.position.x);
      
         estimated_grasps.push_back(current_trajectory);  
//       estimated_grasps[t].grasp_trajectory[w].wrist_pose.pose = pose_tmp.pose;	  
	}    
    
  }
  
  //Debug printing center of Hand Frame  

//   Eigen::Vector3f handFramePos;
//   handFramePos << planned_grasps_[0].trajectory[2].pose.p.x,  planned_grasps_[0].trajectory[2].pose.p.y,  planned_grasps_[0].trajectory[2].pose.p.z;
//   std::cout << "[DEBUG] HandFrame Centroid:" << handFramePos << std::endl; 
//   float dist = (centroid_pcl.head(3)-handFramePos).norm();
//   std::cout << "[DEBUG] Distance Hand frame to Centroid" << dist << std::endl;

  // write the response
  res.grasp_list = estimated_grasps;

  ROS_INFO("Done!");
  res.result = res.SUCCESS;
  return true;
}

bool GraspPlanner::test_planGrasp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  try {
    grasp_->estimate(object_, planned_grasps_);
  }
  catch (const std::exception& ex) {
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
