#include <string>
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <definitions/PoseEstimation.h>
#include <definitions/KinectGrabberService.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree_flann.h>

#include "pacman/UIBK/PoseEstimation/PoseEstimation.h"
#include <pacman/PaCMan/Defs.h>
#include <pacman/PaCMan/PCL.h>

// ROS generated headers
// typically these ones are generated at compilation time, 
// for instance, for a service called ClassParameterSetting.srv 
// within pose_estimation_uibk, we need to include this 
// #include "pose_estimation_uibk/ClassParameterSetting.h" 

//using namespace pacman;
using namespace std;
namespace pose_estimation_uibk 
{

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class PoseEstimator
{
  private:

    // suggested/quasi_mandatory members, note the member_name_ naming convention

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_estimate_poses_;
    
    ros::ServiceClient srv_kinect;
    ros::Publisher pub_rec_scene_;
    
    // ** disable this when using object cloud reader ** //

    Eigen::Matrix4f kinectToRobot;
    
    double offset_x,offset_y;
    sensor_msgs::PointCloud2 rec_scene_cloud;
     string path_to_config,pathToObjDb;

  public:

    // callback functions
    bool estimatePoses(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response);

    //Utility functions
    void poseEigenToMsg(const Eigen::Affine3d&, geometry_msgs::Pose&);

    void callback_kinect(const sensor_msgs::PointCloud2ConstPtr & input);
    void send_occlusion_shape(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > obj_pcds,vector<geometry_msgs::Pose> obj_poses);
    Eigen::Matrix4f transformPoseToUIBKPose(pacman::Mat34 pose_);
    
    // constructor
    PoseEstimator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {   
        pub_rec_scene_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("/pose_estimation_uibk/reconstructed_scene"), 10);

        // advertise service
        srv_estimate_poses_ = nh_.advertiseService(nh_.resolveName("/pose_estimation_uibk/estimate_poses"), &PoseEstimator::estimatePoses, this);

       kinectToRobot <<
           -0.065648,  -0.996571 , 0.0503785,   0.645214,
           -0.644791, 0.00383551,   -0.76435,   0.332272,
            0.761535, -0.0826616 , -0.642831 ,  0.714251,
            0,          0 ,         0,          1;        

    
       rec_scene_cloud.header.frame_id = "/world_link"; 
       rec_scene_cloud.header.seq = 0;
       nh_.param<std::string>("path_to_config",path_to_config, "");
       nh_.param<std::string>("path_to_object_db",pathToObjDb, "");
    }

    //! Empty stub
    ~PoseEstimator() {}
    void publish_scene();
    vector<int> give_object_ids(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
};
  

Eigen::Matrix4f PoseEstimator::transformPoseToUIBKPose(pacman::Mat34 pose_)
{
  Eigen::Matrix4f pose;
  pose <<
    pose_.R.m11,pose_.R.m12,pose_.R.m13,pose_.p.x,
    pose_.R.m21,pose_.R.m22,pose_.R.m23,pose_.p.y,
    pose_.R.m31,pose_.R.m32,pose_.R.m33,pose_.p.z,
    0,0,0,1;
  return pose;
}


bool PoseEstimator::estimatePoses(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response)
{   
    ROS_INFO("Pose estimation service has been reached..."); 
    ros::ServiceClient client = nh_.serviceClient<definitions::KinectGrabberService>("/kinect_grabber/kinect_grab_name");
    definitions::KinectGrabberService srv;
    srv.request.wake_up = "";
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points_ (new pcl::PointCloud<pcl::PointXYZ>());
 
    if( !client.call(srv))
    {   
        cout << "Error for calling grab_kinect service" << endl;
        return false;
    }   

    ros::Duration(0.5).sleep();  
    string filename_ = srv.response.path_to_pclfile;
       
    cout << "service call succeeded , " << filename_ << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_, *xyz_points_) == -1) //* load the file
    {
        cout << "Couldn't read file: " << filename_ << endl;
        return false;
    }

    ROS_INFO("Pose estimation service has been called...");
    
    ROS_INFO("Initializing...");
     
    //filename = "/home/pacman/CODE/pacman/poseEstimation/parametersFiles/config.txt";
    //pathToObjDb ="/home/pacman/CODE/pacman/poseEstimation/data/recognizedObjects";
    
    
    pacman::UIBKPoseEstimation* pose = pacman::UIBKPoseEstimation::create(path_to_config);
    pacman::UIBKObject* objects = pacman::UIBKObject::create(pathToObjDb);
    pacman::UIBKPoseEstimation::Pose::Seq poses_;
    cout << "after creating" << endl;
    pacman::Point3D::Seq points_;
    pose->load_cloud(filename_,points_);   
    cout << "after conversion" << endl;   
    pose->estimate(points_,poses_);
    
    ROS_INFO("Recognizing poses...");
        
    std::vector<definitions::Object> detected_objects(poses_.size());
    for (int i = 0; i < poses_.size (); i++)
    {    
        Eigen::Matrix4f pose = transformPoseToUIBKPose(poses_[i].pose);
        Eigen::Matrix4f transform_matrix = kinectToRobot.inverse() * pose;	
        
	std::cout <<    poses_[i].id << std::endl;
        std::cout << pose << std::endl;

        Eigen::Matrix4d md(transform_matrix.cast<double>());
        Eigen::Affine3d e = Eigen::Affine3d(md);

        geometry_msgs::Pose current_pose;
        poseEigenToMsg(e, current_pose);
	
        definitions::Object object;
        object.pose = current_pose;
        object.name.data = poses_[i].id;
        detected_objects[i] = object;
    }
    
    response.detected_objects = detected_objects;
    response.result = response.SUCCESS;
    
    ROS_INFO("Pose estimation service finisihed, and ready for another service requests...");
    return true;
}

vector<int> PoseEstimator::give_object_ids(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud)
{
    vector<int> ids;
     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     int K = 1;
   
     kdtree.setInputCloud (cloud);
     vector<int> pointIdxNKNSearch(K);
     vector<float> pointNKNSquaredDistance(K);
     vector<float> neighborsDistances;

     pcl::PointXYZ searchPoint;

     for (size_t i = 0; i < obj_cloud->points.size (); ++i)
     {
       searchPoint= obj_cloud->points[i];
   
      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
         ids.push_back(pointIdxNKNSearch[0]);           
     }
    return ids;
}

void PoseEstimator::poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m)
{
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0) 
    {
        m.orientation.x *= -1;
        m.orientation.y *= -1;
        m.orientation.z *= -1;
        m.orientation.w *= -1;
    }
}

} // namespace pose_estimation_uibk

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation_uibk_node");
    ros::NodeHandle nh;

    pose_estimation_uibk::PoseEstimator node(nh);
    ROS_INFO("Pose estimation node ready for service requests...");
    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}