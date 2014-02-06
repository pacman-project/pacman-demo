// use <> for system headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

// use pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/ros/conversions.h>

// ROS generated headers
// typically these ones are generated at compilation time, 
// for instance, for a service called ClassParameterSetting.srv 
// within object_cloud_reader, we need to include this 
//#include "definitions/PoseEstimation.h" 
#include "definitions/ObjectCloudReader.h" 
namespace object_cloud_reader {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class ObjectReader
{
  private:

    // suggested/quasi_mandatory members, note the member_name_ naming convention

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    //ros::ServiceServer srv_estimate_poses_;
    ros::ServiceServer srv_object_reader_;
    //publisher
    ros::Publisher pub_object_point_clouds_;

    // it is very useful to have a listener and broadcaster to know where all frames are
    tf::TransformListener tf_listener_;
    //tf::TransformBroadcaster tf_broadcaster_;

    // reconstructed scene
    sensor_msgs::PointCloud2 current_scene_ros_;

  public:

    // callback functions
    //bool processObjects(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response);
    bool processObjects(definitions::ObjectCloudReader::Request& request, definitions::ObjectCloudReader::Response& response);
    void poseToMatrix4f(geometry_msgs::Pose &pose,Eigen::Matrix4f &mat); 

    // publishes the reconstructed scene
    void publishScene();

    // constructor
    ObjectReader(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        // advertise service
        //srv_estimate_poses_ = nh_.advertiseService(nh_.resolveName("/pose_estimation_uibk"), &ObjectReader::processObjects, this);
        srv_object_reader_ = nh_.advertiseService(nh_.resolveName("/object_reader"), &ObjectReader::processObjects, this);
        //pub_object_point_clouds_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("/object_clouds_from_mesh"), 10,true);
        pub_object_point_clouds_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("/reconstructed_scene"), 10);

        // change this at will
        current_scene_ros_.header.frame_id = "world_link";
    }

    //! Empty stub
    ~ObjectReader() {}

};

void ObjectReader::poseToMatrix4f(geometry_msgs::Pose &pose,Eigen::Matrix4f &mat)
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

// this function is called when service_name_you_want_to_advertise is advertise
//bool ObjectReader::processObjects(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response)
bool ObjectReader::processObjects(definitions::ObjectCloudReader::Request& request, definitions::ObjectCloudReader::Response& response)
{

    /*std::string topic = "/camera/depth_registered/points";

    sensor_msgs::PointCloud2::ConstPtr scene_ptr (new sensor_msgs::PointCloud2);
    scene_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
    
    if (!scene_ptr)
    {
        ROS_ERROR("Point Cloud Conversion service: no point_cloud2 has been received");
        return false;
    }*/
    


    // 1. call /estimate_pose 
    /*definitions::PoseEstimation pose_estimation_result;
    std::string pose_estimation_service_name("/estimate_poses");
    while ( !ros::service::waitForService(pose_estimation_service_name, ros::Duration().fromSec(3.0)) )
    {
        ROS_ERROR("Waiting for service %s...", pose_estimation_service_name.c_str());
        ros::Duration(2).sleep();  

    }

    if (!ros::service::call(pose_estimation_service_name, pose_estimation_result))
    {
        ROS_ERROR("Call to pose estimatione service failed. Continue...");
        ros::Duration(2).sleep();  
        return false; 
    }*/

    //std::vector<definitions::Object> objects = pose_estimation_result.response.detected_objects;
    std::vector<definitions::Object> objects = request.detected_objects;
    
    // 2. read the pcd files
    std::string path_to_database("/home/pacman/poseEstimation/data/PCD-MODELS-DOWNSAMPLED/");
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr current_scene (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (int i=0;i<objects.size();i++)
    {
        if( i == request.object_id )
            continue;
        std::string path_to_object(path_to_database);
        objects[i].name.data.erase(std::remove(objects[i].name.data.begin(), objects[i].name.data.end(),'\n'), objects[i].name.data.end());
        path_to_object += objects[i].name.data;
        path_to_object += ".pcd";

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (path_to_object.c_str(), *cloud) == -1) //* load the file
        {
          ROS_ERROR("Error at reading point cloud  %s... ", path_to_object.c_str());      
          ros::Duration(2).sleep();    
        }
        
        //transform pointcloud with the detected pose
        Eigen::Matrix4f transform_pose;
        poseToMatrix4f(objects[i].pose, transform_pose);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        transformPointCloudWithNormals(*cloud, *transform_cloud, transform_pose); 
        for( size_t id = 0; id < transform_cloud->points.size(); id++ )
            current_scene->points.push_back(transform_cloud->points[id]);
       /* sensor_msgs::PointCloud2 current_object;

        pcl::toROSMsg(*transform_cloud, current_object);

        current_object.header = scene_ptr->header;

        pub_object_point_clouds_.publish(current_object);     */  

    }  
    
    pcl::toROSMsg(*current_scene, current_scene_ros_);    
    response.rec_scene = current_scene_ros_;
    response.result = response.SUCCESS;
    return true;
    
}

void ObjectReader::publishScene()
{
    pub_object_point_clouds_.publish(current_scene_ros_);
}



} // namespace object_cloud_reader

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_cloud_reader_node");
    ros::NodeHandle nh;

    object_cloud_reader::ObjectReader node(nh);

    while(ros::ok())
    {
        ros::spinOnce();
        node.publishScene();
    }

    return 0;
}