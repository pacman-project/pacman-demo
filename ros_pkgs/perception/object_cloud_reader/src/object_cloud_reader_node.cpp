// use <> for system headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/AttachedCollisionObject.h>
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
#include "definitions/ObjectCloudReader.h"

using namespace std;
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
    ros::ServiceServer srv_object_reader_;
    //publisher
    ros::Publisher pub_object_point_clouds_;

    std::string path_to_database_;

    // it is very useful to have a listener and broadcaster to know where all frames are
    tf::TransformListener tf_listener_;
    //tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher attached_object_publisher;

  public:

    // callback functions
    bool processObjects(definitions::ObjectCloudReader::Request& request, definitions::ObjectCloudReader::Response& response);
    void poseToMatrix4f(geometry_msgs::Pose &pose,Eigen::Matrix4f &mat); 

    void send_occlusion_shape(vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > obj_pcds,vector<geometry_msgs::Pose> obj_poses);
    // constructor
    ObjectReader(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        // advertise service
        srv_object_reader_ = nh_.advertiseService(nh_.resolveName("/object_reader"), &ObjectReader::processObjects, this);

        attached_object_publisher = nh_.advertise<moveit_msgs::AttachedCollisionObject>(nh_.resolveName("attached_collision_object"), 1,true);    
        // change this at will

        nh_.param<std::string>("path_to_RecObj",path_to_database_,"/home/pacman/CODE/pacman/poseEstimation/dataFiles/PCD-MODELS-DOWNSAMPLED/");
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
bool ObjectReader::processObjects(definitions::ObjectCloudReader::Request& request, definitions::ObjectCloudReader::Response& response)
{
    std::vector<definitions::Object> objects = request.detected_objects;
    vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > obj_pcds;
    vector<geometry_msgs::Pose> obj_poses;
    // 2. read the pcd files
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr current_scene (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for (int i=0;( (i<objects.size()) && ( (request.retreat < 0 ) || (request.object_id < 0 ) ) );i++)    
    {
        if( i == request.object_id ) 
            continue;
        std::string path_to_object(path_to_database_);
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
        obj_pcds.push_back(transform_cloud);
        obj_poses.push_back(objects[i].pose);
        
        for( size_t id = 0; id < transform_cloud->points.size(); id++ )
            current_scene->points.push_back(transform_cloud->points[id]);
    }  
    send_occlusion_shape(obj_pcds,obj_poses);

    response.result = response.SUCCESS;
    return true;
    
}

void ObjectReader::send_occlusion_shape(vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > obj_pcds,vector<geometry_msgs::Pose> obj_poses)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.header.frame_id = "world_link";
    attached_object.object.id = "box";
    attached_object.link_name = "world_link";
    double epsilon = 0;
    for( size_t i = 0; i < obj_pcds.size(); i++ )
    { 
      double min_x = 1000; double min_y = 1000;  double min_z = 1000;
      double max_x = 0; double max_y = 0; double max_z = 0;
      for( size_t j = 0; j < obj_pcds[i]->points.size(); j++ )
      {
        if( obj_pcds[i]->points[j].x < min_x )
            min_x = obj_pcds[i]->points[j].x;
        if( obj_pcds[i]->points[j].x > max_x )
            max_x = obj_pcds[i]->points[j].x;

        if( obj_pcds[i]->points[j].y < min_y )
            min_y = obj_pcds[i]->points[j].y;  
        if( obj_pcds[i]->points[j].y > max_y )
            max_y = obj_pcds[i]->points[j].y; 

        if( obj_pcds[i]->points[j].z < min_z )
            min_z = obj_pcds[i]->points[j].z;  
        if( obj_pcds[i]->points[j].z > max_z )
            max_z = obj_pcds[i]->points[j].z;                              
      }
      Eigen::Vector4f obj_center;
      pcl::compute3DCentroid(*obj_pcds[i],obj_center);
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;  
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = (max_x - min_x + epsilon);
      primitive.dimensions[1] = (max_y - min_y + epsilon);   
      primitive.dimensions[2] = (max_z - min_z + epsilon);
     
      geometry_msgs::Pose center = obj_poses[i]; 
      center.position.x = obj_center(0); center.position.y = obj_center(1); center.position.z = obj_center(2);
      //center.orientation.x = 0; center.orientation.y = 0; center.orientation.z = 0; center.orientation.w = 1;
    //  cout << "box size: " <<  primitive.dimensions[0] << " "<< primitive.dimensions[1] << " "<< primitive.dimensions[2] <<  endl;
      attached_object.object.primitives.push_back(primitive);
      attached_object.object.primitive_poses.push_back(center);
      attached_object.object.operation = attached_object.object.ADD;
    }
    attached_object_publisher.publish(attached_object);     
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
    }

    return 0;
}