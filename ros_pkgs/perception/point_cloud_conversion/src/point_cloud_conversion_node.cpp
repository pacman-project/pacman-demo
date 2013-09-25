#include <cstdio>
#include <string>
#include <iostream>
#include <cstdint>
// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL from ROS headers
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include "Defs.h"

// ROS generated headers
// typically these ones are generated at compilation time, 
// for instance, for a service called ClassParameterSetting.srv 
// within pacman_pose_estimation, we need to include this 
// #include "pacman_pose_estimation/ClassParameterSetting.h" 
//#include "definitions/PoseEstimation.h" 

namespace point_cloud_conversion {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class PointCloudConverter
{
  private:

    // suggested/quasi_mandatory members, note the member_name_ naming convention

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    pacman::Point3D::Seq pc_fill;

    // services
    ros::ServiceServer srv_convert_point_cloud_;

    // it is very useful to have a listener and broadcaster to know where all frames are
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

  public:

    // callback functions
    bool convertPointCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // constructor
    PointCloudConverter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {   
        // advertise service
        srv_convert_point_cloud_ = nh_.advertiseService(nh_.resolveName("/convert_point_cloud"), &PointCloudConverter::convertPointCloud, this);
    }

    //! Empty stub
    ~PointCloudConverter() {}

};

//bool PointCloudConverter::estimatePoses(PoseEstimation::Request &request, PoseEstimation::Response &response)
bool PointCloudConverter::convertPointCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
// 
    //ros::Time start_time = ros::Time::now();
    std::string topic = nh_.resolveName("/camera/depth_registered/points");
    ROS_INFO("Pose Estimation Service called; waiting for a point_cloud2 on topic %s", topic.c_str());

    sensor_msgs::PointCloud2::ConstPtr scene_ptr (new sensor_msgs::PointCloud2);
    scene_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
    
    if (!scene_ptr)
    {
        ROS_ERROR("Point Cloud Conversion service: no point_cloud2 has been received");
        return false;
    }

    
    ROS_INFO("scene_ptr->height %d", scene_ptr->height);
    ROS_INFO("scene_ptr->width %d", scene_ptr->width);
    ROS_INFO("scene_ptr->point_step %d", scene_ptr->point_step);
    ROS_INFO("scene_ptr->row_step %d", scene_ptr->row_step);
    ROS_INFO("scene_ptr->is_bigendian %d", scene_ptr->is_bigendian);

    sensor_msgs::PointField field;
    pc_fill.resize(scene_ptr->width*scene_ptr->height);
    
    uint8_t tab[4];
    float f = 0.0;
    
    for(int i=0;i<scene_ptr->height;i++){
        for(int j=0;j<scene_ptr->width;j++){
            
            // filling x field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[0];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            pc_fill[scene_ptr->width*i+j].position.x = f;

            // filling y field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[1];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            pc_fill[scene_ptr->width*i+j].position.y = f;

            // filling z field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[2];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            pc_fill[scene_ptr->width*i+j].position.z = f;
           
            // filling rgb field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[3];
            // std::cout << field.name << std::endl;
            for(int k=0;k<4;k++) pc_fill[scene_ptr->width*i+j].colour.uint8[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
        }    
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    for(int i=0;i<scene_ptr->height;i++){
        for(int j=0;j<scene_ptr->width;j++){
            
            // filling x field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[0];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            cloud->at(scene_ptr->width*i+j).x = f;

            // filling y field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[1];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            cloud->at(scene_ptr->width*i+j).y = f;

            // filling z field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[2];
            for(int k=0;k<4;k++) tab[k]=scene_ptr->data[k+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            f = *(const float*)tab;
            cloud->at(scene_ptr->width*i+j).z = f;
           
            // filling rgb field of the pacman::Point3D from sensor_msgs::PointCloud2
            field = scene_ptr->fields[3];
            // std::cout << field.name << std::endl;
            cloud->at(scene_ptr->width*i+j).r = scene_ptr->data[0+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            cloud->at(scene_ptr->width*i+j).g = scene_ptr->data[1+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            cloud->at(scene_ptr->width*i+j).b = scene_ptr->data[2+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
            cloud->at(scene_ptr->width*i+j).a = scene_ptr->data[3+field.offset+j*scene_ptr->point_step+i*scene_ptr->row_step];
        }    
    }

    return true;
}

} // namespace pacman_pose_estimation

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_converter_node");
    ros::NodeHandle nh;

    point_cloud_conversion::PointCloudConverter node(nh);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}