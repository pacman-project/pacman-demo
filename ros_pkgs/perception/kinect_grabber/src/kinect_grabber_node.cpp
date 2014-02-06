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
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <definitions/KinectGrabberService.h>

using namespace std;

namespace kinect_grabber{
//class grabnode
class grabkinect
{
 public:

    // the node handle
    
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_estimate_poses_;
    
    ros::Publisher pub_object_point_clouds_;
    
    ros::Subscriber subKinect;
    
    ros::ServiceServer srv_kinect_;
    
    int id;
    bool REC_TIMES_KINECT;
    string file_name;
    string root_file_;
    
    // callback functions
    bool advertise_frame(definitions::KinectGrabberService::Request& request,definitions::KinectGrabberService::Response& response)
    {
       
       std::stringstream ss;
       ss << root_file_;
       ss << "frame";
       ss << id; ss << ".pcd";
       file_name = ss.str();
       REC_TIMES_KINECT  = true;
       
       response.path_to_pclfile = file_name;
               
       return true;
    }

    void callback_kinect(const sensor_msgs::PointCloud2ConstPtr & input)
    {
      if(REC_TIMES_KINECT){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points_ (new pcl::PointCloud<pcl::PointXYZ>());
    	xyz_points_->height = input->height;
    	xyz_points_->width = input->width;
    	xyz_points_->points.resize(xyz_points_->width*xyz_points_->height); 
    	pcl::fromROSMsg( *input, *xyz_points_);
         
        pub_object_point_clouds_.publish(*xyz_points_);
        
        id++;
        std_msgs::String msg;
        msg.data = file_name;

        pcl::io::savePCDFileASCII (file_name,*xyz_points_);
        usleep(500);
        REC_TIMES_KINECT=false;
        }  
    }
     
    // constructor
    grabkinect(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {   
        root_file_ = "/tmp/";
        REC_TIMES_KINECT = false;
        id = 0;
        // advertise service
        srv_estimate_poses_ = nh_.advertiseService(nh_.resolveName("/kinect_grabber/kinect_grab_name"), &grabkinect::advertise_frame, this);
        ROS_INFO("Wait for /camera/depth/points to publish (openni_launch) ");
	subKinect = nh_.subscribe ("/camera/depth/points", 500, &grabkinect::callback_kinect, this);
        pub_object_point_clouds_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(nh_.resolveName("/kinect_grabber/scene_cloud"), 1);
       
    }

    //! Empty stub
    ~grabkinect() {}
};
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_grabber");
    ros::NodeHandle nh;

    kinect_grabber::grabkinect node(nh);
    ROS_INFO("Kinect Grabber node to get kinect pointcloud and save to /tmp directory ...");
    while(ros::ok())
    {
        ros::Rate r(30);
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

