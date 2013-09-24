#include <string>
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL from ROS headers
// #include <pcl_ros/transforms.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>

// use "" for local headers
#include "I_SegmentedObjects.h"
#include "ParametersPoseEstimation.h"

// ROS generated headers
// typically these ones are generated at compilation time, 
// for instance, for a service called ClassParameterSetting.srv 
// within pacman_pose_estimation, we need to include this 
// #include "pacman_pose_estimation/ClassParameterSetting.h" 
//#include "definitions/PoseEstimation.h" 

namespace pacman_pose_estimation {

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

  public:

    // callback functions
    bool estimatePoses(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // constructor
    PoseEstimator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {   
        // advertise service
        srv_estimate_poses_ = nh_.advertiseService(nh_.resolveName("/estimate_object_poses"), &PoseEstimator::estimatePoses, this);
    }

    //! Empty stub
    ~PoseEstimator() {}

};

// this function is called when service_name_you_want_to_advertise is advertise
//bool PoseEstimator::estimatePoses(PoseEstimation::Request &request, PoseEstimation::Response &response)
bool PoseEstimator::estimatePoses(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{   
    string filename; 
    filename = "/home/pacman/poseEstimation/parametersFiles/config.txt";
    ParametersPoseEstimation params(filename);
    I_SegmentedObjects objects;
    params.recognizePose(objects);

    std::vector<string> names = objects.getObjects();

    std::cout << names.size() << std::endl;

   

    return true;
}

} // namespace pacman_pose_estimation

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation_node");
    ros::NodeHandle nh;

    pacman_pose_estimation::PoseEstimator node(nh);

    while(ros::ok())
    {
        //node.doSomething();
        ros::spinOnce();
    }

    return 0;
}