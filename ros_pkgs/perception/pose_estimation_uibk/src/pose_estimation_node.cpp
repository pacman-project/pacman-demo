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

// PCL 1.7 headers
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

// use "" for local headers
#include "I_SegmentedObjects.h"
#include "ParametersPoseEstimation.h"

// ROS generated headers
// typically these ones are generated at compilation time, 
// for instance, for a service called ClassParameterSetting.srv 
// within pose_estimation_uibk, we need to include this 
// #include "pose_estimation_uibk/ClassParameterSetting.h" 
#include "definitions/PoseEstimation.h" 

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

  public:

    // callback functions
    bool estimatePoses(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response);

    //Utility functions
    void poseEigenToMsg(const Eigen::Affine3d&, geometry_msgs::Pose&);

    // constructor
    PoseEstimator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {   
        // advertise service
        srv_estimate_poses_ = nh_.advertiseService(nh_.resolveName("/estimate_poses"), &PoseEstimator::estimatePoses, this);
    }

    //! Empty stub
    ~PoseEstimator() {}

};

bool PoseEstimator::estimatePoses(definitions::PoseEstimation::Request& request, definitions::PoseEstimation::Response& response)
{   
    ROS_INFO("Pose estimation service has been called...");
    
    ROS_INFO("Initializing...");
    string filename, dirRecognizedObjects; 
    filename = "/home/pacman/CODE/poseEstimation/parametersFiles/config_ROS.txt";
    dirRecognizedObjects = "/home/pacman/CODE/poseEstimation/data/recognizedObjects";
    
    ParametersPoseEstimation params(filename);
    I_SegmentedObjects objects(dirRecognizedObjects);

    ROS_INFO("Recognizing poses...");
    params.recognizePose(objects);
    
    // getting the result of pose estimation
    std::vector<string> names = objects.getObjects();
    std::vector<int> orderedList = objects.getIdsObjects(objects.getHeightList());
    boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = objects.getTransforms ();
    
    std::vector<definitions::Object> detected_objects(names.size());

    int j;
    //Transform these types to ros message types
    for (int i = 0; i < names.size (); i++)
    {     
        j = orderedList[i]; 
        std::cout << j << std::endl;

        Eigen::Matrix4f transform_matrix = transforms->at(j);
        Eigen::Matrix4d md(transform_matrix.cast<double>());
        Eigen::Affine3d e = Eigen::Affine3d(md);

        geometry_msgs::Pose current_pose;
        poseEigenToMsg(e, current_pose);

        definitions::Object object;
        object.pose = current_pose;
        object.name.data = names.at(j);
        detected_objects[i] = object;
    }

    response.detected_objects = detected_objects;
         
    ROS_INFO("Pose estimation service finisihed, and ready for another service requests...");
    return true;
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