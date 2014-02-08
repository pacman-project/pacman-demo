// The structure is more or less the following:
//
// 1) Start the main loop
// 2) Calls the pose_estimation service
// Input: point cloud of the scene
// Output: ordered list of detected objects
// 3) Calls the grasp_planning service
// Input: ordered list of detected objects
// Output: list of grasps for the first object of the scene
// 4) Calls the trajectory_planning service
// Input: ordered list of detected objects
// lists of detected grasps for the first object of the scene
// Output: trajectory for both the arm and the hand
// 5) Calls the trajectory_execution service
// Input: trajectory for both the arm and the hand
// Output: success/failure//

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <vector>

// for the messages used in the services
#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"
#include "definitions/ObjectCloudReader.h"

//hand trajectory sdh messages

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//Mimicing the hardware grasp_types, replace later by inbuild message
std::vector<float> getTargetAnglesFromGraspType(std::string grasp_type, float close_ratio) {
 
std::vector<float> hand_pose;
int grasp;
if(grasp_type=="cylindrical")
grasp=1;
else if(grasp_type=="parallel")
grasp=2;
else if(grasp_type=="centrical")
grasp=3;
else if(grasp_type=="spherical")
grasp=4;
else if(grasp_type=="rim_open")
grasp=5;
else if(grasp_type=="rim_close")
grasp=6;
 
if (close_ratio > 1.0)
close_ratio = 1.0;
else if (close_ratio < 0.0)
close_ratio = 0.0;
 
switch(grasp) {
case 1: // Cylindrical
hand_pose.push_back(0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
hand_pose.push_back((-30+close_ratio*30)* M_PI/180.0);
hand_pose.push_back((30+close_ratio*35)* M_PI/180.0);
return hand_pose;
break;
case 2: // Parallel
hand_pose.push_back(0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((-75.+close_ratio*82.)* M_PI/180.0);
hand_pose.push_back((75.-close_ratio*82.)* M_PI/180.0);
return hand_pose;
 
case 3: // Centrical
hand_pose.push_back((60)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
hand_pose.push_back((-75+close_ratio*82)* M_PI/180.0);
hand_pose.push_back((75-close_ratio*82)* M_PI/180.0);
return hand_pose;
 
case 4: // Spherical
hand_pose.push_back((60)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
hand_pose.push_back((-40+close_ratio*25)* M_PI/180.0);
hand_pose.push_back((40+close_ratio*15)* M_PI/180.0);
return hand_pose;

case 5: //Parallel grasp for rim open
hand_pose.push_back(0.);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
hand_pose.push_back(-0.8);
hand_pose.push_back(0.8);
return hand_pose;

case 6: // parallel grasp for rim close
hand_pose = std::vector<float>(7,0.15);
return hand_pose;
default:
hand_pose.push_back(0);
hand_pose.push_back(0);
hand_pose.push_back(0);
hand_pose.push_back(0);
hand_pose.push_back(0);
hand_pose.push_back(0);
hand_pose.push_back(0);
return hand_pose;
}
}


class DemoSimple
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::ServiceClient pose_client;
    ros::ServiceClient grasp_client;
    ros::ServiceClient trajectory_planner_client;
    ros::ServiceClient trajectory_execution_client;
    ros::Publisher sdh_joint_angles_pub;
    ros::ServiceClient reader_client;
    
    //Stuff
    std::string pose_estimation_service_name;
    std::string grasp_service_name;
    std::string trajectory_planning_service_name;
    std::string trajectory_execution_service_name;
    std::string object_reader_service_name;
    
    //Variables messages
    bool status_robot, status_ros;
    
    std::vector<std::string> joint_names_str;
    
    std::vector<definitions::Object> my_detected_objects;
    std::vector<definitions::Grasp> my_calculated_grasp;
    std::vector<definitions::Trajectory> my_calculated_trajectory;
    
    std::vector<float> my_current_sdh_joints_pregrasp;
    std::vector<float> my_current_sdh_joints;
    
    //Service variables
    definitions::PoseEstimation estimation_srv;
    definitions::GraspPlanning grasp_planning_srv;
    definitions::TrajectoryPlanning trajectory_planning_srv;
    definitions::TrajectoryExecution trajectory_execution_srv;
    definitions::ObjectCloudReader reader_srv;

  public:

    void initRobot(); //e.g checking joint states cmd mode and hand initialization
    
    void goToStartPos();
    void reconstruct_scene();
    
    int doPoseEstimation();
    
    void planGrasps();
    
    void userCheck();
    
    bool executeMovement(bool pre_grasp);
    
    void goToPlacingPos();
    
    void publishSdhJoints(std::vector<float> positions);
    std::vector<double> euler_to_quaternion(double roll,double pitch,double yaw);
    
    // constructor
    DemoSimple(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        pose_estimation_service_name = "/pose_estimation_uibk/estimate_poses";
        grasp_service_name = "/grasp_planner_srv";
        trajectory_planning_service_name = "/trajectory_planner_srv";
        trajectory_execution_service_name = "/trajectory_execution_srv";
        object_reader_service_name = "/object_reader";

        pose_client = nh_.serviceClient<definitions::PoseEstimation>(pose_estimation_service_name);
        grasp_client = nh_.serviceClient<definitions::GraspPlanning>(grasp_service_name);
        trajectory_planner_client = nh_.serviceClient<definitions::TrajectoryPlanning>(trajectory_planning_service_name);
        
        trajectory_execution_client = nh_.serviceClient<definitions::TrajectoryExecution>(trajectory_execution_service_name);
        //Grasp execution
        //sdh_joint_angles_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/real/right_sdh/follow_joint_trajectory/goal", 1);
	sdh_joint_angles_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/simulation/right_sdh/follow_joint_trajectory/goal", 1);
        reader_client = nh_.serviceClient<definitions::ObjectCloudReader>(object_reader_service_name);

        my_current_sdh_joints.resize(7);
        my_current_sdh_joints_pregrasp.resize(7);
	
	joint_names_str.push_back("right_sdh_knuckle_joint");
	joint_names_str.push_back("right_sdh_thumb_2_joint");
	joint_names_str.push_back("right_sdh_thumb_3_joint");
	joint_names_str.push_back("right_sdh_finger_12_joint");
	joint_names_str.push_back("right_sdh_finger_13_joint");
	joint_names_str.push_back("right_sdh_finger_22_joint");
	joint_names_str.push_back("right_sdh_finger_23_joint");
    }

    //! Empty stub
    ~DemoSimple() {}

};

std::vector<double> DemoSimple::euler_to_quaternion(double roll,double pitch,double yaw)
{
  std::vector<double> q(4);
  q[0] = (cos(roll/2) * cos(pitch/2) * cos(yaw/2))
         + (sin(roll/2) * sin(pitch/2) * sin(yaw/2));
  q[1] = (sin(roll/2) * cos(pitch/2) * cos(yaw/2))
         - (cos(roll/2) * sin(pitch/2) * sin(yaw/2));	 
  q[2] = (cos(roll/2) * sin(pitch/2) * cos(yaw/2))
         + (sin(roll/2) * cos(pitch/2) * sin(yaw/2));	 
  q[3] = (cos(roll/2) * cos(pitch/2) * sin(yaw/2))
         - (sin(roll/2) * sin(pitch/2) * cos(yaw/2));	 	 
	 
  return q;
}

void DemoSimple::publishSdhJoints(std::vector<float> positions) 
{        
        
                control_msgs::FollowJointTrajectoryActionGoal actionGoal;
                control_msgs::FollowJointTrajectoryGoal& goal = actionGoal.goal;
                trajectory_msgs::JointTrajectory& traj = goal.trajectory;
		
                traj.joint_names = joint_names_str;
                trajectory_msgs::JointTrajectoryPoint point;
                //conversion to double
		point.positions.resize(positions.size());
		for (int i=0; i < positions.size(); i++){
                point.positions[i] = positions.at(i);
		std::cout << "Joint Position  " << i << " "  << positions.at(i) << std::flush;
		}
		std::cout << "Joint Position" << point << std::flush;
		
		
		std::vector<trajectory_msgs::JointTrajectoryPoint> pointtrajectory;
		pointtrajectory.push_back(point);
                traj.points = pointtrajectory;
                sdh_joint_angles_pub.publish(actionGoal);
                //Secure time for finishing of movement, 2 s
                usleep(2000*1000);
        

        ROS_INFO("FollowJointTrajectoryActionGoals published");
        usleep(1000*1000);
        
}

void DemoSimple::goToStartPos()
{
  
  geometry_msgs::Pose robotpose;
  
  definitions::Grasp current_trajectory;
  current_trajectory.grasp_trajectory.resize(1);  
  
  double roll = 0; 
  double pitch = 0;
  double yaw = -1;
  
  std::vector<double> q = euler_to_quaternion(roll,pitch,yaw);
  
  /*robotpose.position.x = 0.35;
  robotpose.position.y = 0.23;
  robotpose.position.z = 0.54;

  robotpose.orientation.x = 0.755872;
  robotpose.orientation.y = -0.612878;
  robotpose.orientation.z = -0.0464803;
  robotpose.orientation.w = 0.22556;*/

  robotpose.position.x = 0.26;
  robotpose.position.y = 0.20;
  robotpose.position.z = 0.65;
  
  robotpose.orientation.x = 0.755872;
  robotpose.orientation.y = -0.612878;
  robotpose.orientation.z = -0.0464803;
  robotpose.orientation.w = 0.22556;

  /*robotpose.orientation.x = q[0];
  robotpose.orientation.y = q[1];
  robotpose.orientation.z = q[2];
  robotpose.orientation.w = q[3];*/
  
  std::cout << "Initial position: " << robotpose << std::endl;
  current_trajectory.grasp_trajectory[0].wrist_pose.pose = robotpose;
  
  my_calculated_grasp.clear();
  my_calculated_grasp.push_back(current_trajectory);
  
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
  std::vector<definitions::Object> noObject;
  
  trajectory_planning_srv.request.object_list = noObject;
  trajectory_planning_srv.request.object_id = 0;
  
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else	
  {
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
    
    ROS_INFO("Executing Trajectory");
    
    //User input
    char a;
    std::cout << "Check if trajectory is ok (y/n)" << std::endl;
    std::cin >> a;
    
    if (a=='y')
    {    
    //Arm movement
    std::cout << "Executing arm trajectory" << std::endl;
    std::cout << "#of found trajectories: "  << my_calculated_trajectory.size() << std::endl;
    my_calculated_trajectory[0].trajectory_id=0;
    trajectory_execution_srv.request.trajectory = my_calculated_trajectory; //execute 0 trajectory id

      if( !trajectory_execution_client.call(trajectory_execution_srv) )
      {
	      ROS_INFO("trajectory planner service call failed.");
      }
      else
      {
	      ROS_INFO("trajectory execution call succeeded");
      }
        
    }
    else
    {
    std::cout << "Trajectory not valid - restart" << std::endl;
    }
  }
  publishSdhJoints(my_current_sdh_joints_pregrasp);
}

int DemoSimple::doPoseEstimation()
{
   if( !pose_client.call(estimation_srv))
   {
     ROS_INFO("pose estimation service failed. wait...");
     ros::Duration(0.5).sleep();
   }
   my_detected_objects = estimation_srv.response.detected_objects;
   
   std::cout<< "nr of objects: "<<my_detected_objects.size()<<std::endl;
  /* reader_srv.request.detected_objects = my_detected_objects;
   reader_srv.request.object_id = 0;*/

   return my_detected_objects.size();
}

void DemoSimple::reconstruct_scene()
{ 
   if( !reader_client.call(reader_srv) ) 
   {
     ROS_INFO("object reader service failed. wait...");
     ros::Duration(0.5).sleep();     
   }
   ROS_INFO("object reader done");
}
void DemoSimple::planGrasps()
{
  grasp_planning_srv.request.ordered_objects = my_detected_objects;
  grasp_planning_srv.request.object_id = 0;
  if( !grasp_client.call(grasp_planning_srv) )
  {
    ROS_INFO("grasp_planner service call failed. wait...");
    ros::Duration(0.5).sleep();
  }
  if ( grasp_planning_srv.response.result == grasp_planning_srv.response.SUCCESS)
  {
    ROS_INFO("Grasp Planning OK - assigning last wrist pose for arm trajectory planning");
    
// my_calculated_grasp = grasp_planning_srv.response.grasp_list;
    
    //Removing all but the best grasp and only keeping last trajectory
    my_calculated_grasp.clear();
    std::cout << "grasp planner response size: " << grasp_planning_srv.response.grasp_list.size() << std::endl;
    my_calculated_grasp.push_back(grasp_planning_srv.response.grasp_list[0]);
    //Assigning pre-grasp pose
    std::cout << "Size of sdh joints, check conversion to double and ordering, maybe need to reorder " << my_calculated_grasp[0].grasp_trajectory[0].joints.size() << std::endl;
    
    std::cout << "Current goal:" << my_calculated_grasp[0].grasp_trajectory[2].wrist_pose.pose << std::endl;
    
    for (int i = 0; i < my_calculated_grasp[0].grasp_trajectory[0].joints.size(); i++){
    std::cout << "SDH Joint Nr. " << i << " Value:" << my_calculated_grasp[0].grasp_trajectory[0].joints[i] <<  std::endl;
    
    }
    
//     my_current_sdh_joints_pregrasp = my_calculated_grasp[0].grasp_trajectory[0].joints; //size should be checked
    my_current_sdh_joints_pregrasp = getTargetAnglesFromGraspType("rim_open", 0.3);//size should be checked
       
   // my_calculated_grasp[0].grasp_trajectory.erase(my_calculated_grasp[0].grasp_trajectory.begin() , my_calculated_grasp[0].grasp_trajectory.begin()+ 2);
    
//     my_current_sdh_joints = my_calculated_grasp[0].grasp_trajectory[0].joints;
    my_current_sdh_joints = getTargetAnglesFromGraspType("rim_close", 1.0);
    
    
    
   }
   else
   {
     ROS_INFO("grasp_planner did not succeed.");
   }
/*   for (int i =0; i < grasp_planning_srv.response.grasp_list.size(); i++)
   {
//      std::cout << "Grasp Nr." << i << std::endl;
//      std::cout << grasp_planning_srv.response.grasp_list[i].grasp_trajectory[2].wrist_pose << std::endl;
  }*/

}

bool DemoSimple::executeMovement(bool pre_grasp)
{
  bool succeed = false;
  std::cout << "first point: "<< my_calculated_grasp[0].grasp_trajectory[0].wrist_pose << std::endl;
  std::cout << "second point: "<< my_calculated_grasp[0].grasp_trajectory[1].wrist_pose << std::endl;
  std::cout << "third point: "<< my_calculated_grasp[0].grasp_trajectory[2].wrist_pose << std::endl;
  if( !pre_grasp ) {
    /*reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = 0;
    reconstruct_scene();*/

    my_calculated_grasp[0].grasp_trajectory[0] = my_calculated_grasp[0].grasp_trajectory[2];
    my_calculated_grasp[0].grasp_trajectory.erase(my_calculated_grasp[0].grasp_trajectory.begin()+1,my_calculated_grasp[0].grasp_trajectory.begin()+2);
  }
  /*else
  {
    reader_srv.request.detected_objects = my_detected_objects;
    reader_srv.request.object_id = -1;
    reconstruct_scene();
  }*/
  //Only doing stuff for 0th object e.g = grasp_planning_srv.request.object_id = 0;
  //querying trajectory plan
// trajectory_planning_srv.request.ordered_grasp = grasp_planning_srv.response.grasp_list;
  trajectory_planning_srv.request.ordered_grasp = my_calculated_grasp;
  trajectory_planning_srv.request.object_list = estimation_srv.response.detected_objects;
  trajectory_planning_srv.request.object_id = 0;
  if( !trajectory_planner_client.call(trajectory_planning_srv) )
  {
    ROS_INFO("trajectory planner service call failed.");
  }
  else	
  {
    ROS_INFO("trajectory planner call succeeded");
    my_calculated_trajectory = trajectory_planning_srv.response.trajectory;
    ROS_INFO("number of found trajectories are: %d",(int)my_calculated_trajectory.size());
    
    int nr_found_trajectories = my_calculated_trajectory.size();
    
    if(nr_found_trajectories>0) 
    {
     ROS_INFO("Executing Pick and place");
    
     //User input
    char a;
    std::cout << "Check if trajectory is ok (y/n)" << std::endl;
    std::cin >> a;
    
    if (a=='y')
    {    
      
      //Hand Pre-grasp
      publishSdhJoints(my_current_sdh_joints_pregrasp);
      //Arm movement
      std::cout << "Executing arm trajectory" << std::endl;
      my_calculated_trajectory[0].trajectory_id=0;
      trajectory_execution_srv.request.trajectory = my_calculated_trajectory; //execute 0 trajectory id
    
      if( !trajectory_execution_client.call(trajectory_execution_srv) )
      {
	      ROS_INFO("trajectory planner service call failed.");
      }
      else
      {
	// succeed = true;
	      ROS_INFO("trajectory execution call succeeded");
	      if( !pre_grasp )
	        publishSdhJoints(my_current_sdh_joints);	      
      }
    
    //GraspPlanning
    
   
    
    //TODO: Move to Place positions, to be done after testing, query trajectory planning service again.
    
    }
    else if (a=='n')
    {
    std::cout << "Trajectory not valid - restart" << std::endl;
    
    planGrasps();
    executeMovement(pre_grasp);
    
    }
    else
      return false;
    }
    else
    {
      char a;
      std::cout << "Stop execution by pressing 's', otherwise press another key" << std::endl;
      std::cin >> a;
      if( a == 's' )
	return false;
      else
      {
	 std::cout << "Did not found any trajectory for the given grasp, replan another grasp - restart" << std::endl;
	 planGrasps();
         executeMovement(pre_grasp);
      }
	
    }
  }
  
  succeed = true; 
  return succeed;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demoSimple");
  ros::NodeHandle nh;
  
  // Set the freq of the main loop, because now we have a socket client
  ros::Rate loop_rate(15);
  ros::Timer delay();
  
  DemoSimple demo(nh);
  
  demo.goToStartPos();
  
  int nr = demo.doPoseEstimation();
  while( nr > 0)
  {
   demo.planGrasps();
   bool pre_grasp = true;
   std::cout <<  "to plan pre-grasp !!!!" << std::endl;
   bool success= demo.executeMovement(pre_grasp);
   std::cout << "after pre-grasp: " << success << std::endl;
   if( success ) 
   {
     std::cout << "to do grasp!!!" << std::endl; 
     pre_grasp = false;
     demo.executeMovement(pre_grasp);
   }
  
   demo.goToStartPos();
   if( nr > 1 )
     nr = demo.doPoseEstimation();
  }
  return 0;
}
