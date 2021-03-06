#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <string>
#include <dirent.h>
#include <fstream>

#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <definitions/PoseEstimation.h>
#include <definitions/GraspPlanning.h>
#include <definitions/GraspList.h>

using namespace std;
enum Grasps 
{
  cylindrical,
  parallel,
  centrical,
  spherical,
  rim_open,
  rim_close,
  rim_pre_grasp
};

struct handJoints
{
  float knuckle;
  float finger_12;
  float finger_13;
  float finger_22;
  float finger_23;
  float thumb_2;
  float thumb_3;
};
namespace grasp_planner_uibk
{

class GraspPlanner
{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::ServiceServer srv_grasp_planner_;
    vector<definitions::Grasp> estimated_grasps;
    ros::Publisher vis_pub;
    string path_to_dir;
    string root;
    double offset_x,offset_y;
    vector<double> grasp_score_;
    string arm_;
    ros::Publisher pub_grasps_;
    vector<Grasps> grasp_type_;
    Grasps cur_grasp_type_;
  
public:
    
    GraspPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      
      srv_grasp_planner_ = nh_.advertiseService(nh_.resolveName("/grasp_planner_srv"),&GraspPlanner::extractGrasp, this);
      
    //offset_x = -0.04;
      offset_x = -0.02;
      offset_y = 0.03;
       
      offset_x = 0;
      offset_y = 0;
      vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("gripper", 1 );

      nh_.param<std::string>("path_to_dir", root, "");
      pub_grasps_ = nh_.advertise<definitions::GraspList>(nh_.resolveName("/grasp_planner_uibk/grasps"), 1);
    }
    
    ~GraspPlanner()
    {}
    
    bool extractGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res);
    
    vector<geometry_msgs::PoseStamped> searchGraspFile(vector<string> path_to_obj_dir,bool pre_grasp);
    
    geometry_msgs::Pose searchPoseFile(string obj_id);
    
    vector<string> giveAllFiles(string obj_id);
    
    // ** get transformation between given point cloud and the one which we learned in grasp learning
    Eigen::Vector3f get_transformation(geometry_msgs::Pose ref_obj,geometry_msgs::Pose cur_obj);
    
    std::vector<double> euler_to_quaternion(vector<double> vals);
    
    void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::PoseStamped &m);
    
    geometry_msgs::PoseStamped find_transformation(geometry_msgs::Pose ref_obj,geometry_msgs::Pose cur_obj,geometry_msgs::PoseStamped cur_grasp);
    void visualize_gripper(geometry_msgs::PoseStamped gripper_pre_pose,geometry_msgs::PoseStamped gripper_pose,int id);
    void set_arm(string arm);
    vector<float> getTargetAnglesFromGraspType(Grasps grasp_type, float close_ratio);
    geometry_msgs::PoseStamped transform_frame(geometry_msgs::PoseStamped pose);

    geometry_msgs::PoseStamped find_mid_point(geometry_msgs::PoseStamped pre_grasp,geometry_msgs::PoseStamped grasp,vector<float> &mid_grasp_joint,double ratio,double hand_ratio);
};

geometry_msgs::PoseStamped GraspPlanner::find_mid_point(geometry_msgs::PoseStamped pre_grasp,geometry_msgs::PoseStamped grasp,vector<float> &mid_grasp_joint,double ratio,double hand_ratio)
{
 geometry_msgs::PoseStamped mid_point;
 //double ratio = 0.8;
 double mid_x = pre_grasp.pose.position.x + ( grasp.pose.position.x - pre_grasp.pose.position.x ) * ( ratio - 0.);
 double mid_y = pre_grasp.pose.position.y + ( grasp.pose.position.y - pre_grasp.pose.position.y ) * ( ratio - 0.);
 double mid_z = pre_grasp.pose.position.z + ( grasp.pose.position.z - pre_grasp.pose.position.z ) * ( ratio - 0.);
 mid_point.pose.position.x = mid_x;
 mid_point.pose.position.y = mid_y;
 mid_point.pose.position.z = mid_z;
 mid_point.pose.orientation = grasp.pose.orientation;

 /*vector<float> pre_grasp_joints = getTargetAnglesFromGraspType(rim_pre_grasp,1.0);
 vector<float> grasp_joints = getTargetAnglesFromGraspType(rim_close,1.0); 
*/
 vector<float> pre_grasp_joints;
 vector<float> grasp_joints;
 if( cur_grasp_type_ == cylindrical )
 {
   pre_grasp_joints = getTargetAnglesFromGraspType(cylindrical,0.1);
   grasp_joints = getTargetAnglesFromGraspType(cylindrical,1.0); 
 }
 else if( cur_grasp_type_ == spherical )
 {
   pre_grasp_joints = getTargetAnglesFromGraspType(spherical,0.1);
   grasp_joints = getTargetAnglesFromGraspType(spherical,1.0); 
 } 
 else
 {
    pre_grasp_joints = getTargetAnglesFromGraspType(rim_pre_grasp,1.0);
    grasp_joints = getTargetAnglesFromGraspType(rim_close,1.0);
 }


 mid_grasp_joint.resize(pre_grasp_joints.size(),0);
 //double hand_ratio = 0.2;
 for( size_t i = 0; i < mid_grasp_joint.size(); i++ )
   mid_grasp_joint[i] = pre_grasp_joints[i] + ( grasp_joints[i] - pre_grasp_joints[i] ) * ( hand_ratio - 0.); 

  return mid_point;
}

vector<float> GraspPlanner::getTargetAnglesFromGraspType(Grasps grasp_type, float close_ratio) 
{
  
  std::vector<float> hand_pose(7,0);
  handJoints hand_joints; 
  if (close_ratio > 1.0)
    close_ratio = 1.0;
  else if (close_ratio < 0.0)
    close_ratio = 0.0;
 
  // joints angle based on knuckle, finger_1, finger_2, thumb
  switch(grasp_type) {
    case cylindrical: 
    hand_joints.knuckle = 0;
    hand_joints.thumb_2 = (-30+close_ratio*30)* M_PI/180.0;
    hand_joints.thumb_3 = (30+close_ratio*35)* M_PI/180.0;
    hand_joints.finger_12 = (-30+close_ratio*30)* M_PI/180.0;
    hand_joints.finger_13 = (30+close_ratio*35)* M_PI/180.0;
    hand_joints.finger_22 = (-30+close_ratio*30)* M_PI/180.0;
    hand_joints.finger_23 = (30+close_ratio*35)* M_PI/180.0;
    break;

    case parallel: 
    hand_joints.knuckle = 0;
    hand_joints.thumb_2  = (-75.+close_ratio*82.)* M_PI/180.0;
    hand_joints.thumb_3 = (75.-close_ratio*82.)* M_PI/180.0;
    hand_joints.finger_12 = (-75.+close_ratio*82.)* M_PI/180.0;
    hand_joints.finger_13 = (75.-close_ratio*82.)* M_PI/180.0;
    hand_joints.finger_22 = (-75.+close_ratio*82.)* M_PI/180.0;
    hand_joints.finger_23 = (75.-close_ratio*82.)* M_PI/180.0;
    break;

    case centrical: 
    hand_joints.knuckle = (60)* M_PI/180.0;
    hand_joints.thumb_2 = (-75+close_ratio*82)* M_PI/180.0;
    hand_joints.thumb_3 = (75-close_ratio*82)* M_PI/180.0;
    hand_joints.finger_12 = (-75+close_ratio*82)* M_PI/180.0;
    hand_joints.finger_13 = (75-close_ratio*82)* M_PI/180.0;
    hand_joints.finger_22 = (-75+close_ratio*82)* M_PI/180.0;
    hand_joints.finger_23 = (75-close_ratio*82)* M_PI/180.0;
    break;

    case spherical: 
    hand_joints.knuckle = (60)* M_PI/180.0;
    hand_joints.thumb_2 = (-40+close_ratio*25)* M_PI/180.0;
    hand_joints.thumb_3 = (40+close_ratio*15)* M_PI/180.0;
    hand_joints.finger_12 = (-40+close_ratio*25)* M_PI/180.0;
    hand_joints.finger_13 = (40+close_ratio*15)* M_PI/180.0;
    hand_joints.finger_22 = (-40+close_ratio*25)* M_PI/180.0;
    hand_joints.finger_23 = (40+close_ratio*15)* M_PI/180.0;
    break;

    case rim_open: 
    hand_joints.knuckle = 0;
    hand_joints.thumb_2 = -0.5;
    hand_joints.thumb_3 = 0.5;
    hand_joints.finger_12 = -0.5;
    hand_joints.finger_13 = 0.5;
    hand_joints.finger_22 = -0.5;
    hand_joints.finger_23 = 0.5;
    break;

    case rim_close: 
    hand_joints.knuckle = 0.15;
    hand_joints.thumb_2 = 0.15;
    hand_joints.thumb_3 = 0.15;
    hand_joints.finger_12 = 0.15;
    hand_joints.finger_13 = 0.15;
    hand_joints.finger_22 = 0.15;
    hand_joints.finger_23 = 0.15;
    break;

    case rim_pre_grasp:
    hand_joints.knuckle = 0.0;
    hand_joints.thumb_2 = -0.2;
    hand_joints.thumb_3 = 0.1;
    hand_joints.finger_12 = -0.25;
    hand_joints.finger_13 = 0.15;
    hand_joints.finger_22 = -0.25;
    hand_joints.finger_23 = 0.15;
    break;

    default:
    hand_joints.knuckle = 0.0;
    hand_joints.thumb_2 = -0.8;
    hand_joints.thumb_3 = 0.8;
    hand_joints.finger_12 = -0.8;
    hand_joints.finger_13 = 0.8;
    hand_joints.finger_22 = -0.8;
    hand_joints.finger_23 = 0.8;
    break;
  }

   hand_pose[0] = hand_joints.knuckle;
   hand_pose[1] = hand_joints.finger_12;
   hand_pose[2] = hand_joints.finger_13;
   hand_pose[3] = hand_joints.finger_22;
   hand_pose[4] = hand_joints.finger_23;
   hand_pose[5] = hand_joints.thumb_2;
   hand_pose[6] = hand_joints.thumb_3;

  return hand_pose;
}
void GraspPlanner::set_arm(string arm)
{
  if( arm == "right" )
    path_to_dir = root + "/grasps-models-multi-grasps_right/";
  else if(arm == "left")
    path_to_dir = root + "/grasps-models-multi-grasps_left/";
  arm_ = arm;
  //path_to_dir = root + "/grasps-models-multi-grasps_right/";
  cout << "path to directory is: " << path_to_dir << endl;
}

void GraspPlanner::visualize_gripper(geometry_msgs::PoseStamped gripper_pre_pose,geometry_msgs::PoseStamped gripper_pose,int grasp_id)
{
  visualization_msgs::MarkerArray markers;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_link";
  marker.header.stamp = ros::Time();
  stringstream ss;
  ss << "gripper_ns_" << grasp_id;
  marker.ns = ss.str();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  
  int id = -1;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.id = ++id;
  marker.pose.position = gripper_pose.pose.position;
  marker.pose.orientation = gripper_pose.pose.orientation;
  marker.color.a = 0.5;
  marker.color.r = 0.8;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.mesh_resource = "package://schunk_description/meshes/sdh/palm.stl";
  markers.markers.push_back(marker);
 
  cout << "gripper pose is : "  << gripper_pose.pose << endl;
  Eigen::Quaternionf quat(gripper_pose.pose.orientation.w,gripper_pose.pose.orientation.x,gripper_pose.pose.orientation.y,gripper_pose.pose.orientation.z);
  Eigen::Matrix3f rot = quat.toRotationMatrix();
  Eigen::Matrix4f trans_g;
  trans_g.block<3,3>(0,0) = rot;
  trans_g(0,3) = gripper_pose.pose.position.x; trans_g(1,3) = gripper_pose.pose.position.y; trans_g(2,3) = gripper_pose.pose.position.z; 
  trans_g(3,0) = 0; trans_g(3,1) = 0; trans_g(3,2) = 0; trans_g(3,3) = 1;
    
  Eigen::Matrix4f trans_1;
  trans_1 << 
    1,0,0,-0.05,
    0,1,0,-0.02,
    0,0,1,-0.12,
    0,0,0,1;
  
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.id = ++id; 
  Eigen::Matrix4f trans = trans_g * trans_1.inverse();
  Eigen::Matrix3f rot_mat = trans.block<3,3>(0,0);
  Eigen::Quaternionf quat_(rot_mat);
  
  marker.pose.position.x = trans(0,3);
  marker.pose.position.y = trans(1,3);
  marker.pose.position.z = trans(2,3);
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;
  markers.markers.push_back(marker);
 
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.id = ++id; 
  Eigen::Matrix4f trans_2;
  trans_2 << 
    1,0,0,-0.03,
    0,1,0,0.04,
    0,0,1,-0.12,
    0,0,0,1;  
  Eigen::Matrix4f trans_2_ = trans_g * trans_2.inverse();
  Eigen::Matrix3f rot_mat_2 = trans_2_.block<3,3>(0,0);
  Eigen::Quaternionf quat_2(rot_mat_2);
  
  marker.pose.position.x = trans_2_(0,3);
  marker.pose.position.y = trans_2_(1,3);
  marker.pose.position.z = trans_2_(2,3);  
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;
  markers.markers.push_back(marker);
  
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.id = ++id; 
  Eigen::Matrix4f trans_3;
  trans_3 << 
    1,0,0,0.04,
    0,1,0,-0.02,
    0,0,1,-0.12,
    0,0,0,1;   
  Eigen::Matrix4f trans_3_ = trans_g * trans_3.inverse();
  Eigen::Matrix3f rot_mat_3 = trans_3_.block<3,3>(0,0);
  Eigen::Quaternionf quat_3(rot_mat_3);
  
  marker.pose.position.x = trans_3_(0,3);
  marker.pose.position.y = trans_3_(1,3);
  marker.pose.position.z = trans_3_(2,3);  
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;
  markers.markers.push_back(marker); 
  vis_pub.publish( markers );
}

std::vector<double> GraspPlanner::euler_to_quaternion(vector<double> vals)
{
  //express values in mm
  vals[0] /= 1000.; vals[1] /= 1000.; vals[2] /= 1000.;
  
  //convert angle values to radians   
  vals[3] = vals[3] * M_PI/180.; vals[4] = vals[4] * M_PI/180.; vals[5] = vals[5] * M_PI/180.;

  vector<double> quat_vec;
  Eigen::Matrix3f rot;
  
  rot = Eigen::AngleAxisf(vals[3], Eigen::Vector3f::UnitZ())* Eigen::AngleAxisf(vals[4], Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(vals[5], Eigen::Vector3f::UnitX()); //euler angles

  /*double dx,dy,dz;
  dx = M_PI / 180.; dy = M_PI / 180.; dz = M_PI/(2 * 180.);
  Eigen::Matrix3f rot_z;
  rot_z = Eigen::AngleAxisf(dz, Eigen::Vector3f::UnitZ())* Eigen::AngleAxisf(dy, Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(dx, Eigen::Vector3f::UnitX());
  Eigen::Matrix3f rot = rot_z * rot_ref;*/
  Eigen::Matrix4f posematrix;
  
  posematrix.col(0) << rot.col(0),0;
  posematrix.col(1) << rot.col(1),0;
  posematrix.col(2) << rot.col(2),0;
  posematrix.col(3) << vals[0],vals[1],vals[2],1;
  
  Eigen::Quaternionf quat(posematrix.block<3,3>(0,0));
  
  quat_vec.push_back(quat.x()); quat_vec.push_back(quat.y()); quat_vec.push_back(quat.z()); quat_vec.push_back(quat.w());
  
  return quat_vec;
}

geometry_msgs::PoseStamped GraspPlanner::find_transformation(geometry_msgs::Pose ref_obj,geometry_msgs::Pose cur_obj,geometry_msgs::PoseStamped ref_grasp)
{  
  Eigen::Quaternion<float> q_ref(ref_obj.orientation.w,ref_obj.orientation.x,ref_obj.orientation.y,ref_obj.orientation.z);
  Eigen::Vector3f t;
  
  t(0) = ref_obj.position.x; 
  t(1) = ref_obj.position.y; 
  t(2) = ref_obj.position.z;
  
  Eigen::Matrix4f ref_pose_trans;
  ref_pose_trans.block<3,3>(0,0) = q_ref.toRotationMatrix();
  ref_pose_trans.block<3,1>(0,3) = t;
  ref_pose_trans(3,0) = 0; ref_pose_trans(3,1) = 0; ref_pose_trans(3,2) = 0; ref_pose_trans(3,3) = 1;
  
  Eigen::Quaternion<float> q_cur(cur_obj.orientation.w,cur_obj.orientation.x,cur_obj.orientation.y,cur_obj.orientation.z);
  Eigen::Vector3f t_cur;
  
  t_cur(0) = cur_obj.position.x; 
  t_cur(1) = cur_obj.position.y; 
  t_cur(2) = cur_obj.position.z;

  Eigen::Matrix4f cur_pose_trans;
  cur_pose_trans.block<3,3>(0,0) = q_cur.toRotationMatrix();
  cur_pose_trans.block<3,1>(0,3) = t_cur;
  cur_pose_trans(3,0) = 0; cur_pose_trans(3,1) = 0; cur_pose_trans(3,2) = 0; cur_pose_trans(3,3) = 1;
  
  Eigen::Quaternion<float> q_grasp(ref_grasp.pose.orientation.w,ref_grasp.pose.orientation.x,ref_grasp.pose.orientation.y,ref_grasp.pose.orientation.z);
  
  Eigen::Vector3f t_grasp;
  t_grasp(0) = ref_grasp.pose.position.x;
  t_grasp(1) = ref_grasp.pose.position.y; 
  t_grasp(2) = ref_grasp.pose.position.z;

  Eigen::Matrix4f ref_grasp_trans;
  ref_grasp_trans.block<3,3>(0,0) = q_grasp.toRotationMatrix();
  ref_grasp_trans.block<3,1>(0,3) = t_grasp;
  ref_grasp_trans(3,0) = 0; ref_grasp_trans(3,1) = 0; ref_grasp_trans(3,2) = 0; ref_grasp_trans(3,3) = 1;
  
 /* Eigen::Matrix4f t_cur_ref_pose = ref_pose_trans.inverse() * cur_pose_trans;
  cout << "trans between cur obj and ref: " << endl << t_cur_ref_pose << endl;
  Eigen::Matrix4f t_obj_grasp = ref_grasp_trans.inverse() * ref_pose_trans;
  cout << "trans between grasp and object: " << endl << t_obj_grasp << endl;
  Eigen::Matrix4f trans_obj_grasp = t_cur_ref_pose * t_obj_grasp;
  cout << "trans between cur obj and grasp: " << endl << trans_obj_grasp << endl;
  
 // trans_obj_grasp =  cur_pose_trans.inverse() * trans_obj_grasp;
 // cout << "current gripper pose: " << endl << trans_obj_grasp << endl;
  Eigen::Matrix4d md(trans_obj_grasp.cast<double>());
  Eigen::Affine3d e = Eigen::Affine3d(md);
  geometry_msgs::PoseStamped trans_obj;
  poseEigenToMsg(e,trans_obj);
  grasp_score_.push_back(1);*/
  
 // Eigen::Quaternionf dq_obj = q_ref * q_cur.conjugate();
  Eigen::Quaternionf dq_obj = q_cur * q_ref.conjugate();
  /*Eigen::Vector3f dt_obj = t_cur;
  Eigen::Vector3f dt_obj_inv = -t;
  Eigen::Vector3f rot_tmp = dq_obj.toRotationMatrix() * dt_obj_inv;
  Eigen::Matrix3f rel_rot = dt_obj * rot_tmp.transpose();*/
  
  //Eigen::Quaternionf dq_grasp = q_cur * (q_grasp * q_ref.conjugate());//.conjugate();
  Eigen::Quaternionf dq_grasp = q_cur * ((q_grasp.conjugate() * q_ref).conjugate());
  Eigen::Vector3f dt_grasp = t_grasp - t;
  Eigen::Quaternionf q_grasp_new = dq_grasp;
  //Eigen::Vector3f t_grasp_new = dq_obj.toRotationMatrix() * dt_grasp;
  
 // Eigen::Vector3f t_grasp_new = (q_ref.conjugate().toRotationMatrix() * dt_grasp).transpose() * q_cur.toRotationMatrix();
   Eigen::Vector3f t_grasp_new = q_cur.toRotationMatrix() * (q_ref.conjugate().toRotationMatrix() * dt_grasp);
 // Eigen::Vector3f t_grasp_new = rel_rot * dt_grasp;
  
  t_grasp_new = t_grasp_new + t_cur;
  
  geometry_msgs::PoseStamped trans_obj;
  trans_obj.pose.position.x = t_grasp_new(0); 
  trans_obj.pose.position.y = t_grasp_new(1); 
  trans_obj.pose.position.z = t_grasp_new(2);  
  
  trans_obj.pose.orientation.x = q_grasp_new.x(); 
  trans_obj.pose.orientation.y = q_grasp_new.y(); 
  trans_obj.pose.orientation.z = q_grasp_new.z(); 
  trans_obj.pose.orientation.w = q_grasp_new.w(); 
  //grasp_score_.push_back(1);
  
  
  // ** that is the latests tested working version ** //
 /* Eigen::Quaternionf dq = q_grasp;
  Eigen::Vector3f dt = t_grasp - t;
  
  Eigen::Quaternionf q_grasp_new = q_grasp; 
  Eigen::Vector3f t_grasp_new = dt + t_cur;
  
  grasp_score_.push_back(1);
  
  geometry_msgs::PoseStamped trans_obj;
  trans_obj.pose.position.x = t_grasp_new(0); 
  trans_obj.pose.position.y = t_grasp_new(1); 
  trans_obj.pose.position.z = t_grasp_new(2);  
  
  trans_obj.pose.orientation.x = q_grasp_new.x(); 
  trans_obj.pose.orientation.y = q_grasp_new.y(); 
  trans_obj.pose.orientation.z = q_grasp_new.z(); 
  trans_obj.pose.orientation.w = q_grasp_new.w(); */
  
 // cout << "calculated difference in quaternion is: " << dq.x() << " : " << dq.y() << " : " << dq.z() << " : " << dq.w() << endl;
  return trans_obj;
}

void GraspPlanner::poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::PoseStamped &m)
{
    m.pose.position.x = e.translation()[0];
    m.pose.position.y = e.translation()[1];
    m.pose.position.z = e.translation()[2];
    
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    
    if (m.pose.orientation.w < 0) 
    {
        m.pose.orientation.x *= -1;
        m.pose.orientation.y *= -1;
        m.pose.orientation.z *= -1;
        m.pose.orientation.w *= -1;
    }
}

vector<string> GraspPlanner::giveAllFiles(string obj_id)
{
  grasp_type_.clear();

  if( obj_id.find("cuttlery") != string::npos )
    obj_id = "cuttlery";
  
  vector<string> path_to_obj_dir;
  DIR *dpdf;
  
  struct dirent *epdf;
  dpdf = opendir(path_to_dir.c_str());
  
  if (dpdf != NULL)
  {
   while (epdf = readdir(dpdf))
   {
      string cur_path = epdf->d_name;
      if( cur_path.find(obj_id) != string::npos )
      {
       	string path = path_to_dir + cur_path;
	      path_to_obj_dir.push_back(path);
        if( cur_path.find("cylindrical") != string::npos )
           grasp_type_.push_back(cylindrical);
        if( cur_path.find("spherical") != string::npos )
           grasp_type_.push_back(spherical);
        else
          grasp_type_.push_back(rim_close);        
      }
   }
  }
  
  for( size_t i = 0; i < path_to_obj_dir.size(); i++)
    cout << "path is: " << path_to_obj_dir[i] << endl;
  
  return path_to_obj_dir;
}

geometry_msgs::Pose GraspPlanner::searchPoseFile(string obj_id)
{
  geometry_msgs::Pose obj_pose;
  
  string file_name = obj_id;
  file_name = file_name + "/object-pos.txt";
  
  ifstream ifs(file_name.c_str());
  if( ifs.is_open() )
  {
    string line;    
    vector<double> vals;
    while( ifs.good() )
    {
      getline(ifs,line);
      while( line.length() > 1 )
      {
        double val;
	string val_str;
	if( line.find(" ") == string::npos )
	{
	  val_str = line;
	  line = "";
	}
	else
	{
	  val_str = line.substr(0,line.find(" "));
	  line = line.substr(line.find(" ")+1);
	}
	
	stringstream ss(val_str);
	ss  >> val;
	vals.push_back(val);	  
      }
     }  
     
     obj_pose.position.x = vals[0];
     obj_pose.position.y = vals[1];
     obj_pose.position.z = vals[2];
     
     obj_pose.orientation.x = vals[3];
     obj_pose.orientation.y = vals[4];
     obj_pose.orientation.z = vals[5];
     obj_pose.orientation.w = vals[6];
  }
  
  return obj_pose;
}

vector<geometry_msgs::PoseStamped> GraspPlanner::searchGraspFile(vector<string> path_to_obj_dir,bool pre_grasp)
{
  vector<geometry_msgs::PoseStamped> grasps;
  
  for( size_t i = 0; i < path_to_obj_dir.size(); i++ )
  {
    string file_name = path_to_obj_dir[i];
    if( !pre_grasp )
      file_name = file_name + "/gripper-pos.txt";
    else
      file_name = file_name + "/gripper-pre-pos.txt";
    
    ifstream ifs(file_name.c_str());
    
    if( ifs.is_open() )
    {
      string line;
      int count = 0;
      
      vector<double> vals;
      while( ( ifs.good() ) && ( count < 2 ) )
      {
       	getline(ifs,line);
      	//cout << "current line: " << line << endl;
	     while( line.length() > 1 )
	     {
	      double val;
	      string val_str;
	      if( line.find(" ") == string::npos )
	      {
	       val_str = line;
	       line = "";
	     }
	     else
	     {
	       val_str = line.substr(0,line.find(" "));
	       line = line.substr(line.find(" ")+1);
	     }
	     stringstream ss(val_str);
	     ss  >> val;
	     vals.push_back(val);	  
	   }
	   count++;
    }
 
      geometry_msgs::PoseStamped pose_cur;

      if( arm_ == "right" )
      {
        vector<double> q = euler_to_quaternion(vals);

     
        pose_cur.pose.position.x = ( vals[0] / 1000. ) + offset_x;
        pose_cur.pose.position.y = ( vals[1] / 1000. )+ offset_y;
        pose_cur.pose.position.z = ( vals[2] / 1000. );
      
        pose_cur.pose.orientation.x = q[0];
        pose_cur.pose.orientation.y = q[1];
        pose_cur.pose.orientation.z = q[2];
        pose_cur.pose.orientation.w = q[3];
      }
      else if( arm_ == "left" )
      {
        pose_cur.pose.position.x = vals[0];
        pose_cur.pose.position.y = vals[1];
        pose_cur.pose.position.z = vals[2];
      
        pose_cur.pose.orientation.x = vals[3];
        pose_cur.pose.orientation.y = vals[4];
        pose_cur.pose.orientation.z = vals[5];
        pose_cur.pose.orientation.w = vals[6];
      }

      
      grasps.push_back(pose_cur);
    }
    else
      ROS_INFO("cannot open grasp file: %s",file_name.c_str());
  }
  
  return grasps;
}

geometry_msgs::PoseStamped  GraspPlanner::transform_frame(geometry_msgs::PoseStamped pose)
{
    geometry_msgs::PoseStamped pose_trans;

    Eigen::Vector3f trans_to_palm;
    trans_to_palm(0) = 0; trans_to_palm(1) = 0; trans_to_palm(2) = 0.017;
    Eigen::Quaternionf quat_to_palm(0.707,0,0,0.707);
    Eigen::Matrix3f rot_palm = quat_to_palm.toRotationMatrix();
    Eigen::Matrix4f mat_palm;
    mat_palm.block<3,3>(0,0) = rot_palm;
    mat_palm(0,3) = trans_to_palm(0); mat_palm(1,3) = trans_to_palm(1); mat_palm(2,3) = trans_to_palm(2);
    mat_palm(3,0) = 0; mat_palm(3,1) = 0; mat_palm(3,2) = 0; mat_palm(3,3) = 1;

    Eigen::Vector3f pose_transl;
    pose_transl(0) = pose.pose.position.x; pose_transl(1) = pose.pose.position.y; pose_transl(2) = pose.pose.position.z;
    Eigen::Quaternionf quat_pose(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);  
    Eigen::Matrix3f rot_pose = quat_pose.toRotationMatrix();
    Eigen::Matrix4f mat_pose;
    mat_pose.block<3,3>(0,0) = rot_pose;
    mat_pose(0,3) = pose_transl(0); mat_pose(1,3) = pose_transl(1); mat_pose(2,3) = pose_transl(2);
    mat_pose(3,0) = 0; mat_pose(3,1) = 0; mat_pose(3,2) = 0; mat_pose(3,3) = 1;    
    
    Eigen::Matrix4f transformed_pose;
    transformed_pose = mat_pose * mat_palm;

    Eigen::Quaternionf quat(transformed_pose.block<3,3>(0,0));
    pose_trans.pose.orientation.x = quat.x(); pose_trans.pose.orientation.y = quat.y();
    pose_trans.pose.orientation.z = quat.z(); pose_trans.pose.orientation.w = quat.w();

    pose_trans.pose.position.x = transformed_pose(0,3);
    pose_trans.pose.position.y = transformed_pose(1,3);
    pose_trans.pose.position.z = transformed_pose(2,3);

    return pose_trans;   
}

bool GraspPlanner::extractGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res)
{ 
  grasp_score_.clear();
  bool success = false;
  
  estimated_grasps.clear();
  
  definitions::Object object = req.ordered_objects[req.object_id];
  set_arm(req.arm);
  
  string obj_name = object.name.data;
  obj_name = obj_name.substr(0,obj_name.find("\n"));
  
  cout << "object name is: " << obj_name << "."<< endl;
  /*if( obj_name.find("container_2") != string::npos )
    offset_y = 0.02;*/
  
  vector<string> path_to_obj_dir = giveAllFiles(obj_name);
  
  if( path_to_obj_dir.size() == 0 )
  {
    cout << "no grasp found" << endl;
    res.result = res.NO_FEASIBLE_GRASP_FOUND;     
    return false;    
  }
  
  geometry_msgs::Pose obj_ref = searchPoseFile(path_to_obj_dir[0]);
  geometry_msgs::Pose obj_pose = object.pose;
  
  vector<geometry_msgs::PoseStamped> pre_grasps = searchGraspFile(path_to_obj_dir,true);
  vector<geometry_msgs::PoseStamped> grasps = searchGraspFile(path_to_obj_dir,false);
  
  vector<definitions::Grasp> grasp_traj;
 
/*  vector<float> pre_grasp_joints = getTargetAnglesFromGraspType(rim_pre_grasp,1.0);
  // vector<float> pre_grasp_joints = getTargetAnglesFromGraspType(rim_open,1.0);
  vector<float> grasp_joints = getTargetAnglesFromGraspType(rim_close,1.0); */

  int min_id = 0;
  double min = 1000.;
  definitions::GraspList grasp_list;
  for( size_t i = 0; i < grasps.size(); i++ )
  {
    vector<float> pre_grasp_joints;
    vector<float> grasp_joints;
    if( grasp_type_[i] == cylindrical )
    {
      pre_grasp_joints = getTargetAnglesFromGraspType(cylindrical,0.1);
      grasp_joints = getTargetAnglesFromGraspType(cylindrical,1.0); 
      cur_grasp_type_ = cylindrical;
    }
    if( grasp_type_[i] == spherical )
    {
      pre_grasp_joints = getTargetAnglesFromGraspType(spherical,0.1);
      grasp_joints = getTargetAnglesFromGraspType(spherical,1.0); 
      cur_grasp_type_ = spherical;
    }    
    else
    {
      pre_grasp_joints = getTargetAnglesFromGraspType(rim_pre_grasp,1.0);
      grasp_joints = getTargetAnglesFromGraspType(rim_close,1.0);
      cur_grasp_type_ = rim_close;
    }

    geometry_msgs::PoseStamped old_pre_grasp = pre_grasps[i];
    geometry_msgs::PoseStamped old_grasp = grasps[i];
    pre_grasps[i] = find_transformation(obj_ref,obj_pose,pre_grasps[i]);
    grasps[i] = find_transformation(obj_ref,obj_pose,grasps[i]);
     
  // ** since by default planning is on palm ** // 
    pre_grasps[i] = transform_frame(pre_grasps[i]);
    grasps[i] = transform_frame(grasps[i]);
 
    vector<float> mid_grasp_joint; 
    double ratio = 0.8;
    double hand_ratio = 0.2;
    geometry_msgs::PoseStamped mid_grasp = find_mid_point(pre_grasps[i],grasps[i],mid_grasp_joint,ratio,hand_ratio);

    vector<float> mid_grasp_joint2; 
    ratio = 0.9;
    hand_ratio = 0.6;
    geometry_msgs::PoseStamped mid_grasp2 = find_mid_point(pre_grasps[i],grasps[i],mid_grasp_joint2,ratio,hand_ratio);

    if( i == 0 )
      visualize_gripper(pre_grasps[i],grasps[i],i);
   /* double gs = (grasp_score_[grasp_score_.size()-1] + grasp_score_[grasp_score_.size()-2]) / 2.;
    grasp_score_.pop_back();
    grasp_score_[grasp_score_.size()-1] = gs;
    if( gs < min )
    {
      min = gs;
      min_id = i;
    }*/
    cout << "in planner, grasp " << i << " is: " << grasps[i].pose << endl;
    definitions::Grasp cur_traj;
    cur_traj.grasp_trajectory.resize(3);
    pre_grasps[i].header.frame_id = "world_link";
    grasps[i].header.frame_id = "world_link";
    cur_traj.grasp_trajectory[0].wrist_pose.pose = pre_grasps[i].pose;
    cur_traj.grasp_trajectory[0].wrist_pose.header.frame_id = "world_link";
    cur_traj.grasp_trajectory[0].joints = pre_grasp_joints;

    /*cur_traj.grasp_trajectory[1].wrist_pose.pose = grasps[i].pose;
    cur_traj.grasp_trajectory[1].wrist_pose.header.frame_id = "world_link";
//    cur_traj.grasp_trajectory[1].joints = grasp_joints;
    cur_traj.grasp_trajectory[1].joints = pre_grasp_joints;*/

    cur_traj.grasp_trajectory[1].wrist_pose.pose = mid_grasp.pose;
    cur_traj.grasp_trajectory[1].wrist_pose.header.frame_id = "world_link";
    cur_traj.grasp_trajectory[1].joints = mid_grasp_joint; 

   /* cur_traj.grasp_trajectory[2].wrist_pose.pose = mid_grasp2.pose;
    cur_traj.grasp_trajectory[2].wrist_pose.header.frame_id = "world_link";
    cur_traj.grasp_trajectory[2].joints = mid_grasp_joint2; */

    cur_traj.grasp_trajectory[2].wrist_pose.pose = grasps[i].pose;   
    cur_traj.grasp_trajectory[2].wrist_pose.header.frame_id = "world_link";
    cur_traj.grasp_trajectory[2].joints = grasp_joints;
    grasp_traj.push_back(cur_traj);
    grasp_list.grasp_list.push_back(cur_traj);
  }

  cout << "after getting all the grasps" << endl;
 /* if( grasp_traj.size() > 0 ){
    definitions::Grasp min_traj = grasp_traj[min_id];
    definitions::Grasp first_grasp = grasp_traj[0];
    grasp_traj[0] = min_traj;
    grasp_traj[0] = first_grasp;
  }*/
  pub_grasps_.publish(grasp_list);

  res.grasp_list = grasp_traj;
  
  if( grasps.size() > 0 )
  {
    res.result = res.SUCCESS;
    success = true;
    //return true;
  }
  else
  {
    res.result = res.NO_FEASIBLE_GRASP_FOUND; 
    success = false;
    return false;
  }
  return success;
}

}

int main(int argc,char ** argv)
{
  ros::init(argc, argv, "grasp_planner_uibk");
  ros::NodeHandle nh;
  ros::Rate loop_rate(15);
  ros::Timer delay();
 
  grasp_planner_uibk::GraspPlanner grasp(nh);
  ROS_INFO("Grasp Planner UIBK node ready to be called...");
  while(ros::ok())
  {
    ros::Rate r(30);
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}