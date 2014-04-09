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

#include "definitions/PoseEstimation.h"
#include "definitions/GraspPlanning.h"

using namespace std;

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
  
public:
    
    GraspPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      
      srv_grasp_planner_ = nh_.advertiseService(nh_.resolveName("/grasp_planner_srv"),&GraspPlanner::extractGrasp, this);
      
    //offset_x = -0.04;
      offset_x = -0.02;
      offset_y = 0.03; 
      //offset_y = 0.02;
      vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("gripper", 1 );

      nh_.param<std::string>("path_to_dir", root, "");
    }
    
    ~GraspPlanner()
    {}
    
    bool extractGrasp(definitions::GraspPlanning::Request  &req, definitions::GraspPlanning::Response &res);
   // definitions::Grasp searchGraspFile(string obj_id);
    
    vector<geometry_msgs::PoseStamped> searchGraspFile(vector<string> path_to_obj_dir,bool pre_grasp);
    
    geometry_msgs::Pose searchPoseFile(string obj_id);
    
    vector<string> giveAllFiles(string obj_id);
    
    // ** get transformation between given point cloud and the one which we learned in grasp learning
    Eigen::Vector3f get_transformation(geometry_msgs::Pose ref_obj,geometry_msgs::Pose cur_obj);
    
    std::vector<double> euler_to_quaternion(vector<double> vals);
    
    geometry_msgs::PoseStamped apply_transformation(geometry_msgs::PoseStamped cur_obj,Eigen::Vector3f trans_ref);
    
    void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::PoseStamped &m);
    
    geometry_msgs::PoseStamped find_transformation(geometry_msgs::Pose ref_obj,geometry_msgs::Pose cur_obj,geometry_msgs::PoseStamped cur_grasp);
    void visualize_gripper(geometry_msgs::PoseStamped gripper_pre_pose,geometry_msgs::PoseStamped gripper_pose);
    //void evaluate_grasp(vector<geometry_msgs::PoseStamped> pre_grasps,vector<geometry_msgs::PoseStamped> grasps,string obj_name);
    void set_arm(string arm);
    void visualize_gripper(vector<geometry_msgs::PoseStamped> pre_grasps,vector<geometry_msgs::PoseStamped> grasps);
};

void GraspPlanner::set_arm(string arm)
{
  if( arm == "right" )
    path_to_dir = root + "/grasps-models-multi-grasps_right/";
    //path_to_dir = "/home/pacman/Documents/backup-grasps-models_intellact-multi-grasps_right/";
  else if(arm == "left")
    path_to_dir = root + "/grasps-models-multi-grasps_left/";
  cout << "path to directory is: " << path_to_dir << endl;
}

void GraspPlanner::visualize_gripper(geometry_msgs::PoseStamped gripper_pre_pose,geometry_msgs::PoseStamped gripper_pose)
{
  visualization_msgs::MarkerArray markers;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_link";
  marker.header.stamp = ros::Time();
  marker.ns = "gripper_pre_ns";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = gripper_pre_pose.pose.position;
  marker.pose.orientation = gripper_pre_pose.pose.orientation;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  markers.markers.push_back(marker);
  
  marker.ns = "gripper_ns";
  marker.id = 1;
  marker.pose.position = gripper_pose.pose.position;
  marker.pose.orientation = gripper_pose.pose.orientation;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
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
  
  Eigen::Quaternionf dq_obj = q_cur * q_ref.conjugate();

  Eigen::Quaternionf dq_grasp = q_cur * ((q_grasp.conjugate() * q_ref).conjugate());
  Eigen::Vector3f dt_grasp = t_grasp - t;
  Eigen::Quaternionf q_grasp_new = dq_grasp;

   Eigen::Vector3f t_grasp_new = q_cur.toRotationMatrix() * (q_ref.conjugate().toRotationMatrix() * dt_grasp);
  
  t_grasp_new = t_grasp_new + t_cur;
  
  geometry_msgs::PoseStamped trans_obj;
  trans_obj.pose.position.x = t_grasp_new(0); 
  trans_obj.pose.position.y = t_grasp_new(1); 
  trans_obj.pose.position.z = t_grasp_new(2);  
  
  trans_obj.pose.orientation.x = q_grasp_new.x(); 
  trans_obj.pose.orientation.y = q_grasp_new.y(); 
  trans_obj.pose.orientation.z = q_grasp_new.z(); 
  trans_obj.pose.orientation.w = q_grasp_new.w(); 
  grasp_score_.push_back(1);
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

      vector<double> q = euler_to_quaternion(vals);
      geometry_msgs::PoseStamped pose_cur;
     
      pose_cur.pose.position.x = ( vals[0] / 1000. ) + offset_x;
      pose_cur.pose.position.y = ( vals[1] / 1000. )+ offset_y;
      pose_cur.pose.position.z = vals[2] / 1000.;
      
      pose_cur.pose.orientation.x = q[0];
      pose_cur.pose.orientation.y = q[1];
      pose_cur.pose.orientation.z = q[2];
      pose_cur.pose.orientation.w = q[3];
      
      grasps.push_back(pose_cur);
    }
    else
      ROS_INFO("cannot open grasp file: %s",file_name.c_str());
  }
  
  return grasps;
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
  if( obj_name.find("container_2") != string::npos )
    offset_y = 0.02;
  
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
  int min_id = 0;
  double min = 1000.;
  for( size_t i = 0; i < grasps.size(); i++ )
  {
    //cout << grasps[i] << endl;
    geometry_msgs::PoseStamped old_pre_grasp = pre_grasps[i];
    geometry_msgs::PoseStamped old_grasp = grasps[i];
    pre_grasps[i] = find_transformation(obj_ref,obj_pose,pre_grasps[i]);
    grasps[i] = find_transformation(obj_ref,obj_pose,grasps[i]);
    if( i == 0 )
      visualize_gripper(pre_grasps[i],grasps[i]);
    double gs = (grasp_score_[grasp_score_.size()-1] + grasp_score_[grasp_score_.size()-2]) / 2.;
    grasp_score_.pop_back();
    grasp_score_[grasp_score_.size()-1] = gs;
    if( gs < min )
      {
	min = gs;
	min_id = i;
      }
    definitions::Grasp cur_traj;
    cur_traj.grasp_trajectory.resize(3);
    cur_traj.grasp_trajectory[0].wrist_pose.pose = pre_grasps[i].pose;
    cur_traj.grasp_trajectory[1].wrist_pose.pose = grasps[i].pose;
    cur_traj.grasp_trajectory[2].wrist_pose.pose = grasps[i].pose;   
    grasp_traj.push_back(cur_traj);
    
    if( ( obj_name.find("Shaft") != string::npos ) || ( obj_name.find("Bolt") != string::npos ) )
    {
      cout << "consider the real orientation of the object" << endl;
      geometry_msgs::Pose obj_pose_tmp = obj_pose;
      obj_pose.orientation = obj_ref.orientation;
      old_pre_grasp = find_transformation(obj_ref,obj_pose_tmp,old_pre_grasp);
      old_grasp = find_transformation(obj_ref,obj_pose,old_grasp);
      definitions::Grasp old_traj;
      old_traj.grasp_trajectory.resize(3);
      old_traj.grasp_trajectory[0].wrist_pose.pose = old_pre_grasp.pose;
      old_traj.grasp_trajectory[1].wrist_pose.pose = old_grasp.pose;
      old_traj.grasp_trajectory[2].wrist_pose.pose = old_grasp.pose;
      grasp_traj.push_back(old_traj);  
    }
  }
  cout << "after getting all the grasps" << endl;
  if( grasp_traj.size() > 0 ){
    definitions::Grasp min_traj = grasp_traj[min_id];
    definitions::Grasp first_grasp = grasp_traj[0];
    grasp_traj[0] = min_traj;
    grasp_traj[0] = first_grasp;
  }
  
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
 // evaluate_grasp(pre_grasps,grasps,obj_name);
  visualize_gripper(pre_grasps,grasps);
  return success;
}

// void GraspPlanner::evaluate_grasp(vector<geometry_msgs::PoseStamped> pre_grasps,vector<geometry_msgs::PoseStamped> grasps,string obj_name)
// {
//   path_to_dir = "/home/pacman/Documents/grasp-ground-truth/";
//   vector<string> path_to_obj_dir = giveAllFiles(obj_name);
//   vector<geometry_msgs::PoseStamped> pre_grasps_ref = searchGraspFile(path_to_obj_dir,true);
//   vector<geometry_msgs::PoseStamped> grasps_ref = searchGraspFile(path_to_obj_dir,false);
//   cout << grasps_ref.size() << endl;
//   double min_dist = 1000.;
//   double min_rot_dist = 1000.;
//   geometry_msgs::Pose min_pose;
//   Eigen::Quaternion<float> dq;

//   for( size_t i = 0; i < grasps.size(); i++ )
//   {
//     geometry_msgs::PoseStamped grasp_cur = grasps[i];
//     Eigen::Vector3f grasp_trans_cur;
//     grasp_trans_cur(0) = grasp_cur.pose.position.x; grasp_trans_cur(1) = grasp_cur.pose.position.y; grasp_trans_cur(2) = grasp_cur.pose.position.z;
//     Eigen::Quaternion<float> q_grasp_cur(grasp_cur.pose.orientation.w,grasp_cur.pose.orientation.x,grasp_cur.pose.orientation.y,grasp_cur.pose.orientation.z);
//     for( size_t j = 0; j < grasps_ref.size(); j++ )
//     {
//       geometry_msgs::PoseStamped gr = grasps_ref[j];
//       Eigen::Vector3f grasp_trans_ref;
//       grasp_trans_ref(0) = gr.pose.position.x; grasp_trans_ref(1) = gr.pose.position.y; grasp_trans_ref(2) = gr.pose.position.z;
//       Eigen::Quaternion<float> q_grasp_ref(gr.pose.orientation.w,gr.pose.orientation.x,gr.pose.orientation.y,gr.pose.orientation.z);  
//       geometry_msgs::Pose pose_ref = searchPoseFile(path_to_obj_dir[j]);
//       double dist = (grasp_trans_cur - grasp_trans_ref).norm();
//       if( dist < min_dist )
//       {
// 	min_dist = dist;
// 	min_pose = pose_ref;
// 	dq = q_grasp_cur * q_grasp_ref.conjugate();
//       }
//     }
//   }
//   path_to_dir = "/home/pacman/Documents/backup-grasps-models-multi-grasps/";
//   cout << "min ref gripper pose is: " << endl <<
//        min_pose.position.x << " "<< min_pose.position.y << " "<< min_pose.position.z << " "<< 
//        min_pose.orientation.x << " " << min_pose.orientation.y << " "<< min_pose.orientation.z << " "<< min_pose.orientation.w << " "<<endl;  
//   cout << "error is: " << min_dist << endl;
//   cout << "quaternion diff is: " << dq.x() << " "<< dq.y() << " "<< dq.z() << " " << dq.w() << endl;
// }

void GraspPlanner::visualize_gripper(vector<geometry_msgs::PoseStamped> pre_grasps,vector<geometry_msgs::PoseStamped> grasps)
{
  string path_to_model = root + "/hand.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_model.c_str(), *cloud_model) == -1) //* load the file
  {
    ROS_ERROR("Error at reading hand model ...");
    return;
  }
  
  string pre_path = "/tmp/gripper"; 
  if(!boost::filesystem::exists(pre_path))  
    boost::filesystem::create_directory(pre_path);   
  for( size_t i = 0; i < grasps.size(); i++ )
  {
    Eigen::Matrix4f trans;
    Eigen::Quaternion<float> quat(grasps[i].pose.orientation.w,grasps[i].pose.orientation.x,grasps[i].pose.orientation.y,grasps[i].pose.orientation.z);
    trans.block<3,3>(0,0) = quat.toRotationMatrix();
    trans(0,3) = grasps[i].pose.position.x; trans(1,3) = grasps[i].pose.position.y; trans(2,3) = grasps[i].pose.position.z; trans(3,3) = 1;
    trans(3,0) = 0; trans(3,1) = 0; trans(3,2) = 0; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_model,*transform_cloud,trans);  
    
    stringstream ss_grasp;
    ss_grasp << pre_path << "/gripper_" << i << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss_grasp.str(),*transform_cloud, false);
  }
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