#include <pacman/UIBK/PoseEstimation/PoseEstimationImpl.h>
#include <memory>

using namespace pacman;

//-----------------------------------------------------------------------------

std::shared_ptr<I_SegmentedObjects> objects;

UIBKObjectImpl::UIBKObjectImpl(const std::string& path) 
{
   objects.reset(new I_SegmentedObjects());
}

void UIBKObjectImpl::add(const std::string& id, const Point3D::Seq& points) 
{

  objects->addObjectName(id);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points (new pcl::PointCloud<pcl::PointXYZ>());
  convert<pcl::PointXYZ>(points,*xyz_points);
  objects->addPointCloud(xyz_points);

}

void UIBKObjectImpl::get(const std::string& id, Point3D::Seq& points) const 
{

  std::vector<string> names =  objects->getObjects();
  int obj_id = 0;
 
  while( ( names.at(obj_id) != id ) && ( obj_id < names.size() ) )
    obj_id++;
 
 if( obj_id < names.size() )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points = objects->getPointCloudAt(obj_id);
    convert(*xyz_points,points);
   }
}

void UIBKObjectImpl::remove(const std::string& id) 
{
	// TODO (optional)
}

void UIBKObjectImpl::list(std::vector<std::string>& idSeq) const 
{
	idSeq = objects->getObjects();
}

//-----------------------------------------------------------------------------

std::shared_ptr<UIBKObjectImpl> pUIBKObjectImpl;
std::shared_ptr<ParametersPoseEstimation> params_;

UIBKObject* UIBKObject::create(const std::string& path) 
{
	pUIBKObjectImpl.reset(new UIBKObjectImpl(path));
	return pUIBKObjectImpl.get();
}

//-----------------------------------------------------------------------------

UIBKPoseEstimationImpl::UIBKPoseEstimationImpl(const std::string& path) 
{
  transformations = new Transformations();  
  params_.reset(new ParametersPoseEstimation(path)); 
  recognizedObjects = false;
}

void UIBKPoseEstimationImpl::capture(Point3D::Seq& points)
{
        
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points( new pcl::PointCloud<pcl::PointXYZ>() );
  //get point cloud from the kinect
  
  if( params_->useKinect )
  {
    xyz_points = params_->kinectGrabFrame();
  }
  else
  {
    xyz_points = params_->loadPCDFile(params_->test_file);
  }

  xyz_points_ = xyz_points;
  *xyz_points_ = *xyz_points; 
  convert(*xyz_points_,points);
}

void UIBKPoseEstimationImpl::estimate(const Point3D::Seq& points, Pose::Seq& poses) 
{

  //params_->recognizePose(*objects,xyz_points_);
  params_->recognizePose(*objects);

  boost::shared_ptr < vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_ = objects->getTransforms();

  for(int t=0;t<objects->getObjects().size();++t)    
  {
    Pose cur_pos;
    vector<double> obj_heights = objects->getHeightList();

    int id = obj_heights.at(t);
    cur_pos.id = objects->getObjectsNameAt(id);

    Eigen::Matrix3f rotation = transforms_->at (id).block<3,3>(0, 0);
    Eigen::Vector3f translation = transforms_->at (id).block<3,1>(0, 3);

    transformations->transformUIBKPoseToPose(translation,rotation,cur_pos);

    poses.push_back(cur_pos);
  }

  recognizedObjects = true;
}

//-----------------------------------------------------------------------------

std::shared_ptr<UIBKPoseEstimationImpl> pUIBKPoseEstimationImpl;

UIBKPoseEstimation* UIBKPoseEstimation::create(const std::string& path) 
{
	pUIBKPoseEstimationImpl.reset(new UIBKPoseEstimationImpl(path));
	return pUIBKPoseEstimationImpl.get();
}

//-----------------------------------------------------------------------------

void Transformations::transformUIBKPoseToPose(const Eigen::Vector3f& translation, const Eigen::Matrix3f& rotation,pacman::UIBKPoseEstimation::Pose &pose_)
{
  pose_.pose.p.x = translation[0]; 
  pose_.pose.p.y = translation[1];
  pose_.pose.p.z = translation[2];

  pose_.pose.R.m11 = rotation(0,0); 
  pose_.pose.R.m12 = rotation(0,1);
  pose_.pose.R.m13 = rotation(0,2);
  pose_.pose.R.m21 = rotation(1,0);
  pose_.pose.R.m22 = rotation(1,1);
  pose_.pose.R.m23 = rotation(1,2);
  pose_.pose.R.m31 = rotation(2,0);
  pose_.pose.R.m32 = rotation(2,1);
  pose_.pose.R.m33 = rotation(2,2);
}
