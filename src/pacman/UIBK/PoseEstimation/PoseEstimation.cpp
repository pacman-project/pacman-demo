#include <pacman/UIBK/PoseEstimation/PoseEstimationImpl.h>
#include <memory>

using namespace pacman;

//-----------------------------------------------------------------------------

UIBKObjectImpl::UIBKObjectImpl(const std::string& path) {
}

void UIBKObjectImpl::add(const std::string& id, const Point3D::Seq& points) {
	// TODO (optional)
}

void UIBKObjectImpl::get(const std::string& id, Point3D::Seq& points) const {
}

void UIBKObjectImpl::remove(const std::string& id) {
	// TODO (optional)
}

void UIBKObjectImpl::list(std::vector<std::string>& idSeq) const {
}

//-----------------------------------------------------------------------------

std::shared_ptr<UIBKObjectImpl> pUIBKObjectImpl;

UIBKObject* UIBKObject::create(const std::string& path) {
	pUIBKObjectImpl.reset(new UIBKObjectImpl(path));
	return pUIBKObjectImpl.get();
}

//-----------------------------------------------------------------------------

UIBKPoseEstimationImpl::UIBKPoseEstimationImpl(const std::string& path) {
}

void UIBKPoseEstimationImpl::capture(Point3D::Seq& points) {
}

void UIBKPoseEstimationImpl::estimate(const Point3D::Seq& points, Pose::Seq& poses) {
}

//-----------------------------------------------------------------------------

std::shared_ptr<UIBKPoseEstimationImpl> pUIBKPoseEstimationImpl;

UIBKPoseEstimation* UIBKPoseEstimation::create(const std::string& path) {
	pUIBKPoseEstimationImpl.reset(new UIBKPoseEstimationImpl(path));
	return pUIBKPoseEstimationImpl.get();
}

//-----------------------------------------------------------------------------
