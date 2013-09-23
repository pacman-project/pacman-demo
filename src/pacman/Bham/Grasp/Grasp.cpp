#include <pacman/Bham/Grasp/Grasp.h>
#include <pacman/Bham/Grasp/GraspImpl.h>
#include <memory>

using namespace pacman;

//-----------------------------------------------------------------------------

BhamGraspImpl::BhamGraspImpl(const std::string& path) {
}

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const ShunkDexHand::Pose& trajectory) {
}

void BhamGraspImpl::remove(const std::string& id) {
}

void BhamGraspImpl::list(std::vector<std::string>& idSeq) const {
}

void BhamGraspImpl::process() {
}

void BhamGraspImpl::estimate(const Point3D::Seq& points, ShunkDexHand::Pose::Seq& trajectories, std::vector<float_t>& weights) {
}

//-----------------------------------------------------------------------------

std::shared_ptr<BhamGraspImpl> pBhamGraspImpl;

BhamGrasp* BhamGrasp::create(const std::string& path) {
	pBhamGraspImpl.reset(new BhamGraspImpl(path));
	return pBhamGraspImpl.get();
}

//-----------------------------------------------------------------------------
