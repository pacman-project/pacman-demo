#include <pacman/Bham/Grasp/GraspImpl.h>
#include <pacman/PaCMan/PCL.h>
#include <Golem/Phys/Data.h>
#include <Golem/Tools/Data.h>
#include <pcl/io/pcd_io.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

BhamGraspImpl::BhamGraspImpl(golem::Scene &scene) : ShapePlanner(scene) {
}

bool BhamGraspImpl::create(const grasp::ShapePlanner::Desc& desc) {
	(void)ShapePlanner::create(desc);

	scene.setHelp(
		scene.getHelp() +
		"  A                                       PaCMan operations\n"
	);
	
	return true;
}

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const RobotUIBK::Pose::Seq& trajectory) {
}

void BhamGraspImpl::remove(const std::string& id) {
}

void BhamGraspImpl::list(std::vector<std::string>& idSeq) const {
}

void BhamGraspImpl::estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories) {
}

void BhamGraspImpl::spin() {
	try {
		main();
	}
	catch (grasp::Interrupted&) {
	}
}

void BhamGraspImpl::function(TrialData::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'A':
		switch (waitKey("E", "Press a key to (E)xport data...")) {
		case 'E':
		{
			// export data
			std::string path;
			readString("Enter file path: ", path);
			// point cloud
			if (!dataPtr->second.pointCloud.empty()) {
				const std::string pathCloud = path + ".pcd";
				context.write("Exporting point cloud to: %s\n", pathCloud.c_str());
				Point3D::Seq dst;
				convert(dataPtr->second.pointCloud, dst);
				save(pathCloud, dst);
			}
			// appproach trajectory
			if (!dataPtr->second.approachAction.empty()) {
				const std::string pathTrj = path + ".trj";
				context.write("Exporting approach trajectory to: %s\n", pathTrj.c_str());
				ShunkDexHand::Pose::Seq dst;
				convert(dataPtr->second.approachAction, dst);
				save(pathTrj, dst);
			}
			break;
		}
		};
		context.write("Done!\n");
		break;
	};

	ShapePlanner::function(dataPtr, key);
}

void BhamGraspImpl::convert(const ::grasp::Point::Seq& src, Point3D::Seq& dst) const {
	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		Point3D point;
		i.frame.p.get(&point.position.x);
		i.normal.get(&point.normal.x);
		i.colour.get(&point.colour.r);
		dst.push_back(point);
	}
}

void BhamGraspImpl::convert(const Point3D::Seq& src, ::grasp::Point::Seq& dst) const {
	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		Point point;
		point.frame.p.set(&i.position.x);
		point.normal.set(&i.normal.x);
		point.colour.set(&i.colour.r);
		dst.push_back(point);
	}
}

void BhamGraspImpl::convert(const ::grasp::RobotState::List& src, ShunkDexHand::Pose::Seq& dst) const {
	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		ShunkDexHand::Pose pose;
		const Manipulator::Pose poseTmp(grasp.second->getManipulator().getPose(i.config));
		poseTmp.p.get(&pose.pose.p.x);
		poseTmp.R.getRow33(&pose.pose.R.m11);
		std::copy(poseTmp.jc + grasp.second->getManipulator().getArmJoints(), poseTmp.jc + grasp.second->getManipulator().getJoints(), pose.config.c);
		dst.push_back(pose);
	}
}

void BhamGraspImpl::convert(const RobotUIBK::Pose::Seq& src, ::grasp::RobotState::List& dst) const {
	dst.clear();
	for (auto i: src) {
		::grasp::RobotState pose(*robot->getController());
		robot->getController()->setToDefault(pose.config);
		robot->getController()->setToDefault(pose.command);

		dst.push_back(pose);
	}
}

//-----------------------------------------------------------------------------

void pacman::save(const std::string& path, const Point3D::Seq& points) {
	pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
	pacman::convert(points, pclCloud);
	if (pcl::PCDWriter().writeBinaryCompressed(path, pclCloud) < 0)
		throw Message(Message::LEVEL_ERROR, "pacman::save(): pcl::PCDWriter error when writing %s", path.c_str());
}

void pacman::load(const std::string& path, Point3D::Seq& points) {
	pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
	if (pcl::PCDReader().read(path, pclCloud) < 0)
		throw Message(Message::LEVEL_ERROR, "pacman::load(): pcl::PCDReader error when reading %s", path.c_str());
	pacman::convert(pclCloud, points);
}

void pacman::save(const std::string& path, const ShunkDexHand::Pose::Seq& trajectory) {
}

void pacman::load(const std::string& path, ShunkDexHand::Pose::Seq& trajectory) {
}

//-----------------------------------------------------------------------------

Context::Ptr context;
Universe::Ptr universe;

BhamGrasp* BhamGrasp::create(const std::string& path) {
	// Create XML parser and load configuration file
	XMLParser::Ptr pParser = XMLParser::load(path);

	// Find program XML root context
	XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
	if (pXMLContext == NULL)
		throw MsgApplication(Message::LEVEL_CRIT, "Unknown configuration file: %s", path.c_str());

	// Create program context
	golem::Context::Desc contextDesc;
	XMLData(contextDesc, pXMLContext);
	context = contextDesc.create(); // throws
		
	// Create Universe
	Universe::Desc universeDesc;
	XMLData(universeDesc, pXMLContext->getContextFirst("universe"));
	universe = universeDesc.create(*context);
		
	// Create scene
	Scene::Desc sceneDesc;
	XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
	Scene *pScene = universe->createScene(sceneDesc);
		
	// Launch universe
	universe->launch();

	// Setup Birmingham grasp interface
	BhamGraspImpl::Desc bhamGraspDesc;
	XMLData(bhamGraspDesc, context.get(), pXMLContext);

	BhamGraspImpl *pBhamGrasp = dynamic_cast<BhamGraspImpl*>(pScene->createObject(bhamGraspDesc)); // throws
	if (pBhamGrasp == NULL)
		throw Message(Message::LEVEL_CRIT, "BhamGrasp::create(): Unable to create Birmingham grasp interface");

	// Random number generator seed
	context->info("Random number generator seed %d\n", context->getRandSeed()._U32[0]);
	
	return pBhamGrasp;
}

//-----------------------------------------------------------------------------
