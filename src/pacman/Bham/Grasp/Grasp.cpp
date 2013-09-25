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

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const RobotUIBK::Config::Seq& trajectory) {
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
	{
		switch (waitKey("IE", "Press a key to (I)mport/(E)xport data...")) {
		case 'I':
		{
			const int key = waitKey("PT", "Press a key to import (P)oint cloud/(T)trajectory...");
			// export data
			std::string path = dataPtr->first;
			readString("Enter file path: ", path);
			// point cloud
			if (key == 'P') {
				context.write("Importing point cloud from: %s\n", path.c_str());
				Point3D::Seq seq;
				load(path, seq);
				convert(seq, dataPtr->second.pointCloud);
				renderTrialData(dataPtr);
			}
			// appproach trajectory
			if (key == 'T') {
				context.write("Importing approach trajectory from: %s\n", path.c_str());
				RobotUIBK::Config::Seq seq;
				load(path, seq);
				convert(seq, dataPtr->second.approachAction);
			}
			break;
		}
		case 'E':
		{
			const int key = waitKey("PT", "Press a key to export (P)oint cloud/(T)trajectory...");
			// export data
			std::string path = dataPtr->first;
			readString("Enter file path: ", path);
			// point cloud
			if (key == 'P') {
				context.write("Exporting point cloud to: %s\n", path.c_str());
				Point3D::Seq seq;
				convert(dataPtr->second.pointCloud, seq);
				save(path, seq);
			}
			// appproach trajectory
			if (key == 'T') {
				context.write("Exporting approach trajectory to: %s\n", path.c_str());
				RobotUIBK::Config::Seq seq;
				convert(dataPtr->second.approachAction, seq);
				save(path, seq);
			}
			break;
		}
		}
		context.write("Done!\n");
		break;
	}
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

void BhamGraspImpl::convert(const ::grasp::RobotState::List& src, RobotUIBK::Config::Seq& dst) const {
	if (grasp.second->getManipulator().getJoints() < (U32)pacman::RobotUIBK::JOINTS)
		throw Message(Message::LEVEL_ERROR, "BhamGraspImpl::convert(): invalid number of joints");

	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		RobotUIBK::Config configDst;

		// configuration
		const Manipulator::Config configSrc(grasp.second->getManipulator().getConfig(i.config));
		// KukaLWR
		for (std::uintptr_t j = 0; j < grasp.second->getManipulator().getArmJoints(); ++j)
			configDst.arm.c[j] = (float_t)configSrc.jc[j];
		// ShunkDexHand
		const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
		configDst.hand.middle[0] = (float_t)configSrc.jc[offset + 0];
		configDst.hand.middle[1] = (float_t)configSrc.jc[offset + 1];
		configDst.hand.left[0] = (float_t)configSrc.jc[offset + 3];
		configDst.hand.left[1] = (float_t)configSrc.jc[offset + 4];
		configDst.hand.right[0] = (float_t)configSrc.jc[offset + 6];
		configDst.hand.right[1] = (float_t)configSrc.jc[offset + 7];
		configDst.hand.rotation = (float_t)configSrc.jc[offset + 2];

		dst.push_back(configDst);
	}
}

void BhamGraspImpl::convert(const RobotUIBK::Config::Seq& src, ::grasp::RobotState::List& dst) const {
	if (grasp.second->getManipulator().getJoints() < (U32)pacman::RobotUIBK::JOINTS)
		throw Message(Message::LEVEL_ERROR, "BhamGraspImpl::convert(): invalid number of joints");

	dst.clear();
	for (auto i: src) {
		::grasp::RobotState configDst(*grasp.second->getManipulator().getController());
		::grasp::Manipulator::Config config;

		// KukaLWR
		for (std::uintptr_t j = 0; j < grasp.second->getManipulator().getArmJoints(); ++j)
			config.jc[j] = (Real)i.arm.c[j];
		// ShunkDexHand
		const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
		config.jc[offset + 0] = (Real)i.hand.middle[0];
		config.jc[offset + 1] = (Real)i.hand.middle[1];
		config.jc[offset + 3] = (Real)i.hand.left[0];
		config.jc[offset + 4] = (Real)i.hand.left[1];
		config.jc[offset + 6] = (Real)i.hand.right[0];
		config.jc[offset + 7] = (Real)i.hand.right[1];
		config.jc[offset + 2] = (Real)i.hand.rotation;
		config.jc[offset + 5] = -(Real)i.hand.rotation;

		configDst.command = configDst.config = grasp.second->getManipulator().getState(config);
		configDst.ftSensor.setToDefault();
		dst.push_back(configDst);
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

//convert(const RobotUIBK::Config::Seq& src, ::grasp::RobotState::List& dst) const {
void pacman::save(const std::string& path, const RobotUIBK::Config::Seq& trajectory) {

    // open a file
	std::ofstream file(path);
	if (!file.good())
		throw Message(Message::LEVEL_CRIT, "pacman::save(): could not open '%s' file!", path.c_str());

    file << "#robot configuration: hand[0]\thand[1]\t...\thand[6]\tarm[0]\tarm[1]\t...\tarm[6]\n";
    for (U32 i = 0; i < trajectory.size(); ++i) {
        for (U32 i = 0; i<ShunkDexHand::JOINTS; i++)
            file << trajectory[i].hand.c[i] << "\t";
        for (U32 i = 0; i<KukaLWR::JOINTS; i++)
            file << trajectory[i].arm.c[i] << "\t";
        file << std::endl;
    }
    file.close();
}

void pacman::load(const std::string& path, RobotUIBK::Config::Seq& trajectory) {
	// open a file
	std::ifstream file(path);
	if (!file.good())
		throw Message(Message::LEVEL_CRIT, "pacman::load(): '%s' not found!", path.c_str());

    RobotUIBK::Config robot_c;

	const size_t dataSize = ShunkDexHand::JOINTS + KukaLWR::JOINTS;
	std::vector<float_t> conf (dataSize, 0);

	// parse text file
	const char* DELIM = " \t,;:[]#";
	for (std::string line; !file.eof() && std::getline(file, line); )
		if (!line.empty()&&(line[0]!='#')) {
			char *token = std::strtok(const_cast<char*>(line.c_str()), DELIM);

			for (size_t index = 0; (token = std::strtok(NULL, DELIM)) != NULL && index < dataSize; ++index) {
				conf[index] = float_t(atof(token));
			}
            memcpy(robot_c.hand.c, &conf[0], sizeof(float_t)*ShunkDexHand::JOINTS);
            memcpy(robot_c.arm.c, &conf[ShunkDexHand::JOINTS], sizeof(float_t)*KukaLWR::JOINTS);
            trajectory.push_back(robot_c);
		}
    file.close();
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
