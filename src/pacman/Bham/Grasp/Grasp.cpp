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

	// Initialise data collection
	getTrialData().insert(TrialData::Map::value_type("pacman", TrialData(*robot->getController())));
	currentDataPtr = getTrialData().begin();
	scene.getOpenGL(currentDataPtr->second.openGL);

	return true;
}

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const RobotUIBK::Config::Seq& trajectory) {
	convert(points, currentDataPtr->second.pointCloud);
	convert(trajectory, currentDataPtr->second.approachAction);
	grasp.second->add(id, currentDataPtr->second.approachAction, currentDataPtr->second.manipAction, currentDataPtr->second.pointCloud);
	renderTrialData(currentDataPtr);
}

void BhamGraspImpl::remove(const std::string& id) {
	grasp.second->getDataMap().erase(id);
}

void BhamGraspImpl::list(std::vector<std::string>& idSeq) const {
	idSeq.clear();
	idSeq.reserve(grasp.second->getDataMap().size());
	for (auto i: grasp.second->getDataMap())
		idSeq.push_back(i.first);
}

void BhamGraspImpl::estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories) {
	// transform data
	convert(points, currentDataPtr->second.pointCloud);
	targetDataPtr = getTrialData().end();
	
	// compute grip configurations
	grasp.second->findGrip(currentDataPtr->second.pointCloud, currentDataPtr->second.pointCloud, graspPoses);
	//grasp.second->findGripClusters(graspPoses, graspClusters);
		
	// prepare model trajectory
	std::deque<Manipulator::Pose> model;
	for (auto i: grasp.second->getDataMap().begin()->second.approach)
		model.push_front(i);
	golem::Mat34 trnInv;
	trnInv.setInverse(model.back());

	// transform trajectories
	trajectories.clear();
	for (auto i: graspPoses) {
		Trajectory trajectory;
		golem::Mat34 trn;
		trn.multiply(i.toMat34(), trnInv);
		
		for (auto j: model) {
			ShunkDexHand::Pose pose;
			convert(j, pose.config);
			golem::Mat34 frame;
			frame.multiply(trn, j);
			frame.p.get(&pose.pose.p.x);
			frame.R.setRow33(&pose.pose.R.m11);
			trajectory.trajectory.push_back(pose);
		}
		convert(i, trajectory.trajectory.back().config); // overwrite grip configuration (the last waypoint)
		trajectory.likelihood = (float_t)i.likelihood.product;

		trajectories.push_back(trajectory);
	}
	
	// finish
	graspMode = GRASP_MODE_GRIP;
	graspClusterPtr = graspClusterSolutionPtr = 0;
	targetDataPtr = currentDataPtr;
	//printGripInfo();
}

void BhamGraspImpl::spin() {
	try {
		// main loop
		for (;;) {
			try {
				const int key = waitKey();
				// run function
				function(currentDataPtr, key);
			}
			catch (const Cancel& cancel) {
				context.write("%s\n", cancel.what());
			}
			catch (const golem::Message& msg) {
				context.write("%s\n", msg.str().c_str());
			}
			catch (const std::exception& ex) {
				context.write("%s\n", ex.what());
			}
		}
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

	grasp::TrialData::ImportDesc importDesc = this->importDesc;
	importDesc.frames.clear();
	TrialData::process(context, importDesc, dst);
}

void BhamGraspImpl::convert(const ::grasp::Manipulator::Config& src, ShunkDexHand::Config& dst) const {
	const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
	dst.middle[0] = (float_t)src.jc[offset + 0];
	dst.middle[1] = (float_t)src.jc[offset + 1];
	dst.left[0] = (float_t)src.jc[offset + 3];
	dst.left[1] = (float_t)src.jc[offset + 4];
	dst.right[0] = (float_t)src.jc[offset + 6];
	dst.right[1] = (float_t)src.jc[offset + 7];
	dst.rotation = (float_t)src.jc[offset + 2];
}

void BhamGraspImpl::convert(const ShunkDexHand::Config& src, ::grasp::Manipulator::Config& dst) const {
	const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
	dst.jc[offset + 0] = (Real)src.middle[0];
	dst.jc[offset + 1] = (Real)src.middle[1];
	dst.jc[offset + 3] = (Real)src.left[0];
	dst.jc[offset + 4] = (Real)src.left[1];
	dst.jc[offset + 6] = (Real)src.right[0];
	dst.jc[offset + 7] = (Real)src.right[1];
	dst.jc[offset + 2] = (Real)src.rotation;
	dst.jc[offset + 5] = -(Real)src.rotation;
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
		convert(configSrc, configDst.hand);

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
		convert(i.hand, config);

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

void pacman::save(const std::string& path, const RobotUIBK::Config::Seq& trajectory) {
    // open a file
	std::ofstream file(path);
	if (!file.good())
		throw Message(Message::LEVEL_CRIT, "pacman::save(): could not open '%s' file!", path.c_str());

	std::stringstream str;
	str << "#";
	for (std::uintptr_t i = 0; i < KukaLWR::JOINTS; ++i)
		str << '\t' << "arm_" << i;
	for (std::uintptr_t i = 0; i < ShunkDexHand::JOINTS; ++i)
		str << '\t' << "hand_" << i;
	file << str.str() << std::endl;

	for (auto i: trajectory) {
		for (size_t j = 0; j < RobotUIBK::JOINTS; ++j)
			file << (j < KukaLWR::JOINTS ? i.arm.c[j] : i.hand.c[j - KukaLWR::JOINTS]) << '\t';
		file << std::endl;
	}
}

void pacman::load(const std::string& path, RobotUIBK::Config::Seq& trajectory) {
	// open a file
	std::ifstream file(path);
	if (!file.good())
		throw Message(Message::LEVEL_CRIT, "pacman::load(): '%s' not found!", path.c_str());

	// parse text file
	const char* DELIM = " \t,;:";
	trajectory.clear();
	for (std::string line; !file.eof() && std::getline(file, line); )
		if (!line.empty()&&(line[0]!='#')) {
			RobotUIBK::Config config;

			char *token = std::strtok(const_cast<char*>(line.c_str()), DELIM);
			for (size_t index = 0; token != NULL && index < RobotUIBK::JOINTS; token = std::strtok(NULL, DELIM), ++index)
				(index < KukaLWR::JOINTS ? config.arm.c[index] : config.hand.c[index - KukaLWR::JOINTS]) = float_t(atof(token));
            
			trajectory.push_back(config);
		}
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
