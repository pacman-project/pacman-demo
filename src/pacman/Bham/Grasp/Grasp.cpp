#include <pacman/Bham/Control/Control.h>
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

	// Initialise data collection
	getData().insert(Data::Map::value_type(data->name, data->clone()));
	currentDataPtr = getData().begin();
	scene.getOpenGL(to<Data>(currentDataPtr)->openGL);

	scene.getHelp().insert(Scene::StrMapVal("100", "  A                                       PaCMan operations\n"));

	return true;
}

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const RobotUIBK::Config::Seq& trajectory) {
	auto ptr = getPtr<Data>(currentDataPtr);
	if (ptr == nullptr)
		throw Message(Message::LEVEL_ERROR, "BhamGraspImpl::add(): invalid current data pointer");

	Cloud::PointSeq& cloud = ptr->points[Cloud::LABEL_OBJECT];
	convert(points, cloud);
	convert(trajectory, ptr->actionApproach);
	grasp.second->add(id, ptr->actionApproach, ptr->actionManip, cloud);
	
	renderData(currentDataPtr);
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
	auto ptr = getPtr<Data>(currentDataPtr);
	if (ptr == nullptr)
		throw Message(Message::LEVEL_ERROR, "BhamGraspImpl::estimate(): invalid current data pointer");

	// transform data
	Cloud::PointSeq& cloud = ptr->points[Cloud::LABEL_OBJECT];
	convert(points, cloud);
	targetDataPtr = getData().end();
	
	// compute grip configurations
	grasp.second->findGrip(cloud, cloud, ptr->graspPoses);
	grasp.second->findGripClusters(ptr->graspPoses, ptr->graspClusters);
	
	// prepare model trajectory
	std::deque<Manipulator::Pose> model;
	for (auto i: grasp.second->getDataMap().begin()->second.approach)
		model.push_front(i);
	golem::Mat34 trnInv;
	trnInv.setInverse(model.back());

	// move trajectory frame from the arm TCP to the hand frame
	const golem::Controller* pController = robot->getController();
	if (pController->getControllers().size() >= 2) {
		const golem::Controller* pHand = pController->getControllers()[1]; // assuming that the second controller is the hand
		trnInv.multiply(pHand->getGlobalPose(), trnInv);
	}

	// transform trajectories
	trajectories.clear();
	for (auto i: ptr->graspPoses) {
		Trajectory trajectory;
		golem::Mat34 trn;
		trn.multiply(i.toMat34(), trnInv);
		
		for (auto j: model) {
			SchunkDexHand::Pose pose;
			convert(j, pose.config);
			golem::Mat34 frame;
			frame.multiply(trn, j);
			frame.p.get(&pose.pose.p.x);
			frame.R.getRow33(&pose.pose.R.m11);
			trajectory.trajectory.push_back(pose);
		}
		convert(i, trajectory.trajectory.back().config); // overwrite grip configuration (the last waypoint)
		trajectory.likelihood = (float_t)i.likelihood.product;

		trajectories.push_back(trajectory);
	}

	// finish
	ptr->resetDataPointers();
	graspMode = GRASP_MODE_GRIP;
	targetDataPtr = currentDataPtr;
	printGripInfo();
	renderData(currentDataPtr);
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

void BhamGraspImpl::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'A':
	{
		switch (waitKey("IE", "Press a key to (I)mport/(E)xport data...")) {
		case 'I':
		{
			const int key = waitKey("PT", "Press a key to import (P)oint cloud/(T)trajectory...");
			// export data
			std::string path = data->dir;
			readString("Enter file path: ", path);
			// point cloud
			if (key == 'P') {
				context.write("Importing point cloud from: %s\n", path.c_str());
				Point3D::Seq seq;
				load(path, seq);
				convert(seq, to<Data>(dataPtr)->points[Cloud::LABEL_OBJECT]);
				renderData(dataPtr);
			}
			// appproach trajectory
			if (key == 'T') {
				context.write("Importing approach trajectory from: %s\n", path.c_str());
				RobotUIBK::Config::Seq seq;
				load(path, seq);
				convert(seq, to<Data>(dataPtr)->actionApproach);
			}
			break;
		}
		case 'E':
		{
			const int key = waitKey("PT", "Press a key to export (P)oint cloud/(T)trajectory...");
			// export data
			std::string path = data->dir;
			readString("Enter file path: ", path);
			// point cloud
			if (key == 'P') {
				context.write("Exporting point cloud to: %s\n", path.c_str());
				Point3D::Seq seq;
				convert(to<Data>(dataPtr)->points[Cloud::LABEL_OBJECT], seq);
				save(path, seq);
			}
			// appproach trajectory
			if (key == 'T') {
				context.write("Exporting approach trajectory to: %s\n", path.c_str());
				RobotUIBK::Config::Seq seq;
				convert(to<Data>(dataPtr)->actionApproach, seq);
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

void BhamGraspImpl::convert(const ::grasp::Cloud::PointSeq& src, Point3D::Seq& dst) const {
	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		Point3D point;
		point.position.x = (float_t)i.x;
		point.position.y = (float_t)i.y;
		point.position.z = (float_t)i.z;
		point.normal.x =  (float_t)i.normal_x;
		point.normal.y =  (float_t)i.normal_y;
		point.normal.z =  (float_t)i.normal_z;
		point.colour.r = (uint8_t)i.r;
		point.colour.g = (uint8_t)i.g;
		point.colour.b = (uint8_t)i.b;
		point.colour.a = (uint8_t)i.a;
		dst.push_back(point);
	}
}

void BhamGraspImpl::convert(const Point3D::Seq& src, ::grasp::Cloud::PointSeq& dst) const {
	dst.resize(0);
	dst.reserve(src.size());
	for (auto i: src) {
		Cloud::Point point;
		point.x = (float)i.position.x;
		point.y = (float)i.position.y;
		point.z = (float)i.position.z;
		point.normal_x = (float)i.normal.x;
		point.normal_y = (float)i.normal.y;
		point.normal_z = (float)i.normal.z;
		point.r = (uint8_t)i.colour.r;
		point.g = (uint8_t)i.colour.g;
		point.b = (uint8_t)i.colour.b;
		point.a = (uint8_t)i.colour.a;
		dst.push_back(point);
	}

	Cloud::curvature(context, cloudDesc.curvature, dst);
	Cloud::nanRem(context, dst, Cloud::isNanXYZNormalCurvature<Cloud::Point>);
}

void BhamGraspImpl::convert(const ::grasp::Manipulator::Config& src, SchunkDexHand::Config& dst) const {
	const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
	BhamControl::configToPacman(src.jc + grasp.second->getManipulator().getArmJoints(), dst);
}

void BhamGraspImpl::convert(const SchunkDexHand::Config& src, ::grasp::Manipulator::Config& dst) const {
	const std::uintptr_t offset = grasp.second->getManipulator().getArmJoints();
	BhamControl::configToGolem(src, dst.jc + grasp.second->getManipulator().getArmJoints());
}

void BhamGraspImpl::convert(const ::grasp::RobotState::List& src, RobotUIBK::Config::Seq& dst) const {
	if (grasp.second->getManipulator().getJoints() < (U32)pacman::RobotUIBK::Config::JOINTS)
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
		// SchunkDexHand
		convert(configSrc, configDst.hand);

		dst.push_back(configDst);
	}
}

void BhamGraspImpl::convert(const RobotUIBK::Config::Seq& src, ::grasp::RobotState::List& dst) const {
	if (grasp.second->getManipulator().getJoints() < (U32)pacman::RobotUIBK::Config::JOINTS)
		throw Message(Message::LEVEL_ERROR, "BhamGraspImpl::convert(): invalid number of joints");

	dst.clear();
	for (auto i: src) {
		::grasp::RobotState configDst(*grasp.second->getManipulator().getController());
		::grasp::Manipulator::Config config;

		// KukaLWR
		for (std::uintptr_t j = 0; j < grasp.second->getManipulator().getArmJoints(); ++j)
			config.jc[j] = (Real)i.arm.c[j];
		// SchunkDexHand
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
	for (std::uintptr_t i = 0; i < KukaLWR::Config::JOINTS; ++i)
		str << '\t' << "arm_" << i;
	for (std::uintptr_t i = 0; i < SchunkDexHand::Config::JOINTS; ++i)
		str << '\t' << "hand_" << i;
	file << str.str() << std::endl;

	for (auto i: trajectory) {
		for (size_t j = 0; j < RobotUIBK::Config::JOINTS; ++j)
			file << (j < KukaLWR::Config::JOINTS ? i.arm.c[j] : i.hand.c[j - KukaLWR::Config::JOINTS]) << '\t';
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
			for (size_t index = 0; token != NULL && index < RobotUIBK::Config::JOINTS; token = std::strtok(NULL, DELIM), ++index)
				(index < KukaLWR::Config::JOINTS ? config.arm.c[index] : config.hand.c[index - KukaLWR::Config::JOINTS]) = float_t(atof(token));
            
			trajectory.push_back(config);
		}
}

//-----------------------------------------------------------------------------

Context::Ptr context;
Universe::Ptr universe;

BhamGrasp::Ptr BhamGrasp::create(const std::string& path) {
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

	return BhamGrasp::Ptr(pBhamGrasp, [&] (BhamGrasp*) {
		universe.release();
		context.release();
	});
}

//-----------------------------------------------------------------------------
