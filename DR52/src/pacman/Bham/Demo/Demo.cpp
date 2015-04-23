#include <pacman/Bham/Demo/Demo.h>
#include <Grasp/Core/Import.h>
#include <Grasp/App/Player/Data.h>
#include <Golem/Phys/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

data::Data::Ptr Demo::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new Demo::Data(context));
	static_cast<Demo::Data*>(data.get())->create(*this);
	return data;
}

Demo::Data::Data(golem::Context &context) : grasp::Player::Data(context), owner(nullptr) {
}

void Demo::Data::create(const Desc& desc) {
	Player::Data::create(desc);

	modelVertices.clear();
	modelTriangles.clear();
}

void Demo::Data::setOwner(Manager* owner) {
	grasp::Player::Data::setOwner(owner);
	this->owner = is<Demo>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
	dataName = this->owner->dataName;
}

void Demo::Data::createRender() {
	Player::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->csRenderer);
		owner->modelRenderer.reset();
		owner->modelRenderer.setColour(owner->modelColourSolid);
		owner->modelRenderer.addSolid(modelVertices.data(), (U32)modelVertices.size(), modelTriangles.data(), (U32)modelTriangles.size());
		owner->modelRenderer.setColour(owner->modelColourWire);
		owner->modelRenderer.addWire(modelVertices.data(), (U32)modelVertices.size(), modelTriangles.data(), (U32)modelTriangles.size());
	}
}

void Demo::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const data::Handler::Map& handlerMap) {
	data::Data::load(prefix, xmlcontext, handlerMap);

	try {
		dataName.clear();
		golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (dataName.length() > 0) {
			FileReadStream frs((prefix + sepName + dataName).c_str());
			frs.read(modelVertices, modelVertices.end());
			frs.read(modelTriangles, modelTriangles.end());
		}
	}
	catch (const std::exception&) {

	}
}

void Demo::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	data::Data::save(prefix, xmlcontext);

	if (dataName.length() > 0) {
		golem::XMLData("data_name", const_cast<std::string&>(dataName), xmlcontext, true);
		FileWriteStream fws((prefix + sepName + dataName).c_str());
		fws.write(modelVertices.begin(), modelVertices.end());
		fws.write(modelTriangles.begin(), modelTriangles.end());
	}
}

//------------------------------------------------------------------------------

void Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler_trj", modelHandlerTrj, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_trj", modelItemTrj, xmlcontext->getContextFirst("model"));
	modelScanPose.xmlData(xmlcontext->getContextFirst("model scan_pose"));
	golem::XMLData(modelColourSolid, xmlcontext->getContextFirst("model colour solid"));
	golem::XMLData(modelColourWire, xmlcontext->getContextFirst("model colour wire"));

	golem::XMLData("sensor", graspSensorForce, xmlcontext->getContextFirst("grasp"));
	golem::XMLData(graspThresholdForce, xmlcontext->getContextFirst("grasp threshold"));
	golem::XMLData("event_time_wait", graspEventTimeWait, xmlcontext->getContextFirst("grasp"));
	golem::XMLData("close_duration", graspCloseDuration, xmlcontext->getContextFirst("grasp"));
	graspPoseOpen.xmlData(xmlcontext->getContextFirst("grasp pose_open"));
	graspPoseClosed.xmlData(xmlcontext->getContextFirst("grasp pose_closed"));

	golem::XMLData("camera", objectCamera, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler_scan", objectHandlerScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler", objectHandler, xmlcontext->getContextFirst("object"));
	golem::XMLData("item_scan", objectItemScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("item", objectItem, xmlcontext->getContextFirst("object"));
	objectScanPoseSeq.clear();
	XMLData(objectScanPoseSeq, objectScanPoseSeq.max_size(), xmlcontext->getContextFirst("object"), "scan_pose");
	objectFrameAdjustment.load(xmlcontext->getContextFirst("object frame_adjustment"));
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : 
	Player(scene), modelCamera(nullptr), modelHandler(nullptr), graspSensorForce(nullptr), objectCamera(nullptr), objectHandlerScan(nullptr), objectHandler(nullptr)
{}

pacman::Demo::~Demo() {
}

grasp::Camera* pacman::Demo::getWristCamera() const
{
	const std::string id("OpenNI+OpenNI");
	grasp::Sensor::Map::const_iterator i = sensorMap.find(id);
	if (i == sensorMap.end())
	{
		context.write("%s was not found\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	grasp::Camera* camera = grasp::is<grasp::Camera>(i);

	// want the wrist-mounted camera
	if (!camera->hasVariableMounting())
	{
		context.write("%s is a static camera\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	return camera;
}

golem::Mat34 pacman::Demo::getWristPose() const
{
	const golem::U32 wristJoint = 6; // @@@
	grasp::ConfigMat34 pose;
	getPose(wristJoint, pose);
	return pose.w;
}

void pacman::Demo::gotoWristPose(const golem::Mat34& w)
{
	const golem::Mat34 R = controller->getChains()[armInfo.getChains().begin()]->getReferencePose();
	const golem::Mat34 wR = w * R;

	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);

	golem::Controller::State::Seq trajectory;
	const grasp::RBDist err = findTrajectory(begin, nullptr, &wR, trajectoryDuration, trajectory);

	sendTrajectory(trajectory);
	controller->waitForEnd();
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));
}


void pacman::Demo::rotateObjectInHand()
{
	auto showPose = [&](const std::string& description, const golem::Mat34& m) {
		context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};

	const double maxstep = 45.0;
	double angstep = 30.0;
	int rotType;
	for (;;)
	{
		std::ostringstream os;
		os << "rotation: Clockwise or Anticlockwise screw, Up or Down, Left or Right; STEP(" << angstep << " deg):+/-";
		rotType = option("CAUDLR+-", os.str().c_str());
		if (rotType == '+')
		{
			angstep *= 2;
			if (angstep > maxstep) angstep = maxstep;
			continue;
		}
		if (rotType == '-')
		{
			angstep /= 2;
			continue;
		}
		break;
	}

	golem::Vec3 rotAxis(golem::Vec3::zero());

	switch(rotType)
	{
	case 'C':
		rotAxis.z = 1.0;
		break;
	case 'A':
		rotAxis.z = 1.0;
		angstep = -angstep;
		break;
	case 'U':
		rotAxis.y = 1.0;
		break;
	case 'D':
		rotAxis.y = 1.0;
		angstep = -angstep;
		break;
	case 'L':
		rotAxis.x = 1.0;
		break;
	case 'R':
		rotAxis.x = 1.0;
		angstep = -angstep;
		break;
	}

	golem::Mat33 rot(golem::Mat33(angstep / 180.0 * golem::REAL_PI, rotAxis));

	golem::Mat34 pose = getWristPose();
	showPose("before", pose);

	pose.R = pose.R * rot;
	showPose("commanded", pose);

	gotoWristPose(pose);
	showPose("final wrist", getWristPose());

	grasp::ConfigMat34 cp;
	getPose(0, cp);
	context.debug("c1=\"%f\" c2=\"%f\" c3=\"%f\" c4=\"%f\" c5=\"%f\" c6=\"%f\" c7=\"%f\"\n", cp.c[0], cp.c[1], cp.c[2], cp.c[3], cp.c[4], cp.c[5], cp.c[6], cp.c[7]);
		
	recordingStart(dataCurrentPtr->first, recordingLabel, false);
	context.write("taken snapshot\n");
}

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::Demo::Desc."));

	// create object
	Player::create(desc); // throws

	dataName = desc.dataName;

	grasp::Sensor::Map::const_iterator modelCameraPtr = sensorMap.find(desc.modelCamera);
	modelCamera = modelCameraPtr != sensorMap.end() ? is<Camera>(modelCameraPtr->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	grasp::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;
	grasp::data::Handler::Map::const_iterator modelHandlerTrjPtr = handlerMap.find(desc.modelHandlerTrj);
	modelHandlerTrj = modelHandlerTrjPtr != handlerMap.end() ? modelHandlerTrjPtr->second.get() : nullptr;
	if (!modelHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model trajectory handler: %s", desc.modelHandlerTrj.c_str());
	modelItemTrj = desc.modelItemTrj;
	modelScanPose = desc.modelScanPose;
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	grasp::Sensor::Map::const_iterator graspSensorForcePtr = sensorMap.find(desc.graspSensorForce);
	graspSensorForce = graspSensorForcePtr != sensorMap.end() ? is<FT>(graspSensorForcePtr->second.get()) : nullptr;
	if (!graspSensorForce)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown grasp F/T sensor: %s", desc.graspSensorForce.c_str());
	graspThresholdForce = desc.graspThresholdForce;
	graspEventTimeWait = desc.graspEventTimeWait;
	graspCloseDuration = desc.graspCloseDuration;
	graspPoseOpen = desc.graspPoseOpen;
	graspPoseClosed = desc.graspPoseClosed;

	grasp::Sensor::Map::const_iterator objectCameraPtr = sensorMap.find(desc.objectCamera);
	objectCamera = objectCameraPtr != sensorMap.end() ? is<Camera>(objectCameraPtr->second.get()) : nullptr;
	if (!objectCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object capture camera: %s", desc.objectCamera.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerScanPtr = handlerMap.find(desc.objectHandlerScan);
	objectHandlerScan = objectHandlerScanPtr != handlerMap.end() ? objectHandlerScanPtr->second.get() : nullptr;
	if (!objectHandlerScan)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object (scan) data handler: %s", desc.objectHandlerScan.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerPtr = handlerMap.find(desc.objectHandler);
	objectHandler = objectHandlerPtr != handlerMap.end() ? objectHandlerPtr->second.get() : nullptr;
	if (!objectHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object (process) data handler: %s", desc.objectHandler.c_str());
	objectItemScan = desc.objectItemScan;
	objectItem = desc.objectItem;
	objectScanPoseSeq = desc.objectScanPoseSeq;
	objectFrameAdjustment = desc.objectFrameAdjustment;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  Z                                       menu SZ Tests\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo, (E)stimate model pose, (A)ttach object, add (M)odel, test (Query)...";
	}));

	// model pose estimation
	menuCmdMap.insert(std::make_pair("PE", [=]() {
		// run robot
		gotoPose(modelScanPose);
		// block keyboard and mouse interaction
		InputBlock inputBlock(*this);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			to<Data>(dataCurrentPtr)->modelVertices.clear();
			to<Data>(dataCurrentPtr)->modelTriangles.clear();
		}
		// obtain snapshot handler
		data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(modelCamera->getSnapshotHandler());
		if (handlerSnapshotPtr == handlerMap.end())
			throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", modelCamera->getSnapshotHandler().c_str());
		// capture and insert data
		data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
		if (!capture)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", modelCamera->getSnapshotHandler().c_str());
		data::Item::Map::iterator ptr;
		data::Item::Ptr item = capture->capture(*modelCamera, [&](const grasp::TimeStamp*) -> bool { return true; });
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// generate features
		data::Transform* transform = is<data::Transform>(handlerSnapshotPtr);
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", modelCamera->getSnapshotHandler().c_str());
		data::Item::List list;
		list.insert(list.end(), ptr);
		item = transform->transform(list);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// estimate pose
		data::Model* model = is<data::Model>(ptr);
		if (!model)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Model interface", ptr->first.c_str());
		grasp::ConfigMat34 robotPose;
		golem::Mat34 modelPose;
		Vec3Seq modelVertices;
		TriangleSeq modelTriangles;
		model->model(robotPose, modelPose, &modelVertices, &modelTriangles);
		// create triangle mesh
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->modelVertices = modelVertices;
			to<Data>(dataCurrentPtr)->modelTriangles = modelTriangles;
		}
		//grasp::Contact3D::Triangle::Seq modelMesh;
		//for (grasp::TriangleSeq::const_iterator j = modelTriangles.begin(); j != modelTriangles.end(); ++j)
		//	to<Data>(dataCurrentPtr)->modelMesh.push_back(Contact3D::Triangle(modelVertices[j->t1], modelVertices[j->t3], modelVertices[j->t2]));
		// done
		context.write("Done!\n");
	}));

	// model attachement
	menuCtrlMap.insert(std::make_pair("PA", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (C)apture/(L)oad object...";
	}));
	menuCmdMap.insert(std::make_pair("PAC", [=]() {
		// grasp and scan object
		grasp::data::Item::Map::iterator ptr = objectGraspAndCapture();
		// compute features and add to data bundle
		objectProcess(ptr);

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("PAL", [=]() {
		// cast to data::Import
		data::Import* import = is<data::Import>(objectHandlerScan);
		if (!import)
			throw Cancel("Object handler does not implement data::Import");

		// replace current handlers
		UI::removeCallback(*this, getCurrentHandler());
		UI::addCallback(*this, import);

		// load/import item
		readPath("Enter file path: ", dataImportPath, import->getFileTypes());
		data::Item::Ptr item = import->import(dataImportPath);

		// compute reference frame and adjust object frame
		data::Location3D* location = is<data::Location3D>(item.get());
		if (!location)
			throw Cancel("Object handler does not implement data::Location3D");
		Vec3Seq points, pointsTrn;
		for (size_t i = 0; i < location->getNumOfLocations(); ++i)
			points.push_back(location->getLocation(i));
		pointsTrn.resize(points.size());
		Mat34 frame = RBPose::createFrame(points), frameInv;
		frameInv.setInverse(frame);
		context.write("Press a key to: accept <Enter>, (%s/%s) to adjust position, (%s/%s) to adjust orientation... ",
			objectFrameAdjustment.linKeysLarge.c_str(), objectFrameAdjustment.linKeysSmall.c_str(), objectFrameAdjustment.angKeysLarge.c_str(), objectFrameAdjustment.angKeysSmall.c_str());
		for (bool accept = false;;) {
			const Mat34 trn = frame * frameInv; // frame = trn * frameInit, trn = frame * frameInit^-1
			for (size_t i = 0; i < points.size(); ++i)
				trn.multiply(pointsTrn[i], points[i]);
			{
				golem::CriticalSectionWrapper csw(csRenderer);
				objectRenderer.reset();
				if (accept) break;
				for (size_t i = 0; i < pointsTrn.size(); ++i)
					objectRenderer.addPoint(pointsTrn[i], objectFrameAdjustment.colourSolid);
				objectRenderer.addAxes3D(frame, objectFrameAdjustment.frameSize);
			}
			const int key = waitKey();
			switch (key) {
			case 27: throw Cancel("\nCancelled");
			case 13: context.write(")<Enter>(\n"); accept = true; break;
			default: objectFrameAdjustment.adjust(key, frame);
			}
		}
		// transform
		location->transform(frame * frameInv);

		// add item to data bundle
		data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(objectItemScan);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(objectItemScan, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}

		// compute features and add to data bundle
		objectProcess(ptr);
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("PM", [=]() {
		data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(objectItem);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Cancel("Object item has not been created - run attach object");

		RenderBlock renderBlock(*this);

		// compute reference frame and adjust object frame
		data::Location3D* location = is<data::Location3D>(ptr);
		if (!location)
			throw Cancel("Object handler does not implement data::Location3D");
		Vec3Seq points, pointsTrn;
		for (size_t i = 0; i < location->getNumOfLocations(); ++i)
			points.push_back(location->getLocation(i));
		pointsTrn.resize(points.size());
		Mat34 frame = forwardTransformArm(lookupState()), frameInv;
		frameInv.setInverse(frame);
		context.write("Press a key to: accept <Enter>... ");
		// attach object to the robot's end-effector
		for (bool accept = false;;) {
			const Mat34 trn = frame * frameInv; // frame = trn * frameInit, trn = frame * frameInit^-1
			for (size_t i = 0; i < points.size(); ++i)
				trn.multiply(pointsTrn[i], points[i]);
			{
				golem::CriticalSectionWrapper csw(csRenderer);
				objectRenderer.reset();
				if (accept) break;
				for (size_t i = 0; i < pointsTrn.size(); ++i)
					objectRenderer.addPoint(pointsTrn[i], objectFrameAdjustment.colourSolid);
			}
			const int key = waitKey(20);
			switch (key) {
			case 27: throw Cancel("\nCancelled");
			case 13: context.write(")<Enter>(\n"); accept = true; break;
			}
			frame = forwardTransformArm(lookupState());
		}
		// transform
		location->transform(frame * frameInv);

		// add trajectory waypoint
		{
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItemTrj);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItemTrj, modelHandlerTrj->create()));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		data::Trajectory* trajectory = is<data::Trajectory>(ptr);
		if (!trajectory)
			throw Cancel("Trajectory handler does not implement data::Trajectory");
		// add current state
		Controller::State::Seq seq = trajectory->getWaypoints();
		seq.push_back(lookupState());
		trajectory->setWaypoints(seq);

		context.write("Done!\n");
	}));

	// main demo
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// estimate pose
		if (to<Data>(dataCurrentPtr)->modelVertices.empty() || to<Data>(dataCurrentPtr)->modelTriangles.empty())
			menuCmdMap["PE"]();

		// run demo
		for (;;) {
			// grasp and scan object
			grasp::data::Item::Map::iterator ptr = objectGraspAndCapture();
			// compute features and add to data bundle
			(void)objectProcess(ptr);
		}
	}));

	menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (R)otate, (N)udge, (O)bjectGraspAndCapture...";
	}));

	menuCmdMap.insert(std::make_pair("ZO", [=]() {
		context.write("Testing objectGraspAndCapture()\n");
		objectGraspAndCapture();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZR", [=]() {
		context.write("rotate object in hand\n");
		rotateObjectInHand();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZN", [=]() {
		context.write("nudge wrist\n");

		auto showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};

		double s = 5; // 5cm step
		int dirCode;
		for (;;)
		{
			std::ostringstream os;
			os << "up:7 down:1 left:4 right:6 back:8 front:2 null:0 STEP(" << s << " cm):+/-";
			dirCode = option("7146820+-", os.str().c_str());
			if (dirCode == '+')
			{
				s *= 2;
				if (s > 20.0) s = 20.0;
				continue;
			}
			if (dirCode == '-')
			{
				s /= 2;
				continue;
			}
			break;
		}
		s /= 100.0; // cm -> m

		golem::Vec3 nudge(golem::Vec3::zero());
		switch (dirCode)
		{
		case '7': // up
			nudge.z = s;
			break;
		case '1': // down
			nudge.z = -s;
			break;
		case '4': // left
			nudge.y = -s;
			break;
		case '6': // right
			nudge.y = s;
			break;
		case '8': // back
			nudge.x = -s;
			break;
		case '2': // front
			nudge.x = s;
			break;
		};

		golem::Mat34 pose;

		grasp::Camera* camera = getWristCamera();
		pose = camera->getFrame();
		showPose("camera before", pose);

		pose = getWristPose();
		showPose("before", pose);

		pose.p = pose.p + nudge;
		showPose("commanded", pose);

		//const golem::Mat34 goalPose = activeSense->computeGoal(pose, camera);
		//gotoPoseWS(goalPose, 0.1, 0.1);

		gotoWristPose(pose);

		showPose("final wrist", getWristPose());
		pose = camera->getFrame();
		showPose("final camera", pose);

		grasp::ConfigMat34 cp;
		getPose(0, cp);
		context.debug("c1=\"%f\" c2=\"%f\" c3=\"%f\" c4=\"%f\" c5=\"%f\" c6=\"%f\" c7=\"%f\"\n", cp.c[0], cp.c[1], cp.c[2], cp.c[3], cp.c[4], cp.c[5], cp.c[6], cp.c[7]);

		context.write("Done!\n");
	}));
}
//------------------------------------------------------------------------------

// move robot to grasp open pose, wait for force event, grasp object (closed pose), move through scan poses and capture object, add as objectScan
// 1. move robot to grasp open pose
// 2. wait for force event; reset bias and wait for change > threshold (set in xml)
// 3. grasp object (closed pose - generate in virtual env; strong enough for both plate and cup; only fingers should move)
// full hand grasp, fingers spread out; thumb in centre
// 4. move through scan poses and capture object, add as objectScan
// return ptr to Item
grasp::data::Item::Map::iterator pacman::Demo::objectGraspAndCapture() {
	data::Item::Ptr item;

	// TODO 1-4:
	//attach object
	//add model

	// Finally: insert object scan, remove old one
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(csData);
	to<Data>(dataCurrentPtr)->itemMap.erase(objectItemScan);
	grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(objectItemScan, item));
	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	return ptr;
}

//------------------------------------------------------------------------------

// Process object image and add to data bundle
void pacman::Demo::objectProcess(grasp::data::Item::Map::iterator ptr) {
	// generate features
	data::Transform* transform = is<data::Transform>(objectHandler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", objectHandler->getID().c_str());

	data::Item::List list;
	list.insert(list.end(), ptr);
	data::Item::Ptr item = transform->transform(list);

	// insert processed object, remove old one
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(csData);
	to<Data>(dataCurrentPtr)->itemMap.erase(objectItem);
	(void)to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(objectItem, item));
	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
}

//------------------------------------------------------------------------------

void pacman::Demo::render() const {
	Player::render();
	golem::CriticalSectionWrapper cswRenderer(csRenderer);
	modelRenderer.render();
	objectRenderer.render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
