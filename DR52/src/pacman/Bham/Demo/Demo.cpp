#include <pacman/Bham/Demo/Demo.h>
#include <Golem/Math/Rand.h>
#include <Grasp/Core/Data.h>
#include <Grasp/Grasp/Model.h>
#include <Grasp/Core/Import.h>
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

	drainerPose.setId();
	drainerVertices.clear();
	drainerTriangles.clear();
	drainerModelTriangles.clear();
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
		owner->drainerRenderer.reset();
		owner->drainerRenderer.setColour(owner->drainerColourSolid);
		owner->drainerRenderer.addSolid(drainerVertices.data(), (U32)drainerVertices.size(), drainerTriangles.data(), (U32)drainerTriangles.size());
		owner->drainerRenderer.setColour(owner->drainerColourWire);
		owner->drainerRenderer.addWire(drainerVertices.data(), (U32)drainerVertices.size(), drainerTriangles.data(), (U32)drainerTriangles.size());
	}
}

void Demo::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const data::Handler::Map& handlerMap) {
	data::Data::load(prefix, xmlcontext, handlerMap);

	try {
		dataName.clear();
		golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (dataName.length() > 0) {
			FileReadStream frs((prefix + sepName + dataName).c_str());
			frs.read(drainerPose);
			frs.read(drainerVertices, drainerVertices.end());
			frs.read(drainerTriangles, drainerTriangles.end());
			frs.read(drainerModelTriangles, drainerModelTriangles.end());
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
		fws.write(drainerPose);
		fws.write(drainerVertices.begin(), drainerVertices.end());
		fws.write(drainerTriangles.begin(), drainerTriangles.end());
		fws.write(drainerModelTriangles.begin(), drainerModelTriangles.end());
	}
}

//------------------------------------------------------------------------------

void Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", drainerCamera, xmlcontext->getContextFirst("drainer"));
	golem::XMLData("handler", drainerHandler, xmlcontext->getContextFirst("drainer"));
	golem::XMLData("item", drainerItem, xmlcontext->getContextFirst("drainer"));
	drainerScanPose.xmlData(xmlcontext->getContextFirst("drainer scan_pose"));
	golem::XMLData(drainerColourSolid, xmlcontext->getContextFirst("drainer colour solid"));
	golem::XMLData(drainerColourWire, xmlcontext->getContextFirst("drainer colour wire"));

	this->activeSense->load(xmlcontext);

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("tracker rotate_in_hand");
	XMLData("nstep", tracker_nstep, pxmlcontext);
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : Player(scene), drainerCamera(nullptr), drainerHandler(nullptr) {
}

pacman::Demo::~Demo() {
}

/*************************USEFUL FUNCTIONS FOR ACTIVE SENS*******************************************************/

void pacman::Demo::postprocess(golem::SecTmReal elapsedTime)
{

	Player::postprocess(elapsedTime);
	golem::CriticalSectionWrapper csw(activeSense->getCSViewHypotheses());
	for (pacman::HypothesisSensor::Seq::iterator it = activeSense->getViewHypotheses().begin(); it != activeSense->getViewHypotheses().end(); it++)
	{
		(*it)->draw((*it)->getAppearance(), this->sensorRenderer);
	}
	golem::Mat34 centroidFrame;
	centroidFrame.setId();
	centroidFrame.p = activeSense->getParameters().centroid;
	sensorRenderer.addAxes3D(centroidFrame, golem::Vec3(0.05));

}

bool pacman::Demo::gotoPoseWS(const grasp::ConfigMat34& pose, const Real& linthr, const golem::Real& angthr) {
	// current state
	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);

	// find trajectory
	golem::Controller::State::Seq trajectory;
	grasp::RBDist err = findTrajectory(begin, nullptr, &pose.w, trajectoryDuration, trajectory);
	

	
	if (err.lin >= linthr || err.ang >= angthr)
	{
		return false;
	}

	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));

	return true;
}

bool pacman::Demo::gotoPoseConfig(const grasp::ConfigMat34& config, const Real& linthr, const golem::Real& angthr) {
	// current state
	golem::Controller::State begin = lookupState();
	//context.debug("STATE[1]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
	// target
	golem::Controller::State end = begin;
	end.cpos.set(config.c.data(), config.c.data() + std::min(config.c.size(), (size_t)info.getJoints().size()));
	// find trajectory
	golem::Controller::State::Seq trajectory;
	grasp::RBDist err = findTrajectory(begin, &end, nullptr, trajectoryDuration, trajectory);




	if (err.lin >= linthr || err.ang >= angthr)
	{
		context.write("Error treshold exceeded: lin: %f ang: %f", err.lin, err.ang);
		return false;
	}

	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));

	return true;
}

void pacman::Demo::scanPoseActive(grasp::data::Item::List& scannedImageItems, ScanPoseCommand scanPoseCommand, const std::string itemLabel) {
	data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(to<Sensor>(sensorCurrentPtr)->getSnapshotHandler());
	if (handlerSnapshotPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", to<Sensor>(sensorCurrentPtr)->getSnapshotHandler().c_str());
	data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", to<Sensor>(sensorCurrentPtr)->getSnapshotHandler().c_str());

	this->dataItemLabel = itemLabel;

	Camera* camera = is<Camera>(sensorCurrentPtr);
	const bool isEnabledDeformationMap = camera && camera->getCurrentCalibration()->isEnabledDeformationMap();
	ScopeGuard guard([&]() { if (camera) camera->getCurrentCalibration()->enableDeformationMap(isEnabledDeformationMap); });
	if (camera && camera->getCurrentCalibration()->hasDeformationMap())
		camera->getCurrentCalibration()->enableDeformationMap(option("YN", "Use deformation map (Y/N)...") == 'Y');



	for (bool stop = false; !stop;) {
		stop = scanPoseCommand == nullptr || !scanPoseCommand();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(csData);
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, capture->capture(*to<Camera>(sensorCurrentPtr), [&](const grasp::TimeStamp*) -> bool { return true; })));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
			scannedImageItems.push_back(ptr);
		}
	}

	context.write("Done!\n");
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

	const int rotType = option("CAUDLR", "rotation: Clockwise or Anticlockwise screw, Up or Down, Left or Right");

	const double maxstep = 30.0;
	double angstep = 360.0 / tracker_nstep;
	int dirCode;
	for (;;)
	{
		std::ostringstream os;
		os << "rotation: Clockwise or Anticlockwise screw, Up or Down, Left or Right; STEP(" << angstep << " deg):+/-";
		dirCode = option("CAUDLR+-", os.str().c_str());
		if (dirCode == '+')
		{
			angstep *= 2;
			if (angstep > maxstep) angstep = maxstep;
			continue;
		}
		if (dirCode == '-')
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
	
	recordingStart(dataCurrentPtr->first, recordingLabel, false);
	context.write("taken snapshot\n");
}

//-------------------------------------------------------------------------------------------------------------------

void pacman::Demo::create(const Desc& desc) {

//	grasp::Manager& manager = dynamic_cast<grasp::Manager&>(*this);
//	RenderBlock b(manager);

	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws

	this->activeSense = desc.activeSense;

	tracker_nstep = desc.tracker_nstep;

	dataName = desc.dataName;

	grasp::Sensor::Map::const_iterator camera = sensorMap.find(desc.drainerCamera);
	drainerCamera = camera != sensorMap.end() ? is<Camera>(camera->second.get()) : nullptr;
	if (!drainerCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown drainer pose estimation camera: %s", desc.drainerCamera.c_str());
	
	grasp::data::Handler::Map::const_iterator handler = handlerMap.find(desc.drainerHandler);
	drainerHandler = handler != handlerMap.end() ? handler->second.get() : nullptr;
	if (!drainerHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown drainer data handler: %s", desc.drainerHandler.c_str());
	
	drainerItem = desc.drainerItem;
	drainerScanPose = desc.drainerScanPose;
	drainerColourSolid = desc.drainerColourSolid;
	drainerColourWire = desc.drainerColourWire;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       PaCMan Demo\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  C                                       Camera Sensor Options\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  L                                       TrackLab\n"));


	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo, load and estimate (M)odel...";
	}));
	
	// drainer pose scan and estimation
	menuCmdMap.insert(std::make_pair("PM", [=]() {
		// run robot
		gotoPose(drainerScanPose);
		// block keyboard and mouse interaction
		InputBlock inputBlock(*this);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(drainerItem);
			to<Data>(dataCurrentPtr)->drainerVertices.clear();
			to<Data>(dataCurrentPtr)->drainerTriangles.clear();
			to<Data>(dataCurrentPtr)->drainerModelTriangles.clear();
		}
		// obtain snapshot handler
		data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(drainerCamera->getSnapshotHandler());
		if (handlerSnapshotPtr == handlerMap.end())
			throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", drainerCamera->getSnapshotHandler().c_str());
		// capture and insert data
		data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
		if (!capture)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", drainerCamera->getSnapshotHandler().c_str());
		data::Item::Map::iterator ptr;
		data::Item::Ptr item = capture->capture(*drainerCamera, [&](const grasp::TimeStamp*) -> bool { return true; });
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(drainerItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(drainerItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// generate features
		data::Transform* transform = is<data::Transform>(handlerSnapshotPtr);
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", drainerCamera->getSnapshotHandler().c_str());
		data::Item::List list;
		list.insert(list.end(), ptr);
		item = transform->transform(list);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(drainerItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(drainerItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// estimate pose
		data::Model* model = is<data::Model>(ptr);
		if (!model)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Model interface", ptr->first.c_str());
		grasp::ConfigMat34 robotPose;
		golem::Mat34 drainerPose;
		Vec3Seq drainerVertices;
		TriangleSeq drainerTriangles;
		model->model(robotPose, drainerPose, &drainerVertices, &drainerTriangles);
		// create triangle mesh
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->drainerVertices = drainerVertices;
			to<Data>(dataCurrentPtr)->drainerTriangles = drainerTriangles;
			for (grasp::TriangleSeq::const_iterator j = drainerTriangles.begin(); j != drainerTriangles.end(); ++j)
				to<Data>(dataCurrentPtr)->drainerModelTriangles.push_back(Contact3D::Triangle(drainerVertices[j->t1], drainerVertices[j->t3], drainerVertices[j->t2]));
		}
		// done
		context.write("Model load and estimation completed!\n");
	}));
	
	// main demo
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// estimate pose
		if (to<Data>(dataCurrentPtr)->drainerModelTriangles.empty())
			menuCmdMap["PM"]();

		// run demo
		context.write("TODO!\n");
		context.write("Done!\n");
	}));

	menuCtrlMap.insert(std::make_pair("L", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (R)otate, (N)udge...";
	}));

	menuCmdMap.insert(std::make_pair("LR", [=]() {
		context.write("rotate object in hand\n");
		rotateObjectInHand();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("LN", [=]() {
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

		context.write("Done!\n");
	}));

	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (D)Demo ActiveSense\n(E)Goto to Camera Hypothesis Pose\n(V)Set OpenGL View Point to Camera Hypothesis View\n(N)View from Next ViewHypothesis\n(K)View from Mounted Sensor\n(H)Print Sensor Hypothesis Matrices";
		//menuCmdMap.erase("CE");
		//menuCmdMap.erase("CV");
	}));



	//ActiveSense Demo 
	menuCmdMap.insert(std::make_pair("CD", [=]() {

		grasp::data::Item::Map predModelMap, trajMap, predQueryMap, imageMap, pointCurvMap;
		grasp::data::Item::Map::iterator itemPredModelPtr, itemTrajPtr, itemPredQueryPtr, itemImagePtr, itemPointCurvPtr;

		auto filter = [&](const grasp::data::Item::Map& itemMap, const std::string& filterID, grasp::data::Item::Map& outMap) {
			for (grasp::data::Item::Map::const_iterator it = itemMap.begin(); it != itemMap.end(); it++)
			{
				if (it->second->getHandler().getID().compare(filterID) == 0)
				{
					outMap.insert(*it);
				}
			}
		};

		
		//Filter by PredictorModel+PredictorModel HandlerID
		filter(dataCurrentPtr->second->itemMap, "PredictorModel+PredictorModel", predModelMap);
		select(itemPredModelPtr, predModelMap.begin(), predModelMap.end(), "Select PredModel:\n", [](grasp::data::Item::Map::iterator ptr) -> std::string{
		return ptr->first + ": " + ptr->second->getHandler().getID();
		});

		activeSense->setPredModelItem(itemPredModelPtr);
		activeSense->nextBestView();

	}));
	menuCmdMap.insert(std::make_pair("CH", [=]() {


		size_t index = activeSense->getViewHypotheses().size();
		Menu::selectIndex(activeSense->getViewHypotheses(), index, "Camera Hypothesis");


		//Show pose lambda
		typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
		ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};
		//end of lambda


		golem::Mat34 frame;
		frame.setId();

		//Camera Frame
		frame = activeSense->getViewHypothesis(index - 1)->getFrame();
		showPose("Camera Frame Pose", frame);
		frame.setInverse(frame);
		showPose("Camera Frame InvPose", frame);

		//ViewFrame Pose
		frame = activeSense->getViewHypothesis(index - 1)->getViewFrame();
		showPose("ViewFrame Pose", frame);
		frame.setInverse(frame);
		showPose("ViewFrame InvPose", frame);

		//Pose
		frame = activeSense->getViewHypothesis(index - 1)->getPose();
		showPose("Pose", frame);
		frame.setInverse(frame);
		showPose("InvPose", frame);

	}));

	menuCmdMap.insert(std::make_pair("CE", [=]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		grasp::CameraDepth* camera = grasp::to<grasp::CameraDepth>(sensorCurrentPtr);

		size_t index = activeSense->getViewHypotheses().size();
		Menu::selectIndex(activeSense->getViewHypotheses(), index, "Choose Camera Hypothesis to Go");

		

		if (this->activeSense->getParameters().generationMethod == ActiveSense::EGenerationMethod::G_RANDOM_SPHERE)
		{
			Mat34 goal = activeSense->computeGoal(activeSense->getViewHypothesis(index - 1)->getFrame(), camera);
			this->gotoPoseWS(goal);
		}
		else if (this->activeSense->getParameters().generationMethod == ActiveSense::EGenerationMethod::G_FIXED)
		{
			this->gotoPoseConfig(activeSense->getViewHypothesis(index - 1)->getConfig());
		}
		else
		{
			context.write("Unknown generation method!\n");
		}

		
		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CV", [&]() {

		size_t index = activeSense->getViewHypotheses().size();
		Menu::selectIndex(activeSense->getViewHypotheses(), index, "Camera Hypothesis");

		activeSense->getViewHypothesis(index - 1)->setGLView(this->scene);


		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CK", [&]() {



		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});



		HypothesisSensor::setGLView(this->scene, grasp::to<grasp::CameraDepth>(sensorCurrentPtr)->getFrame());

		//Show pose lambda
		typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
		ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};
		//end of lambda
		showPose("SensorFrame: ", grasp::to<grasp::CameraDepth>(sensorCurrentPtr)->getFrame());

		context.write("Done!\n");

	}));


	menuCmdMap.insert(std::make_pair("CN", [&]() {

		activeSense->generateViewsFromSeq(activeSense->getParameters().configSeq);
		if (activeSense->getViewHypotheses().size())
		{
			context.write("View from ViewHypothesis %d\n", this->currentViewHypothesis);
			activeSense->getViewHypothesis(this->currentViewHypothesis)->setGLView(this->scene);

			this->currentViewHypothesis = (this->currentViewHypothesis + 1) % activeSense->getViewHypotheses().size();
			
		}
		
		context.write("Done!\n");
		

	}));

	menuCmdMap.insert(std::make_pair("CG", [&]() {

		{
			golem::CriticalSectionWrapper csw(activeSense->getCSViewHypotheses());
			activeSense->generateViews();
		}


		context.write("Done!\n");


	}));




	//ActiveSense initialisation
	this->currentViewHypothesis = 0;
	this->selectedCamera = 0;
	this->initActiveSense(this);
}

//------------------------------------------------------------------------------

void pacman::Demo::render() const {
	Player::render();
	golem::CriticalSectionWrapper cswRenderer(csRenderer);
	drainerRenderer.render();
	activeSense->render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
