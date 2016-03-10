#include <pacman/Bham/ActiveSensDemo/ActiveSensDemo.h>

#include <Golem/Math/Rand.h>
#include <Grasp/Core/Import.h>
#include <Grasp/App/Player/Data.h>
#include <Golem/UI/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;


//-----------------------------------------------------------------------------

namespace
{
	std::string toXMLString(const golem::Mat34& m)
	{
		char buf[BUFSIZ], *begin = buf, *const end = buf + sizeof(buf) - 1;
		golem::snprintf(begin, end,
			"m11=\"%f\" m12=\"%f\" m13=\"%f\" m21=\"%f\" m22=\"%f\" m23=\"%f\" m31=\"%f\" m32=\"%f\" m33=\"%f\" v1=\"%f\" v2=\"%f\" v3=\"%f\"",
			m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33, m.p.x, m.p.y, m.p.z);
		return std::string(buf);
	}

	std::string toXMLString(const grasp::ConfigMat34& cfg, const bool shortFormat = false)
	{
		std::ostringstream os;
		os.precision(6);
		const size_t n = shortFormat ? 7 : cfg.c.size();
		for (size_t i = 0; i < n; ++i)
		{
			os << (i == 0 ? "c" : " c") << i + 1 << "=\"" << cfg.c[i] << "\"";
		}

		return os.str();
	}

	template <typename E> std::string enumToString(const golem::U32 i, const std::map<std::string, E>& emap)
	{
		for (auto j = emap.begin(); j != emap.end(); ++j)
			if (j->second == i) return j->first;
		return std::string("enumToString: enum code not found");
	}
}

void Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	this->activeSense->load(xmlcontext);
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : 
	Player(scene)
{}

pacman::Demo::~Demo() {
}

/*************************USEFUL FUNCTIONS FOR ACTIVE SENS*******************************************************/

void pacman::Demo::postprocess(golem::SecTmReal elapsedTime) {
	Player::postprocess(elapsedTime);
	
	golem::CriticalSectionWrapper csw(activeSense->getCSViewHypotheses());
#if 0
	for (pacman::HypothesisSensor::Seq::iterator it = activeSense->getViewHypotheses().begin(); it != activeSense->getViewHypotheses().end(); it++)
	{
		(*it)->draw((*it)->getAppearance(), this->sensorRenderer);
	}
#endif
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
	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);
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
	{
		context.write("pacman::Demo::scanPoseActive: **** USING DEFORMATION MAP ****\n");
		camera->getCurrentCalibration()->enableDeformationMap(true);
	}

	for (bool stop = false; !stop;) {
		stop = scanPoseCommand == nullptr || !scanPoseCommand();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, capture->capture(*to<Camera>(sensorCurrentPtr), [&](const grasp::TimeStamp*) -> bool { return true; })));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
			scannedImageItems.push_back(ptr);
		}
	}

	context.write("Done!\n");
}

void pacman::Demo::perform2(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, const bool testTrajectory, const bool controlRecording)
{
	// doesn't interfere with recording

	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);
	findTrajectory(begin, &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

	// create trajectory item
	data::Item::Ptr itemTrajectory;
	data::Handler::Map::const_iterator handlerPtr = handlerMap.find(trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", trajectoryHandler.c_str());
	data::Handler* handler = is<data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", trajectoryHandler.c_str());
	

	trajectoryIf->setWaypoints(completeTrajectory);

	// block displaying the current item
	RenderBlock renderBlock(*this);

	// test trajectory
	if (testTrajectory) {
		// insert trajectory to data with temporary name
		const std::string itemLabelTmp = item + dataDesc->sepName + makeString("%f", context.getTimer().elapsed());
		ScopeGuard removeItem([&]() {
			UI::removeCallback(*this, getCurrentHandler());
			{
				golem::CriticalSectionWrapper csw(scene.getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(itemLabelTmp);
			}
			createRender();
		});
		{
			golem::CriticalSectionWrapper csw(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabelTmp, itemTrajectory));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// enable GUI interaction and refresh
		UI::addCallback(*this, getCurrentHandler());
		createRender();
		// prompt user
		EnableKeyboardMouse enableKeyboardMouse(*this);
		option("\x0D", "Press <Enter> to accept trajectory...");
	}

	// go to initial state
	sendTrajectory(initTrajectory);
	// wait until the last trajectory segment is sent
	controller->waitForEnd();

	if (controlRecording)
	{
		context.write("pacman::Demo::perform2:\n\n\n\n******** CONTROLLING RECORDING: STARTED ********\n\n\n\n");
		// start recording
		recordingStart(data, item, true);
		recordingWaitToStart();
	}

	// send trajectory
	sendTrajectory(trajectory);

	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		// print every 10th robot state
		if (i % 10 == 0)
			context.write("State #%d\r", i);
	}

	if (controlRecording)
	{
		context.write("pacman::Demo::perform2:\n\n\n\n******** CONTROLLING RECORDING: STOPPED ********\n\n\n\n");
		// stop recording
		recordingStop(trajectoryIdlePerf);
		recordingWaitToStop();

		// insert trajectory
		{
			golem::CriticalSectionWrapper csw(scene.getCS());
			data::Data::Map::iterator data = dataMap.find(recorderData);
			if (data == dataMap.end())
				throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to find Data %s", recorderData.c_str());
			data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
		}
	}

	context.write("Performance finished!\n");
}

void pacman::Demo::showRecordingState()
{
	if (recordingActive())
		context.write("pacman::Demo::\n\n******** RECORDING VIDEO ********\n");
	else
		context.write("pacman::Demo::\n\n******** NOT RECORDING VIDEO!!! ********\n");
}


//-------------------------------------------------------------------------------------------------------------------

grasp::Camera* Demo::getWristCamera(const bool dontThrow) const
{
	const std::string id("OpenNI+OpenNI");
	grasp::Sensor::Map::const_iterator i = sensorMap.find(id);
	if (i == sensorMap.end())
	{
		if (dontThrow) return nullptr;
		context.write("%s was not found\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	grasp::Camera* camera = grasp::is<grasp::Camera>(i);

	// want the wrist-mounted camera
	if (!camera->hasVariableMounting())
	{
		if (dontThrow) return nullptr;
		context.write("%s is a static camera\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	return camera;
}

golem::Mat34 Demo::getWristPose() const
{
	const golem::U32 wristJoint = 6; // @@@
	grasp::ConfigMat34 pose;
	getPose(wristJoint, pose);
	return pose.w;
}

golem::Controller::State::Seq Demo::getTrajectoryFromPose(const golem::Mat34& w, const SecTmReal duration)
{
	const golem::Mat34 R = controller->getChains()[armInfo.getChains().begin()]->getReferencePose();
	const golem::Mat34 wR = w * R;

	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);

	golem::Controller::State::Seq trajectory;
	const grasp::RBDist err = findTrajectory(begin, nullptr, &wR, duration, trajectory);

	return trajectory;
}

grasp::ConfigMat34 Demo::getConfigFromPose(const golem::Mat34& w)
{
	golem::Controller::State::Seq trajectory = getTrajectoryFromPose(w, SEC_TM_REAL_ZERO);
	const golem::Controller::State& last = trajectory.back();
	ConfigMat34 cfg(RealSeq(61, 0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < 7; ++i)
	{
		cfg.c[i] = last.cpos.data()[i];
	}
	return cfg;
}

golem::Controller::State Demo::lookupStateArmCommandHand() const
{
	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);
	golem::Controller::State state = begin;	// current state

	//TODO: Uncertain here! Should it be createState?
	golem::Controller::State cmdHand = controller->createState();//lookupCommand();	// commanded state (wanted just for hand)
	state.cpos.set(handInfo.getJoints(), cmdHand.cpos); // only set cpos ???
	return state;
}

void Demo::setHandConfig(Controller::State::Seq& trajectory, const golem::Controller::State cmdHand)
{
	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
	{
		Controller::State& state = *i;
		state.setToDefault(handInfo.getJoints().begin(), handInfo.getJoints().end());
		state.cpos.set(handInfo.getJoints(), cmdHand.cpos);
	}
}

void Demo::setHandConfig(Controller::State::Seq& trajectory, const grasp::ConfigMat34& handPose)
{
	ConfigspaceCoord cposHand;
	cposHand.set(handPose.c.data(), handPose.c.data() + std::min(handPose.c.size(), (size_t)info.getJoints().size()));

	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
	{
		Controller::State& state = *i;
		state.setToDefault(handInfo.getJoints().begin(), handInfo.getJoints().end());
		state.cpos.set(handInfo.getJoints(), cposHand);
	}
}

void Demo::gotoWristPose(const golem::Mat34& w, const SecTmReal duration)
{
	golem::Controller::State::Seq trajectory = getTrajectoryFromPose(w, duration);
	sendTrajectory(trajectory);
	controller->waitForEnd();
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));
}

void Demo::gotoPose2(const ConfigMat34& pose, const SecTmReal duration, const bool ignoreHand)
{
	context.debug("Demo::gotoPose2: %s\n", toXMLString(pose).c_str());

	// always start with hand in commanded config, not actual
	golem::Controller::State begin = lookupStateArmCommandHand();	// current state but commanded state for hand
	golem::Controller::State end = begin;
	end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, &end, nullptr, duration, trajectory);

	if (ignoreHand)
		setHandConfig(trajectory, begin); // overwrite hand config with current commanded cpos

	sendTrajectory(trajectory);
	controller->waitForEnd();
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));
}

void Demo::releaseHand(const double openFraction, const SecTmReal duration)
{
	double f = 1.0 - openFraction;
	f = std::max(0.0, std::min(1.0, f));

	golem::Controller::State currentState = lookupStateArmCommandHand();
	ConfigMat34 openPose(RealSeq(61, 0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < openPose.c.size(); ++i)
		openPose.c[i] = currentState.cpos.data()[i];

	// TODO use proper indices - handInfo.getJoints()
	const size_t handIndexBegin = 7;
	const size_t handIndexEnd = handIndexBegin + 5 * 4;
	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		openPose.c[i] *= f;

	gotoPose2(openPose, duration);
}

void Demo::releaseHand2(const double openFraction, const SecTmReal duration, const golem::Controller::State partReleaseConfig)
{
	golem::Controller::State currentState = lookupStateArmCommandHand();
	currentState.cpos.set(handInfo.getJoints(), partReleaseConfig.cpos);

	ConfigMat34 openPose(RealSeq(61, 0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < openPose.c.size(); ++i)
		openPose.c[i] = currentState.cpos.data()[i];


	//context.debug("Demo::releaseHand2: %s\n", toXMLString(openPose).c_str());
	//if (option("YN", "OK? (Y/N)") == 'Y')
	gotoPose2(openPose, duration);

	//if (option("YN", "release hand OK? (Y/N)") == 'Y')
	releaseHand(openFraction, duration);
}

void Demo::closeHand(const double closeFraction, const SecTmReal duration)
{
	// @@@ HACK @@@

	const double f = std::max(0.0, std::min(1.0, closeFraction));

	// !!! TODO use proper indices
	ConfigMat34 pose(RealSeq(61, 0.0)), finalPose(RealSeq(61, 0.0));
	golem::Controller::State currentState = lookupStateArmCommandHand();
	for (size_t i = 0; i < pose.c.size(); ++i)
		pose.c[i] = currentState.cpos.data()[i];

	// TODO use proper indices - handInfo.getJoints()
	const size_t handIndexBegin = 7;
	const size_t handIndexEnd = handIndexBegin + 5 * 4;
	for (size_t i = handIndexBegin; i < handIndexEnd; i += 4)
	{
		finalPose.c[i + 1] = 0.85;
		finalPose.c[i + 2] = 0.2;
		finalPose.c[i + 3] = 0.2;
	}
	// ease off the thumb
	finalPose.c[handIndexBegin + 0] = 0.22;
	finalPose.c[handIndexBegin + 1] = 0.6;
	finalPose.c[handIndexBegin + 2] = 0.1;
	finalPose.c[handIndexBegin + 3] = 0.1;

	//context.debug("Demo::closeHand: finalPose: %s\n", toXMLString(finalPose).c_str());

	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		pose.c[i] += f * (finalPose.c[i] - pose.c[i]);

	gotoPose2(pose, duration);
}

void Demo::executeDropOff()
{
	context.debug("Moving to drop-off point...\n");
	gotoPose2(activeSense->getParameters().dropOffPose, trajectoryDuration, true);

	//context.debug("Waiting 1s before releasing grasp...\n");
	//Sleep::msleep(SecToMSec(1.0));

	const double withdrawReleaseFraction = 1.0;
	context.debug("Releasing hand by %g%% in %gs...\n", withdrawReleaseFraction*100.0, trajectoryDuration);
	
	
	if (activeSense->pLastExecutedWaypoint != nullptr)
	{
		releaseHand2(withdrawReleaseFraction, trajectoryDuration, *(activeSense->pLastExecutedWaypoint));
	}

	context.write("Done!\n");
}

//------------------------------------------------------------------------------

void Demo::nudgeWrist()
{
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

	grasp::Camera* camera = getWristCamera(true);
	if (camera != nullptr)
	{
		pose = camera->getFrame();
		showPose("camera before", pose);
	}

	pose = getWristPose();
	showPose("before", pose);

	pose.p = pose.p + nudge;
	showPose("commanded", pose);

	gotoWristPose(pose);

	showPose("final wrist", getWristPose());
	if (camera != nullptr)
	{
		pose = camera->getFrame();
		showPose("final camera", pose);
	}

	grasp::ConfigMat34 cp;
	getPose(0, cp);
	context.debug("%s\n", toXMLString(cp, true).c_str());
}

void Demo::rotateObjectInHand()
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

	switch (rotType)
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
	context.debug("%s\n", toXMLString(cp, true).c_str());

	//recordingStart(dataCurrentPtr->first, recordingLabel, false);
	//context.write("taken snapshot\n");
}

//------------------------------------------------------------------------------

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::Demo::Desc."));

	// create object
	Player::create(desc); // throws

	//ActiveSense initialisation
	this->activeSense = desc.activeSense;
	this->currentViewHypothesis = 0;
	this->selectedCamera = 0;
	this->initActiveSense(this);

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  C                                       Camera Sensor Options\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  Z                                       menu SZ Tests\n"));


	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to:\nchange ActiveSense (P)arameters\n(G)enerate possible camera poses\n(D)emo ActiveSense\n"
			"(E) Goto to Camera Hypothesis Pose\n"
			"(V) Set OpenGL View Point to Camera Hypothesis View\n(N) View from Next-Best-View Hypothesis\n(K) View from wrist-mounted sensor\n(H) Print Sensor Hypothesis Matrices";
		//menuCmdMap.erase("CE");
		//menuCmdMap.erase("CV");
	}));

	menuCtrlMap.insert(std::make_pair("CP", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc =
			"Press a key to:\n(L) List parameters\n"
			"(S) Choose Selection Method\n(A) Choose Alternative Selection Method\n"
			"(G) Choose Generation Method\n(M) Choose Coverage Method\n(C) Choose Stopping Criteria";
	
	}));


	//ActiveSense Demo 
	menuCmdMap.insert(std::make_pair("CD", [=]()
	{
		grasp::data::Item::Map predModelMap;
		grasp::data::Item::Map::iterator itemPredModelPtr;

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
		itemPredModelPtr = predModelMap.begin();
		select(
			itemPredModelPtr,
			predModelMap.begin(),
			predModelMap.end(),
			"Select PredModel:\n",
			[](grasp::data::Item::Map::iterator ptr) -> std::string{ return ptr->first + ": " + ptr->second->getHandler().getID(); });

		activeSense->setPredModelItem(itemPredModelPtr);

		activeSense->resetNextBestViewSequential(); // always start from first fixed pose when using sequential selection method

		const int k = option("YN", "Enable user input during execution (Y/N)?");
		activeSense->setAllowInput(k == 'Y');

		if (!recordingActive())
		{
			const int k = option("YN", "Start video recording (Y/N)?");
			if (k == 'Y')
			{
				recordingStart(dataCurrentPtr->first, "ActiveSense-Demo", true);
				recordingWaitToStart();
			}
		}

		showRecordingState();

		if (!activeSense->getParameters().configSeq.empty())
		{
			const int k = option("CF", "For the first view, use (C)urrent camera pose, or first (F)ixed pose?");
			if (k == 'F')
			{
				context.debug("ActiveSense: pacman::Demo: moving to first fixed NBV pose\n");
				const grasp::ConfigMat34& pose = activeSense->getParameters().configSeq.front();
				gotoPoseConfig(pose);
				// then throw this pose away if using sequential selection method
				if (activeSense->getParameters().selectionMethod == ActiveSense::ESelectionMethod::S_SEQUENTIAL)
					activeSense->incrNextBestViewSequential();
			}
		}

		activeSense->nextBestView();

		showRecordingState();

		activeSense->executeTrajectory();

		showRecordingState();

		if (option("YN", "Confirm drop-off action (Y/N)") == 'Y')
			executeDropOff();

		if (recordingActive() && option("YN", "Stop recording video? (Y/N)") == 'Y')
		{
			recordingStop(golem::SEC_TM_REAL_ZERO);
			recordingWaitToStop();
			context.write("\n\n\nNOW SAVE THE DATA BUNDLE!!!\n\n\n");
		}

		context.write(">>>>>>>> ActiveSense Demo Finished! <<<<<<<<\n");

	}));
	menuCmdMap.insert(std::make_pair("CH", [=]() {


		U32 index = (U32)activeSense->getViewHypotheses().size();
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

		U32 index = (U32)activeSense->getViewHypotheses().size();
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

		U32 index = (U32)activeSense->getViewHypotheses().size();
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
		getWristCamera(); // why this call???

		context.write("Done!\n");


	}));


	menuCmdMap.insert(std::make_pair("CPL", [&]() {
		const ActiveSense::Parameters& params = activeSense->getParameters();
		context.write("view generation method:       %s\n", enumToString(params.generationMethod, params.getGenerationMethodMap()).c_str());
		context.write("selection method:             %s\n", enumToString(params.selectionMethod, params.getSelectionMethodMap()).c_str());
		context.write("alternative selection method: %s\n", enumToString(params.alternativeSelectionMethod, params.getSelectionMethodMap()).c_str());
		context.write("stopping criteria:            %s\n", enumToString(params.stoppingCriteria, params.getStoppingCriteriaMap()).c_str());
		context.write("coverage method:              %s\n", enumToString(params.coverageMethod, params.getCoverageMethodMap()).c_str());
		context.write("max numbers of views:         %d\n", params.nviews);
		context.write("coverage threshold:           %g\n", params.coverageThr);
		context.write("centroid calc:                %s\n", params.useManualCentroid ? "manual" : "auto from captured point cloud");
	}));

	menuCmdMap.insert(std::make_pair("CPS", [&]() {

		std::map<std::string, ActiveSense::ESelectionMethod> selectionMethodMap = activeSense->getParameters().getSelectionMethodMap();
		std::map<std::string, ActiveSense::ESelectionMethod>::iterator selectionPtr(selectionMethodMap.begin());
		select(selectionPtr, selectionMethodMap.begin(), selectionMethodMap.end(), "Select Selection Method:\n", [](std::map<std::string, ActiveSense::ESelectionMethod>::iterator ptr) -> std::string{
			return ptr->first;
		});
		context.write("Selected: %s\n", selectionPtr->first.c_str());
		activeSense->getParameters().selectionMethod = selectionPtr->second;

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("CPA", [&]() {

		std::map<std::string, ActiveSense::ESelectionMethod> selectionMethodMap = activeSense->getParameters().getSelectionMethodMap();
		std::map<std::string, ActiveSense::ESelectionMethod>::iterator selectionPtr(selectionMethodMap.begin());
		select(selectionPtr, selectionMethodMap.begin(), selectionMethodMap.end(), "Select Alternative Selection Method:\n", [](std::map<std::string, ActiveSense::ESelectionMethod>::iterator ptr) -> std::string{
			return ptr->first;
		});
		context.write("Selected: %s\n", selectionPtr->first.c_str());
		activeSense->getParameters().alternativeSelectionMethod = selectionPtr->second;

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("CPG", [&]() {

		std::map<std::string, ActiveSense::EGenerationMethod> selectionMap = activeSense->getParameters().getGenerationMethodMap();
		std::map<std::string, ActiveSense::EGenerationMethod>::iterator selectionPtr(selectionMap.begin());
		select(selectionPtr, selectionMap.begin(), selectionMap.end(), "Select Generation Method:\n", [](std::map<std::string, ActiveSense::EGenerationMethod>::iterator ptr) -> std::string{
			return ptr->first;
		});
		context.write("Selected: %s\n",selectionPtr->first.c_str());
		activeSense->getParameters().generationMethod = selectionPtr->second;


		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("CPM", [&]() {

		std::map<std::string, ActiveSense::ECoverageMethod> selectionMap = activeSense->getParameters().getCoverageMethodMap();
		std::map<std::string, ActiveSense::ECoverageMethod>::iterator selectionPtr(selectionMap.begin());
		select(selectionPtr, selectionMap.begin(), selectionMap.end(), "Select Coverage Method:\n", [](std::map<std::string, ActiveSense::ECoverageMethod>::iterator ptr) -> std::string{
			return ptr->first;
		});
		context.write("Selected: %s\n", selectionPtr->first.c_str());
		activeSense->getParameters().coverageMethod = selectionPtr->second;

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("CPC", [&]() {

		std::map<std::string, ActiveSense::EStoppingCriteria> selectionMap = activeSense->getParameters().getStoppingCriteriaMap();
		std::map<std::string, ActiveSense::EStoppingCriteria>::iterator selectionPtr(selectionMap.begin());
		select(selectionPtr, selectionMap.begin(), selectionMap.end(), "Select Stopping Criteria:\n", [](std::map<std::string, ActiveSense::EStoppingCriteria>::iterator ptr) -> std::string{
			return ptr->first;
		});
		context.write("Selected: %s\n", selectionPtr->first.c_str());
		const golem::U32 stoppingCriteria = selectionPtr->second;
		activeSense->getParameters().stoppingCriteria = stoppingCriteria;

		if (stoppingCriteria == ActiveSense::C_NVIEWS || stoppingCriteria == ActiveSense::C_NVIEWS_COVERAGE)
			readNumber("max number of views: ", activeSense->getParameters().nviews);

		if (stoppingCriteria == ActiveSense::C_COVERAGE || stoppingCriteria == ActiveSense::C_NVIEWS_COVERAGE)
			readNumber("coverage threshold: ", activeSense->getParameters().coverageThr);

		context.write("Done!\n");
	}));

	////////////////////////////////////////////////////////////////////////////
	//                               UTILITIES                                // 
	////////////////////////////////////////////////////////////////////////////

	menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc =
			"Press a key to: execute (D)rop off, (R)otate, (N)udge, (H)and control, change trajectory d(U)ration ...";
	}));

	menuCmdMap.insert(std::make_pair("ZD", [=]() {


		if (option("YN", "Confirm drop-off action (Y/N)") == 'Y')
		{
			executeDropOff();
			context.write("Done!\n");
		}

		if (recordingActive() && option("YN", "Stop recording video? (Y/N)") == 'Y')
		{
			recordingStop(golem::SEC_TM_REAL_ZERO);
			recordingWaitToStop();
			context.write("\n\n\nNOW SAVE THE DATA BUNDLE!!!\n\n\n");
		}

	}));

	menuCmdMap.insert(std::make_pair("ZU", [=]() {
		context.write("*** Change trajectory duration BEWARE ***\n");

		do
			readNumber("trajectoryDuration ", trajectoryDuration);
		while (trajectoryDuration < 1.0);

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZR", [=]() {
		context.write("rotate object in hand\n");
		rotateObjectInHand();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZN", [=]() {
		context.write("nudge wrist\n");
		nudgeWrist();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZH", [=]() {
		context.write("Hand Control\n");
		for (;;)
		{
			const int k = option("+-01 ", "increase grasp:+  relax grasp:-  open:0  close:1  <SPACE> to end");
			if (k == 32) break;
			switch (k)
			{
			case '+':
				closeHand(0.1, 1.0);
				break;
			case '1':
				closeHand(1.0, 4.0);
				break;
			case '-':
				releaseHand(0.1, 1.0);
				break;
			case '0':
				releaseHand(1.0, 2.0);
				break;
			}
		}
		context.write("Done!\n");
	}));


	// END OF MENUS
}

//------------------------------------------------------------------------------

void pacman::Demo::render() const {
	Player::render();
	
	activeSense->render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
