#include <pacman/Bham/Demo/Demo.h>
#include <Golem/Math/Rand.h>
#include <Grasp/Core/Data.h>
#include <Grasp/Grasp/Model.h>


using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------



void pacman::Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::Player::Desc::load(context, xmlcontext);

	this->activeSense->load(xmlcontext);



}

//------------------------------------------------------------------------------



pacman::Demo::Demo(golem::Scene &scene) : grasp::Player(scene) {

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

bool pacman::Demo::gotoPoseConfig(const grasp::ConfigMat34& pose, const Real& linthr, const golem::Real& angthr) {
	// current state
	golem::Controller::State begin = lookupState();
	//context.debug("STATE[1]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
	// target
	golem::Controller::State end = begin;
	end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));
	// find trajectory
	golem::Controller::State::Seq trajectory;
	grasp::RBDist err = findTrajectory(begin, &end, nullptr, trajectoryDuration, trajectory);




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

void pacman::Demo::render() const
{
	Player::render();
	activeSense->render();
}

//-------------------------------------------------------------------------------------------------------------------

void pacman::Demo::create(const Desc& desc) {

//	grasp::Manager& manager = dynamic_cast<grasp::Manager&>(*this);
//	RenderBlock b(manager);

	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws

	this->activeSense = desc.activeSense;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       PaCMan Demo\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  C                                       Camera Sensor Options\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  L                                       TrackLab\n"));


	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo...";
	}));
	menuCmdMap.insert(std::make_pair("PD", [=]() {

		context.write("TODO!\n");
		context.write("Done!\n");

	}));

	menuCtrlMap.insert(std::make_pair("L", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (T)est..";
	}));
	menuCmdMap.insert(std::make_pair("LT", [=]() {

		context.write("Welcome to TrackLab Test 1\n");
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

		Mat34 goal = activeSense->computeGoal(activeSense->getViewHypothesis(index - 1)->getFrame(), camera);
		this->gotoPoseWS(goal);

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

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
