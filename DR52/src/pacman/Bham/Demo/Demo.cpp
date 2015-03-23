#include <pacman/Bham/Demo/Demo.h>
#include <Golem/Math/Rand.h>
#include <Grasp/Core/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------



void pacman::Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::Player::Desc::load(context, xmlcontext);

}

//------------------------------------------------------------------------------



pacman::Demo::Demo(golem::Scene &scene) : grasp::Player(scene) {

	this->currentViewHypothesis = 0;
	this->selectedCamera = 0;
	this->initActiveSense(this);
}

pacman::Demo::~Demo() {
}

/*************************USEFUL FUNCTIONS FOR ACTIVE SENS*******************************************************/

void pacman::Demo::postprocess(golem::SecTmReal elapsedTime)
{

	Player::postprocess(elapsedTime);
	golem::CriticalSectionWrapper csw(activeSense.getCSViewHypotheses());
	for (pacman::HypothesisSensor::Seq::iterator it = activeSense.getViewHypotheses().begin(); it != activeSense.getViewHypotheses().end(); it++)
	{
		(*it)->draw((*it)->getAppearance(), this->sensorRenderer);
	}

}

void pacman::Demo::gotoPoseWS(const grasp::ConfigMat34& pose) {
	// current state
	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);

	// find trajectory
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, nullptr, &pose.w, trajectoryDuration, trajectory);
	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	Sleep::msleep(SecToMSec(trajectoryIdleEnd));
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

//-------------------------------------------------------------------------------------------------------------------

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws


	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  C                                       Camera Sensor Options\n"));
	

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo...";
	}));
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		
		context.write("TODO!\n");
		context.write("Done!\n");
		
	}));

	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (D)Demo ActiveSense\n(E)Goto to Camera Hypothesis Pose\n(V)Set OpenGL View Point to Camera Hypothesis View\n(N)View from Next ViewHypothesis\n(K)View from Mounted Sensor\n(H)Print Sensor Hypothesis Matrices";
		//menuCmdMap.erase("CE");
		//menuCmdMap.erase("CV");
	}));



	//Camera Debug
	menuCmdMap.insert(std::make_pair("CD", [=]() {


		//activeSense.generateRandomViews(golem::Vec3(0.5, -0.5, 0.05),100,0.20);
		activeSense.executeActiveSense(10, 0.20);

	}));
	menuCmdMap.insert(std::make_pair("CH", [=]() {

		size_t index = activeSense.getViewHypotheses().size();
		Menu::selectIndex(activeSense.getViewHypotheses(), index, "Camera Hypothesis");

		
		//Show pose lambda
		typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
		ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};
		//end of lambda


		golem::Mat34 frame;
		frame.setId();

		//Camera Frame
		frame = activeSense.getViewHypothesis(index - 1)->getFrame();
		showPose("Camera Frame Pose", frame);
		frame.setInverse(frame);
		showPose("Camera Frame InvPose", frame);

		//ViewFrame Pose
		frame = activeSense.getViewHypothesis(index - 1)->getViewFrame();
		showPose("ViewFrame Pose", frame);
		frame.setInverse(frame);
		showPose("ViewFrame InvPose", frame);

		//Pose
		frame = activeSense.getViewHypothesis(index - 1)->getPose();
		showPose("Pose", frame);
		frame.setInverse(frame);
		showPose("InvPose", frame);

	}));

	menuCmdMap.insert(std::make_pair("CE", [=]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		grasp::CameraDepth* camera = grasp::to<grasp::CameraDepth>(sensorCurrentPtr);
		
		size_t index = activeSense.getViewHypotheses().size();
		Menu::selectIndex(activeSense.getViewHypotheses(), index, "Choose Camera Hypothesis to Go");
		
		Mat34 goal = activeSense.computeGoal(activeSense.getViewHypothesis(index - 1)->getFrame(),camera);
		this->gotoPoseWS(goal);

		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CV", [&]() {

		size_t index = activeSense.getViewHypotheses().size();
		Menu::selectIndex(activeSense.getViewHypotheses(), index, "Camera Hypothesis");

		activeSense.getViewHypothesis(index - 1)->setGLView(this->scene);


		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CK", [&]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		activeSense.getViewHypothesis(0)->setGLView(this->scene, grasp::to<grasp::CameraDepth>(sensorCurrentPtr)->getFrame());


		context.write("Done!\n");

	}));


	menuCmdMap.insert(std::make_pair("CN", [&]() {

		context.write("View from ViewHypothesis %d\n", this->currentViewHypothesis);
		activeSense.getViewHypothesis(this->currentViewHypothesis)->setGLView(this->scene);

		this->currentViewHypothesis = (this->currentViewHypothesis + 1) % activeSense.getViewHypotheses().size();
		
		context.write("Done!\n");

	}));

	
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
