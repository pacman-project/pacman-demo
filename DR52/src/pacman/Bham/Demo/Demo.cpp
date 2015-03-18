#include <pacman/Bham/Demo/Demo.h>
#include <Golem/Math/Rand.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------



void pacman::Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::Player::Desc::load(context, xmlcontext);

}

//------------------------------------------------------------------------------



pacman::Demo::Demo(golem::Scene &scene) : grasp::Player(scene) {

	this->initActiveSense(scene);
}

pacman::Demo::~Demo() {
}

/*************************USEFUL FUNCTIONS FOR ACTIVE SENS*******************************************************/
void pacman::Demo::initActiveSense(golem::Scene& scene)
{
	Mat34 sensorPose;

	/*Up right sensor
	sensorPose.R.setColumn(0,golem::Vec3(0.0, 0.0, 1.0));
	sensorPose.R.setColumn(1,golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(2,golem::Vec3(0.0, 1.0, 0.0));
	*/

	//Upside down sensor
	sensorPose.R.setColumn(0, golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(1, golem::Vec3(0.0, 0.0, -1.0));
	sensorPose.R.setColumn(2, golem::Vec3(0.0, 1.0, 0.0));

	sensorPose.p.set(0.5, -0.5, 0.025);

	//Example sensor hypothesis is going to be used just as an object in the scene
	dummyObject = pacman::HypothesisSensor::Ptr(new HypothesisSensor(sensorPose));

	//Generating sensor hypotheses around dummyObject's center
	this->activeSense.generateRandomViews(this->activeSense.getViewHypotheses(), dummyObject->getPose().p,20);
}

void pacman::Demo::postprocess(golem::SecTmReal elapsedTime)
{

	Player::postprocess(elapsedTime);
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
//-------------------------------------------------------------------------------------------------------------------

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws


	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  CE                                      Go to Camera Hypothesis\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  CV                                      Change view to Camera Hypothesis\n"));
	//TODO: Add instructions for the other options


	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo...";
	}));
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		

		context.write("Done!\n");
		
	}));

	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (E)Goto to Camera Hypothesis Pose\n(V)Set OpenGL View Point to Camera Hypothesis View\n(S)Show camera matrices\n(N)View from Next ViewHypothesis\n(K)View from Mounted Sensor\n(S)Print Mounted Sensors Matrices\n(H)Print Sensor Hypothesis Matrices";
		//menuCmdMap.erase("CE");
		//menuCmdMap.erase("CV");
	}));

	menuCmdMap.insert(std::make_pair("CS", [=]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		grasp::CameraDepth* camera = grasp::to<grasp::CameraDepth>(sensorCurrentPtr);

		//Show pose lambda
		typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
		ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};
		//end of lambda
		
		
		golem::Mat34 frame;
		frame.setId();

		//Camera Frame
		frame = camera->getFrame();
		showPose("Camera Frame Pose", frame);
		frame.setInverse(frame);
		showPose("Camera Frame InvPose", frame);

		//Calib Parameters Pose
		frame = camera->getCurrentCalibration()->getParameters().pose;
		showPose("CalbParams Pose", frame);
		frame.setInverse(frame);
		showPose("CalbParams InvPose", frame);

		//Calib Extrinsics
		frame = camera->getCurrentCalibration()->getExtrinsicModel().pose;
		showPose("Calb Extrinsics Pose", frame);
		frame.setInverse(frame);
		showPose("Calb Extrinsics InvPose", frame);

		//Calib Frame
		frame = camera->getCurrentCalibration()->getFrame();
		showPose("Calb Frame Pose", frame);
		frame.setInverse(frame);
		showPose("Calb Frame InvPose", frame);

		//Deformation Map

		if (camera->getCurrentCalibration()->hasDeformationMap())
		{
			bool used = camera->getCurrentCalibration()->isEnabledDeformationMap();
			ConfigMat34 config;
			camera->getConfig(config);
			frame = camera->getCurrentCalibration()->getDeformation(config);
			showPose("DeformationMap", frame);
			frame.setInverse(frame);
			showPose("DeformationMap [used: " + used? "Yes]" : "No]",frame);
		}
		else
		{
			context.write("NO Deformation Map!\n");
		}

		

		//Reference Pose
		Mat34 refPose(controller->getChains()[armInfo.getChains().begin()]->getReferencePose());
		showPose("RefPose", refPose);
		refPose.setInverse(refPose);
		showPose("InvRefPose", refPose);

		

		context.write("Done!\n");

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

		golem::Mat34 frame, invFrame, refPose, invRefPose;
		grasp::ConfigMat34 wrist, invWrist, base, invBase;
		frame.setId(), invFrame.setId(), wrist.w.setId(), invWrist.w.setId(), refPose.setId(), invRefPose.setId();

		//Reference frame
		refPose = controller->getChains()[armInfo.getChains().begin()]->getReferencePose();
		invRefPose.setInverse(refPose);
		
		//Wrist pose (7th joint)
		camera->getConfig(wrist);
		//Inverse wrist
		invWrist.w.setInverse(wrist.w);

		//It should be the local frame of the camera
		frame = invWrist.w*camera->getFrame();
		invFrame.setInverse(frame);

		
		Mat34 goal = activeSense.getViewHypothesis(index - 1)->getFrame()*invFrame*refPose;
	
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
