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

/*************************USEFUL FUNCTION FOR ACTIVE SENS*******************************************************/
void pacman::Demo::initActiveSense(golem::Scene& scene)
{
	this->currentViewHypothesis = 0;

	Mat34 dummyPose;
	dummyPose.setId();
	dummyPose.p.set(0.0, 0.0, 0.025);

	pacman::HypothesisSensor::ConfigQuery configQuery = [&](U32 joint, pacman::HypothesisSensor::Config& config) { getPose(joint, config); };
	this->dummySensor = pacman::HypothesisSensor::Ptr(new pacman::HypothesisSensor(context, dummyPose, configQuery, 7, golem::RGBA(255, 125, 0, 125)));

	golem::Vec3 center;
	Mat34 sensorPose;
	golem::Rand rand;
	rand.setRandSeed(context.getRandSeed());


	/*//Up right sensor
	sensorPose.R.setColumn(0,golem::Vec3(0.0, 0.0, 1.0));
	sensorPose.R.setColumn(1,golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(2,golem::Vec3(0.0, 1.0, 0.0));
	*/

	//Upside down sensor
	sensorPose.R.setColumn(0, golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(1, golem::Vec3(0.0, 0.0, -1.0));
	sensorPose.R.setColumn(2, golem::Vec3(0.0, 1.0, 0.0));

	sensorPose.p.set(0.5, -0.5, 0.025);
	center = sensorPose.p;

	//Adding example sensor hypothesis
	pacman::HypothesisSensor::Ptr sensor(new HypothesisSensor(context, sensorPose, configQuery, 0));
	this->viewHypotheses.push_back(sensor);

	//generate 5 random poses centered
	golem::Vec3 randomVec;
	golem::Vec3 ori;
	float radius = 0.25;

	for (int i = 0; i < 20; i++)
	{
		//Generate random orientation
		float x, y, z, theta;
		U8 r, g, b, a;

		r = (U8)rand.nextUniform<float>(50, 255), g = (U8)rand.nextUniform<float>(50, 255), b = (U8)rand.nextUniform<float>(50, 255), a = 255;
		theta = rand.nextUniform<float>();
		x = rand.nextUniform<float>(-1, 1), y = rand.nextUniform<float>(-1, 1), z = rand.nextUniform<float>(-1, 1);
		printf("Generating random> XYZ: %f %f %f  RGBA: %d %d %d %d Theta: %f\n", x, y, z, r, g, b, a, theta);
		randomVec.set(x, y, z);
		randomVec.normalise();
		randomVec *= radius;

		//Generate point
		sensorPose = golem::Mat34(golem::Mat34::identity());
		sensorPose.p = center + randomVec;

		//Generate orientation
		ori = center - sensorPose.p;
		golem::Quat q;
		golem::Vec3 f = ori;
		golem::Vec3 up(0.0, 0.0, 1.0);
		golem::Vec3 n = f.cross(up);
		up = n.cross(f);

		f.normalise();
		up.normalise();
		n.normalise();

		q.fromAngleAxis(theta, ori);



		//Up right sensor
		/*sensorPose.R.setColumn(0, up);
		sensorPose.R.setColumn(1, n);
		sensorPose.R.setColumn(2, f);
		*/


		//Upside down sensor
		sensorPose.R.setColumn(0, n);
		sensorPose.R.setColumn(1, -up);
		sensorPose.R.setColumn(2, f);


		pacman::HypothesisSensor::Ptr s(new HypothesisSensor(context, sensorPose, configQuery, 0, golem::RGBA(r, g, b, a)));
		this->viewHypotheses.push_back(s);

	}
}

void pacman::Demo::postprocess(golem::SecTmReal elapsedTime)
{

	Player::postprocess(elapsedTime);

	dummySensor->draw(dummySensor->getAppearance(), this->sensorRenderer);
	for (pacman::HypothesisSensor::Seq::iterator it = viewHypotheses.begin(); it != viewHypotheses.end(); it++)
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
		// TODO
		// select and remove
		float tx, ty, tz;
		ConfigMat34 pose;
		
		//context.write("Insert Position:\n");
		//scanf("%f %f %f", &tx, &ty, &tz);
		//context.write("%f %f %f\n",tx,ty,tz);
		pose.w.p.set(tx, ty, tz);
		
		//pose.c.resize(info.getJoints().size());
		//seq[index - 1].cpos.get(pose.c.begin(), pose.c.end());
		//gotoPoseWS(pose);
		// done!
		createRender();

		


		context.write("Done!\n");
		
	}));

	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (E)Goto to Camera Hypothesis Pose,(V)Set OpenGL View Point to Camera Hypothesis View, (S)Show camera matrices\n(N)View from Next ViewHypothesis\n(K)View from Mounted Sensor\n(S)Print Mounted Sensors Matrices\n(H)Print Sensor Hypothesis Matrices";
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

		size_t index = viewHypotheses.size();
		Menu::selectIndex(viewHypotheses, index, "Camera Hypothesis");

		
		//Show pose lambda
		typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
		ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
			context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		};
		//end of lambda


		golem::Mat34 frame;
		frame.setId();

		//Camera Frame
		frame = viewHypotheses[index - 1]->getFrame();
		showPose("Camera Frame Pose", frame);
		frame.setInverse(frame);
		showPose("Camera Frame InvPose", frame);

		//LocalFrame Pose
		frame = viewHypotheses[index - 1]->getLocalFrame();
		showPose("LocalFrame Pose", frame);
		frame.setInverse(frame);
		showPose("LocalFrame InvPose", frame);

		//ViewFrame Pose
		frame = viewHypotheses[index - 1]->getViewFrame();
		showPose("ViewFrame Pose", frame);
		frame.setInverse(frame);
		showPose("ViewFrame InvPose", frame);

		//Pose
		frame = viewHypotheses[index - 1]->getPose();
		showPose("Pose", frame);
		frame.setInverse(frame);
		showPose("InvPose", frame);

	}));

	menuCmdMap.insert(std::make_pair("CE", [=]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		grasp::CameraDepth* camera = grasp::to<grasp::CameraDepth>(sensorCurrentPtr);
		
		size_t index = viewHypotheses.size();
		Menu::selectIndex(viewHypotheses, index, "Choose Camera Hypothesis to Go");

		golem::Mat34 frame, invFrame, refPose, invRefPose;
		grasp::ConfigMat34 wrist, invWrist, base, invBase;
		frame.setId(), invFrame.setId(), wrist.w.setId(), invWrist.w.setId(), refPose.setId(), invRefPose.setId();

		refPose = controller->getChains()[armInfo.getChains().begin()]->getReferencePose();
		invRefPose.setInverse(refPose);
		
		camera->getConfig(wrist);
		
		invWrist.w.setInverse(wrist.w);
		getPose(0, base);
		invBase.w.setInverse(base.w);

		
		frame = invWrist.w*camera->getFrame();
		invFrame.setInverse(frame);

		Mat34 m = frame;
		m = base.w;
		context.write("\n\nBase: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}", m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		m = frame;
		context.write("\nFrame: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}", m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		m = invFrame;
		context.write("\ninvFrame: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n\n", m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		
		/*Mat34 t;
		t.R.m11 = 0.0828823; t.R.m12 = 0.996521; t.R.m13 = -0.00874724;
		t.R.m21 = -0.996559; t.R.m22 = 0.0828765; t.R.m23 = -0.00102093;
		t.R.m31 = -0.000292434; t.R.m32 = 0.00880176;t.R.m33 = 0.999961;
		t.p.v1 = 0.3162; t.p.v2 = -0.231877; t.p.v3 = -0.347643;
		//t.setInverse(t);*/
		Mat34 goal = viewHypotheses[index - 1]->getFrame()*refPose*invFrame;
	
		this->gotoPoseWS(goal);

		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CV", [&]() {

		size_t index = viewHypotheses.size();
		Menu::selectIndex(viewHypotheses, index, "Camera Hypothesis");

		viewHypotheses[index - 1]->setGLView(this->scene);


		context.write("Done!\n");

	}));

	menuCmdMap.insert(std::make_pair("CK", [&]() {

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](grasp::Sensor::Map::const_iterator ptr) -> const std::string&{
			return ptr->second->getID();
		});

		viewHypotheses[0]->setGLView(this->scene, grasp::to<grasp::CameraDepth>(sensorCurrentPtr)->getFrame());


		context.write("Done!\n");

	}));


	menuCmdMap.insert(std::make_pair("CN", [&]() {

		context.write("View from ViewHypothesis %d\n", this->currentViewHypothesis);
		viewHypotheses[this->currentViewHypothesis]->setGLView(this->scene);

		this->currentViewHypothesis = (this->currentViewHypothesis + 1) % viewHypotheses.size();
		
		context.write("Done!\n");

	}));

	
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
