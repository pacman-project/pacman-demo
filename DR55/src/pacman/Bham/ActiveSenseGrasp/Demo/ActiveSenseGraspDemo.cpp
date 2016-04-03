/** @file ActiveSenseDemo.cpp
 *
 * ActiveSenseDemo Grasp
 *
 * @author	Ermano Arruda
 *
 *
 */

#include "pacman/Bham/ActiveSenseGrasp/Demo/ActiveSenseGraspDemo.h"

#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSenseGraspCore.h"

#include <Grasp/Data/Image/Image.h>


#include <ActiveSense/Core/utils3d.h>

#include "pacman/Bham/ActiveSenseGrasp/Utils/PCLConversions.h"


using namespace golem;
using namespace grasp;

namespace pacman {
//-----------------------------------------------------------------------------

void ActiveSenseDemo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
    BaseDemoDR55::Desc::load(context, xmlcontext);

    this->activeSense->load(xmlcontext);


    xmlcontext = xmlcontext->getContextFirst("active_sense");

    collisionDesc->load(xmlcontext->getContextFirst("collision"));

    //////// Manipulator /////////////
    manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
    manipulatorAppearance.load(xmlcontext->getContextFirst("manipulator appearance"));
    golem::XMLData("item_trj", manipulatorItemTrj, xmlcontext->getContextFirst("manipulator"));
    grasp::XMLData(manipulatorPoseStdDev, xmlcontext->getContextFirst("manipulator pose_stddev"), false);

    golem::XMLData("duration", manipulatorTrajectoryDuration, xmlcontext->getContextFirst("manipulator trajectory"));
    golem::XMLData(trajectoryThresholdForce, xmlcontext->getContextFirst("manipulator threshold"));

    golem::XMLData("release_fraction", withdrawReleaseFraction, xmlcontext->getContextFirst("manipulator withdraw_action"));
    golem::XMLData("lift_distance", withdrawLiftDistance, xmlcontext->getContextFirst("manipulator withdraw_action"));
    //////// End Manipulator /////////////

    context.write("SUCCESS READING XML!\n");
}

//------------------------------------------------------------------------------

ActiveSenseDemo::ActiveSenseDemo(Scene &scene) : BaseDemoDR55(scene) {
}

ActiveSenseDemo::~ActiveSenseDemo() {
}

void ActiveSenseDemo::create(const Desc& desc) {
    desc.assertValid(grasp::Assert::Context("ActiveSenseDemo::Desc."));

    // create object
    BaseDemoDR55::create(desc); // throws

    this->toggleShowCurrentHandler = true;

	// manipulator
	manipulator = desc.manipulatorDesc->create(*getPlanner().planner, getPlanner().controllerIDSeq);
	manipulatorAppearance = desc.manipulatorAppearance;
	manipulatorItemTrj = desc.manipulatorItemTrj;

	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(desc.manipulatorPoseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(desc.manipulatorPoseStdDev.ang));
	poseDistanceMax = Math::sqr(desc.manipulatorPoseStdDevMax);

	manipulatorTrajectoryDuration = desc.manipulatorTrajectoryDuration;
	trajectoryThresholdForce = desc.trajectoryThresholdForce;

	withdrawReleaseFraction = desc.withdrawReleaseFraction;
	withdrawLiftDistance = desc.withdrawLiftDistance;

    this->collision = desc.collisionDesc->create(*manipulator, this);

    //ActiveSense initialisation
    this->activeSense = desc.activeSense;
    this->currentViewHypothesis = 0;
    this->selectedCamera = 0;
    this->initActiveSense(this);
    context.write("SUCCESS CREATING ACTIVE SENSE!\n");
    context.debug("Sensor list:\n");
    for(auto it = sensorMap.begin(); it != sensorMap.end(); it++)
        context.debug("%s\n",it->first.c_str());


    setMenus();


}

//------------------------------------------------------------------------------
// OTHERS
//------------------------------------------------------------------------------
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

void ActiveSenseDemo::showRecordingState()
{
    if (recordingActive())
        context.write("Demo::\n\n******** RECORDING VIDEO ********\n");
    else
        context.write("Demo::\n\n******** NOT RECORDING VIDEO!!! ********\n");
}

void ActiveSenseDemo::setHandConfig(Controller::State::Seq& trajectory, const golem::Controller::State cmdHand)
{
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


    for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
    {
        Controller::State& state = *i;
		state.setToDefault(getPlanner().handInfo.getJoints().begin(), getPlanner().handInfo.getJoints().end());
		state.cpos.set(getPlanner().handInfo.getJoints(), cmdHand.cpos);
    }
}

void ActiveSenseDemo::gotoPose3(const ConfigMat34& pose, const SecTmReal duration, const bool ignoreHand)
{
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


    context.debug("Demo::gotoPose3: %s\n", toXMLString(pose).c_str());

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
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

void ActiveSenseDemo::releaseRightHand(const double openFraction, const SecTmReal duration)
{

	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


	double f = 1.0 - openFraction;
	f = std::max(0.0, std::min(1.0, f));

	golem::Controller::State currentState = grasp::Waypoint::lookup(*controller).state;
	ConfigMat34 openPose(RealSeq(61, 0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < openPose.c.size(); ++i)
		openPose.c[i] = currentState.cpos.data()[i];

	// TODO use proper indices - handInfo.getJoints()
	const size_t handIndexBegin = 7;
	const size_t handIndexEnd = handIndexBegin + 5 * 4;
	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		openPose.c[i] *= f;

	gotoPose3(openPose, duration);
}

void ActiveSenseDemo::releaseRightHand2(const double openFraction, const SecTmReal duration, const golem::Controller::State partReleaseConfig)
{
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


    golem::Controller::State currentState = lookupStateArmCommandHand();
	currentState.cpos.set(getPlanner().handInfo.getJoints(), partReleaseConfig.cpos);

    ConfigMat34 openPose(RealSeq(61, 0.0)); // !!! TODO use proper indices
    for (size_t i = 0; i < openPose.c.size(); ++i)
        openPose.c[i] = currentState.cpos.data()[i];


    //context.debug("Demo::releaseHand2: %s\n", toXMLString(openPose).c_str());
    //if (option("YN", "OK? (Y/N)") == 'Y')
    gotoPose3(openPose, duration);

    //if (option("YN", "release hand OK? (Y/N)") == 'Y')
    releaseRightHand(openFraction, duration);
}

void ActiveSenseDemo::closeRightHand(const double closeFraction, const SecTmReal duration)
{
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


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

	//context.debug("BaseDemoDR55::closeHand: finalPose: %s\n", toXMLString(finalPose).c_str());

	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		pose.c[i] += f * (finalPose.c[i] - pose.c[i]);

	gotoPose3(pose, duration);
}

void ActiveSenseDemo::executeDropOff()
{
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


    context.debug("Moving to drop-off point...\n");
    gotoPose3(activeSense->getParameters().dropOffPose, getPlanner().trajectoryDuration, true);

    //context.debug("Waiting 1s before releasing grasp...\n");
    //Sleep::msleep(SecToMSec(1.0));

    const double withdrawReleaseFraction = 1.0;
    context.debug("Releasing hand by %g%% in %gs...\n", withdrawReleaseFraction*100.0, getPlanner().trajectoryDuration);


    if (activeSense->pLastExecutedWaypoint != nullptr)
    {
		releaseRightHand2(withdrawReleaseFraction, getPlanner().trajectoryDuration, *(activeSense->pLastExecutedWaypoint));
    }

    context.write("Done!\n");
}

//------------------------------------------------------------------------------

void ActiveSenseDemo::render() const {
    BaseDemoDR55::render();
    demoRenderer.render();
}


//------------------------------------------------------------------------------

void ActiveSenseDemo::setMenus() {

    menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
        desc = "Press a key to:\nchange ActiveSense (P)arameters\n(G)enerate possible camera poses\n(D)emo ActiveSense\n"
                "(E) Goto to Camera Hypothesis Pose\n"
                "(V) Set OpenGL View Point to Camera Hypothesis View\n(N) View from Next-Best-View Hypothesis\n(K) View from wrist-mounted sensor\n(H) Print Sensor Hypothesis Matrices\n"
                "(C) Set contact points\n"
                "(S) Show/Suppress octree components points"
				"(X) Collect data"
				"(Z) Execute Demo";
				"(Y) Right hand menu";
        //menuCmdMap.erase("CE");
        //menuCmdMap.erase("CV");
    }));

    menuCtrlMap.insert(std::make_pair("CP", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
        desc =
                "Press a key to:\n(L) List parameters\n"
                "(S) Choose Selection Method\n(A) Choose Alternative Selection Method\n"
                "(G) Choose Generation Method\n(M) Choose Coverage Method\n(C) Choose Stopping Criteria";

    }));

    menuCtrlMap.insert(std::make_pair("CS", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
        desc =
                "Press a key to:\n(W) Toggle show workspace tree\n"
                "(C) Toggle show contact tree\n"
                "(L) Toggle show last collision result \n"
                "(F) Toggle workspace free space result";

    }));

    menuCmdMap.insert(std::make_pair("CC", [=]()
    {

        grasp::data::Item::Map contactModelMap;
        grasp::data::Item::Map::iterator itemContactModelPtr;

        auto filter = [&](const grasp::data::Item::Map& itemMap, const std::string& filterID, grasp::data::Item::Map& outMap) {
            for (grasp::data::Item::Map::const_iterator it = itemMap.begin(); it != itemMap.end(); it++)
            {
                if (it->second->getHandler().getID().compare(filterID) == 0)
                {
                    outMap.insert(*it);
                }
            }
        };
        //Filter by ContactModel+ContactModel, HandlerID
        filter(dataCurrentPtr->second->itemMap, "ContactModel+ActiveSenseGraspDataContactModel", contactModelMap);
        itemContactModelPtr = contactModelMap.begin();
        select(
                    itemContactModelPtr,
                    contactModelMap.begin(),
                    contactModelMap.end(),
                    "Select contactModel:\n",
                    [](grasp::data::Item::Map::iterator ptr) -> std::string{ return ptr->first + ": " + ptr->second->getHandler().getID(); });

        activeSense->setContactModelItem(itemContactModelPtr);

    }));

    menuCmdMap.insert(std::make_pair("CT", [=]()
    {
        if( toggleShowCurrentHandler ){
            UI::removeCallback(*this, getCurrentHandler());
            clearItemsView();
        }
        else
            UI::addCallback(*this, getCurrentHandler());

        toggleShowCurrentHandler = !toggleShowCurrentHandler;
        context.write("Done!\n");
    }));

    //ActiveSense Demo
    menuCmdMap.insert(std::make_pair("CD", [=]()
    {

        grasp::data::Item::Map contactModelMap;
        grasp::data::Item::Map::iterator itemContactModelPtr;

        auto filter = [&](const grasp::data::Item::Map& itemMap, const std::string& filterID, grasp::data::Item::Map& outMap) {
            for (grasp::data::Item::Map::const_iterator it = itemMap.begin(); it != itemMap.end(); it++)
            {
                if (it->second->getHandler().getID().compare(filterID) == 0)
                {
                    outMap.insert(*it);
                }
            }
        };
        //Filter by ContactModel+ContactModel, HandlerID
        filter(dataCurrentPtr->second->itemMap, activeSense->getParameters().contactHandler, contactModelMap);
        itemContactModelPtr = contactModelMap.begin();
        select(
                    itemContactModelPtr,
                    contactModelMap.begin(),
                    contactModelMap.end(),
                    "Select contactModel:\n",
                    [](grasp::data::Item::Map::iterator ptr) -> std::string{ return ptr->first + ": " + ptr->second->getHandler().getID(); });

        activeSense->setContactModelItem(itemContactModelPtr);



        activeSense->resetNextBestViewSequential(); // always start from first fixed pose when using sequential selection method

        const int k = option("YN", "Enable user input during execution (Y/N)?");
        activeSense->setAllowInput(k == 'Y');

        if (!recordingActive())
        {
            const int k = option("YN", "Start video recording (Y/N)?");
            if (k == 'Y')
            {
                recordingStart(dataCurrentPtr->first, "ActiveSense-Demo" + this->activeSense->experiment_alias, true);
                recordingWaitToStart();
            }
        }

        showRecordingState();

        if (!activeSense->getParameters().configSeq.empty())
        {
            const int k = option("CF", "For the first view, use (C)urrent camera pose, or first (F)ixed pose?");
            if (k == 'F')
            {
                context.debug("ActiveSense: Demo: moving to first fixed NBV pose\n");
                const grasp::ConfigMat34& pose = activeSense->getParameters().configSeq.front();
                gotoPoseConfig(pose);
                // then throw this pose away if using sequential selection method
                if (activeSense->getParameters().selectionMethod == ActiveSense::ESelectionMethod::S_SEQUENTIAL)
                    activeSense->incrNextBestViewSequential();
            }
        }




        activeSense->nextBestView3();
        //activeSense->collectData();

        //option("Y", "Pause");

        //activeSense->safetyExploration();

        showRecordingState();

        activeSense->executeTrajectory2();

        //        if (option("YN", "Confirm drop-off action (Y/N)") == 'Y')
        //            executeDropOff();

        if (recordingActive() && option("YN", "Stop recording video? (Y/N)") == 'Y')
        {
            recordingStop(golem::SEC_TM_REAL_ZERO);
            recordingWaitToStop();
            context.write("\n\n\nNOW SAVE THE DATA BUNDLE!!!\n\n\n");
        }


        if( option("YN", "Open hand? (Y/N)") == 'Y' )
			releaseRightHand(1.0, 2.0);

        context.write(">>>>>>>> ActiveSense Demo Finished! <<<<<<<<\n");

    }));

	menuCmdMap.insert(std::make_pair("CX", [=]()
	{

		grasp::data::Item::Map contactModelMap;
		grasp::data::Item::Map::iterator itemContactModelPtr;

		auto filter = [&](const grasp::data::Item::Map& itemMap, const std::string& filterID, grasp::data::Item::Map& outMap) {
			for (grasp::data::Item::Map::const_iterator it = itemMap.begin(); it != itemMap.end(); it++)
			{
				if (it->second->getHandler().getID().compare(filterID) == 0)
				{
					outMap.insert(*it);
				}
			}
		};
		//Filter by ContactModel+ContactModel, HandlerID
		filter(dataCurrentPtr->second->itemMap, activeSense->getParameters().contactHandler, contactModelMap);
		itemContactModelPtr = contactModelMap.begin();
		select(
			itemContactModelPtr,
			contactModelMap.begin(),
			contactModelMap.end(),
			"Select contactModel:\n",
			[](grasp::data::Item::Map::iterator ptr) -> std::string{ return ptr->first + ": " + ptr->second->getHandler().getID(); });

		activeSense->setContactModelItem(itemContactModelPtr);



		activeSense->resetNextBestViewSequential(); // always start from first fixed pose when using sequential selection method

		const int k = option("YN", "Enable user input during execution (Y/N)?");
		activeSense->setAllowInput(k == 'Y');

		if (!recordingActive())
		{
			const int k = option("YN", "Start video recording (Y/N)?");
			if (k == 'Y')
			{
				recordingStart(dataCurrentPtr->first, "ActiveSense-Demo" + this->activeSense->experiment_alias, true);
				recordingWaitToStart();
			}
		}

		showRecordingState();

		if (!activeSense->getParameters().configSeq.empty())
		{
			const int k = option("CF", "For the first view, use (C)urrent camera pose, or first (F)ixed pose?");
			if (k == 'F')
			{
				context.debug("ActiveSense: Demo: moving to first fixed NBV pose\n");
				const grasp::ConfigMat34& pose = activeSense->getParameters().configSeq.front();
				gotoPoseConfig(pose);
				// then throw this pose away if using sequential selection method
				if (activeSense->getParameters().selectionMethod == ActiveSense::ESelectionMethod::S_SEQUENTIAL)
					activeSense->incrNextBestViewSequential();
			}
		}


		activeSense->collectData();

		//option("Y", "Pause");

		//activeSense->safetyExploration();


		context.write(">>>>>>>> ActiveSense Demo Finished! <<<<<<<<\n");

	}));
	

	menuCmdMap.insert(std::make_pair("CY", [=]() {
		context.write("Right Hand Control\n");
		for (;;)
		{
			const int k = option("+-01 ", "increase grasp:+  relax grasp:-  open:0  close:1  <SPACE> to end");
			if (k == 32) break;
			switch (k)
			{
			case '+':
				closeRightHand(0.1, 1.0);
				break;
			case '1':
				closeRightHand(1.0, 4.0);
				break;
			case '-':
				releaseRightHand(0.1, 1.0);
				break;
			case '0':
				releaseRightHand(1.0, 2.0);
				break;
			}
		}
		context.write("Done!\n");
	}));



	menuCmdMap.insert(std::make_pair("CZ", [&]() {

		this->graspWithActiveSense();

	}));

    menuCmdMap.insert(std::make_pair("CSW", [&]() {

        this->activeSense->getOnlineModel2().showWorkpaceTree = !this->activeSense->getOnlineModel2().showWorkpaceTree;

        this->createRender();
        context.write("Done!\n");

    }));

    menuCmdMap.insert(std::make_pair("CSF", [&]() {

        this->activeSense->getOnlineModel2().showFreeSpace = !this->activeSense->getOnlineModel2().showFreeSpace;

        this->createRender();
        context.write("Done!\n");

    }));

    menuCmdMap.insert(std::make_pair("CSC", [&]() {

        this->activeSense->getOnlineModel2().showContactTree = !this->activeSense->getOnlineModel2().showContactTree;

        this->createRender();
        context.write("Done!\n");

    }));

    menuCmdMap.insert(std::make_pair("CSL", [&]() {

        this->activeSense->getOnlineModel2().showLastResult = !this->activeSense->getOnlineModel2().showLastResult;

        this->createRender();
        context.write("Done!\n");

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

        // Update visual elements
        this->createRender();
        context.write("Done!\n");


    }));


    menuCmdMap.insert(std::make_pair("CPL", [&]() {
        const ActiveSense::Parameters& params = activeSense->getParameters();
        context.write("view generation method:       %s\n", enumToString(params.generationMethod, params.getGenerationMethodMap()).c_str());
        context.write("selection method:             %s\n", enumToString(params.selectionMethod, params.getSelectionMethodMap()).c_str());
        context.write("alternative selection method: %s\n", enumToString(params.alternativeSelectionMethod, params.getSelectionMethodMap()).c_str());
        context.write("stopping criteria:            %s\n", enumToString(params.stoppingCriteria, params.getStoppingCriteriaMap()).c_str());
        context.write("coverage method:              %s\n", enumToString(params.coverageMethod, params.getCoverageMethodMap()).c_str());
        context.write("max numbers of nbv views per trial:         %d\n", params.nviews);
        context.write("absolute max numbers of nbv views:         %d\n", params.maxnviews);
        context.write("absolute max numbers safety views:         %d\n", params.maxsafetyviews);
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
        // Regenerate views
        activeSense->generateViews();

        // Update visual elements
        this->createRender();
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

        if (stoppingCriteria == ActiveSense::C_NVIEWS || stoppingCriteria == ActiveSense::C_NVIEWS_COVERAGE){
            readNumber("max number of nbv views per trial: ", activeSense->getParameters().nviews);
            readNumber("absolute max number of nbv views: ", activeSense->getParameters().maxnviews);
            readNumber("absolute max number of safety views: ", activeSense->getParameters().maxsafetyviews);
        }

        if (stoppingCriteria == ActiveSense::C_COVERAGE || stoppingCriteria == ActiveSense::C_NVIEWS_COVERAGE)
            readNumber("coverage threshold: ", activeSense->getParameters().coverageThr);

        context.write("Done!\n");
    }));



    ////////////////////////////////////////////////////////////////////////////
    //                               UTILITIES                                //
    ////////////////////////////////////////////////////////////////////////////

    menuCtrlMap.insert(std::make_pair("X", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
        desc =
                "Press a key to: execute \n(D)rop off, New (E)xperiment, (F)inish recording, (T)ransform all items with selected handler  ...";
    }));

	// active sense drop off
    menuCmdMap.insert(std::make_pair("XD", [=]() {


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

	// active sense stop recording
    menuCmdMap.insert(std::make_pair("XF", [=]() {


        if (recordingActive() && option("YN", "Stop recording video? (Y/N)") == 'Y')
        {
            recordingStop(golem::SEC_TM_REAL_ZERO);
            recordingWaitToStop();
            context.write("\n\n\nNOW SAVE THE DATA BUNDLE!!!\n\n\n");
        }

    }));

	// active sesne new experiment
    menuCmdMap.insert(std::make_pair("XE", [=]() {

        if( activeSense->out ) fclose(activeSense->out);

        readString("Experiment Alias: ", activeSense->experiment_alias);
        readString("Experiment Id: ", activeSense->experiment_id);
        readString("Experiment Trial: ", activeSense->experiment_trial);

        std::stringstream ss;

        ss << "./ExperimentLogs/" << activeSense->experiment_id << "_" << activeSense->experiment_alias << "_.txt";

        activeSense->out = fopen(ss.str().c_str(), "a");


        context.write("Done!\n");
    }));


	

	// active sense transform all items with compatible selected handler
    menuCmdMap.insert(std::make_pair("XT", [&] () {
        // find handlers supporting data::Transform
        typedef std::vector<std::pair<data::Handler*, data::Transform*>> TransformMap;
        TransformMap transformMap;
        for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
            data::Transform* transform = is<data::Transform>(i);
            if (transform) transformMap.push_back(std::make_pair(i->second.get(), transform));
        }
        if (transformMap.empty())
            throw Cancel("No handlers support Transform interface");
        // pick up handler
        TransformMap::const_iterator transformPtr = transformMap.begin();

        select(transformPtr, transformMap.begin(), transformMap.end(), "Transform:\n", [] (TransformMap::const_iterator ptr) -> std::string {
            std::stringstream str;
            for (StringSeq::const_iterator i = ptr->second->getTransformInterfaces().begin(); i != ptr->second->getTransformInterfaces().end(); ++i) str << *i << " ";
            return std::string("Handler: ") + ptr->first->getID() + std::string(", Item interfaces: ") + str.str();
        });



        for(auto it = to<Data>(dataCurrentPtr)->itemMap.begin(); it != to<Data>(dataCurrentPtr)->itemMap.end(); it++)
        {
            if(transformPtr->second->isTransformSupported(*it->second)){

                data::Item::List itemList;
                itemList.clear();
                itemList.push_back(it);

                // transform
                RenderBlock renderBlock(*this);
                UI::addCallback(*this, transformPtr->first);

                data::Item::Ptr item = transformPtr->second->transform(itemList);
                {
                    golem::CriticalSectionWrapper cswData(scene.getCS());
                    const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(std::string("Cube")+it->first, item));
                    Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
                }
            }


        }




        // done!
        context.write("Done!\n");
    }));




    // END OF MENUS
}

void ActiveSenseDemo::graspWithActiveSense(){

	grasp::data::Item::Map contactModelMap;
	grasp::data::Item::Map::iterator itemContactModelPtr;

	auto filter = [&](const grasp::data::Item::Map& itemMap, const std::string& filterID, grasp::data::Item::Map& outMap) {
		for (grasp::data::Item::Map::const_iterator it = itemMap.begin(); it != itemMap.end(); it++)
		{
			if (it->second->getHandler().getID().compare(filterID) == 0)
			{
				outMap.insert(*it);
			}
		}
	};
	//Filter by ContactModel+ContactModel, HandlerID
	filter(dataCurrentPtr->second->itemMap, activeSense->getParameters().contactHandler, contactModelMap);
	itemContactModelPtr = contactModelMap.begin();
	select(
		itemContactModelPtr,
		contactModelMap.begin(),
		contactModelMap.end(),
		"Select contactModel:\n",
		[](grasp::data::Item::Map::iterator ptr) -> std::string{ return ptr->first + ": " + ptr->second->getHandler().getID(); });

	activeSense->setContactModelItem(itemContactModelPtr);



	activeSense->resetNextBestViewSequential(); // always start from first fixed pose when using sequential selection method

	const int k = option("YN", "Enable user input during execution (Y/N)?");
	activeSense->setAllowInput(k == 'Y');

	if (!recordingActive())
	{
		const int k = option("YN", "Start video recording (Y/N)?");
		if (k == 'Y')
		{
			recordingStart(dataCurrentPtr->first, "ActiveSense-Demo" + this->activeSense->experiment_alias, true);
			recordingWaitToStart();
		}
	}

	showRecordingState();

	if (!activeSense->getParameters().configSeq.empty())
	{
		const int k = option("CF", "For the first view, use (C)urrent camera pose, or first (F)ixed pose?");
		if (k == 'F')
		{
			context.debug("ActiveSense: Demo: moving to first fixed NBV pose\n");
			const grasp::ConfigMat34& pose = activeSense->getParameters().configSeq.front();
			gotoPoseConfig(pose);
			// then throw this pose away if using sequential selection method
			if (activeSense->getParameters().selectionMethod == ActiveSense::ESelectionMethod::S_SEQUENTIAL)
				activeSense->incrNextBestViewSequential();
		}
	}

	activeSense->nextBestView3();

	showRecordingState();

	activeSense->executeTrajectory2();

	if (recordingActive() && option("YN", "Stop recording video? (Y/N)") == 'Y')
	{
		recordingStop(golem::SEC_TM_REAL_ZERO);
		recordingWaitToStop();
		context.write("\n\n\nNOW SAVE THE DATA BUNDLE!!!\n\n\n");
	}


	if (option("YN", "Open hand? (Y/N)") == 'Y')
		releaseRightHand(1.0, 2.0);

	context.write(">>>>>>>> ActiveSense Demo Finished! <<<<<<<<\n");
}
/*************************USEFUL FUNCTIONS FOR ACTIVE SENS*******************************************************/

golem::Controller::State ActiveSenseDemo::lookupState() const {

    golem::Controller::State state = grasp::Waypoint::lookup(*controller).state;	// current state

    //Old way of getting current state
    //golem::Controller::State begin = controller->createState();
    //controller->lookupState(SEC_TM_REAL_MAX, begin);

    return state;
}

golem::Controller::State ActiveSenseDemo::lookupCommand() const {
    golem::Controller::State cmdHand = grasp::Waypoint::lookup(*controller).command;	// commanded state (wanted just for hand)
    return cmdHand;
}


void ActiveSenseDemo::postprocess(golem::SecTmReal elapsedTime) {
    BaseDemoDR55::postprocess(elapsedTime);

    //    golem::CriticalSectionWrapper csw(scene.getCS());
    //    for (size_t i = 0; i < point->getNumOfPoints(); ++i)
    //        objectRenderer.addPoint(point->getPoint(i), golem::RGBA::BLACK);

    //      context.debug("postprocess method\n");
    //    golem::CriticalSectionWrapper csw(activeSense->getCSViewHypotheses());
    //    golem::CriticalSectionWrapper csw2(scene.getCS());
    //    manipulatorAppearance.draw(*manipulator, manipulator->getConfig(lookupState()), demoRenderer);
    //    this->demoRenderer.reset();
    //    if(activeSense->getParameters().showSensorHypotheses){
    //        for (HypothesisSensor::Seq::iterator it = activeSense->getViewHypotheses().begin(); it != activeSense->getViewHypotheses().end(); it++)
    //        {
    //            (*it)->draw((*it)->getAppearance(), this->demoRenderer);
    //        }
    //    }
    //    golem::Mat34 centroidFrame;
    //    centroidFrame.setId();
    //    centroidFrame.p = activeSense->getParameters().centroid;
    //    demoRenderer.addAxes3D(centroidFrame, golem::Vec3(0.05));
}

void ActiveSenseDemo::clearItemsView(){
    for( auto it_i = dataMap.begin(); it_i != dataMap.end(); it_i++ ){
        for (auto it_j = to<Data>(it_i)->itemMap.begin();
             it_j != to<Data>(it_i)->itemMap.end();
             it_j++){
            //context.debug("Cleaning handlers");
            UI::removeCallback(*this, &it_j->second->getHandler());

        }
    }
}

void ActiveSenseDemo::clearRender() {
    UI::removeCallback(*this, getCurrentHandler());

    //context.debug("Cleaning handlers loop");
    //if( dataCurrentPtr != dataMap.end() ){

    //    for( auto it_i = dataMap.begin(); it_i != dataMap.end(); it_i++ ){
    //        for (auto it_j = to<Data>(it_i)->itemMap.begin();
    //             it_j != to<Data>(it_i)->itemMap.end();
    //             it_j++){
    //            //context.debug("Cleaning handlers");
    //            UI::removeCallback(*this, &it_j->second->getHandler());

    //        }
    //    }
    //}

    golem::CriticalSectionWrapper csw(scene.getCS());
    this->demoRenderer.reset();
    BaseDemoDR55::createRender();
}

void ActiveSenseDemo::createRender() {
    BaseDemoDR55::createRender();


    {
        golem::CriticalSectionWrapper csw(scene.getCS());
        //owner->modelRenderer.reset();

        // Do stuff (re-add visual elements corresponding to data entities, or create them)
        // E.g. sensor hypothesis don't move much, so we can draw them here,
        // and if we ever change them, just call this->createRender again
        this->demoRenderer.reset();
        if(activeSense->getParameters().showSensorHypotheses){
            for (HypothesisSensor::Seq::iterator it = activeSense->getViewHypotheses().begin(); it != activeSense->getViewHypotheses().end(); it++)
            {
                (*it)->draw((*it)->getAppearance(), this->demoRenderer);
            }
        }
        golem::Mat34 centroidFrame;
        centroidFrame.setId();
        centroidFrame.p = activeSense->getParameters().centroid;
        demoRenderer.addAxes3D(centroidFrame, golem::Vec3(0.05));



        //    manipulatorAppearance.draw(*manipulator, manipulator->getConfig(lookupState()), demoRenderer);
    }

    activeSense->getOnlineModel2().draw(demoRenderer,scene);


}

bool ActiveSenseDemo::gotoPoseWS(const grasp::ConfigMat34& pose, const Real& linthr, const golem::Real& angthr) {
    
	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });

	
	// current state
    golem::Controller::State begin = controller->createState();
    controller->lookupState(SEC_TM_REAL_MAX, begin);

    // find trajectory
    golem::Controller::State::Seq trajectory;
    grasp::RBDist err = findTrajectory(begin, nullptr, &pose.w, getPlanner().trajectoryDuration, trajectory);

    //grasp::data::Trajectory()




    if (err.lin >= linthr || err.ang >= angthr)
    {
        return false;
    }

    sendTrajectory(trajectory);
    // wait for end
    controller->waitForEnd();
    // sleep
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));




    return true;
}

bool ActiveSenseDemo::gotoPoseWS2(const grasp::ConfigMat34& pose, const Real& linthr, const golem::Real& angthr) {
	// current state
	grasp::Waypoint waypoint = grasp::Waypoint::lookup(*controller);
	golem::Controller::State begin = waypoint.command;// lookupStateArmCommandHand();

	//grasp::RealSeq handCmd; handCmd.assign(getPlanner().handInfo.getJoints().size(), REAL_ZERO);
	//for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i) {
	//	const U32 k = i - getPlanner().handInfo.getJoints().begin();
	//	handCmd[k] = begin.cpos[i];
	//}
	//context.write("Thumb command pose: [%f %f %f %f]\n", handCmd[0], handCmd[1], handCmd[2], handCmd[3]);
	// find trajectory
	golem::Controller::State::Seq trajectory = getTrajectoryFromPose(pose.w, golem::SEC_TM_REAL_ZERO);
	//grasp::RBDist err = findTrajectory(begin, nullptr, &pose.w, getPlanner().trajectoryDuration, trajectory);

	for (auto&i : trajectory) {
		i.reserved = waypoint.command.reserved;
		auto h = i.cpos.data() + *getPlanner().handInfo.getJoints().begin();
		context.write("trajectory: %f %f %f %f\n", h[0], h[1], h[2], h[3]);
	}

	auto h = waypoint.state.cpos.data() + *getPlanner().handInfo.getJoints().begin();
	context.write("state: %f %f %f %f\n", h[0], h[1], h[2], h[3]);
	h = waypoint.command.cpos.data() + *getPlanner().handInfo.getJoints().begin();
	context.write("command: %f %f %f %f\n", h[0], h[1], h[2], h[3]);

	/*if (err.lin >= linthr || err.ang >= angthr)
	{
		return false;
	}*/
	// profiling
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::profile(): At least two waypoints required");

	trajectory.front().t = golem::SEC_TM_REAL_ZERO;
	trajectory.back().t = getPlanner().trajectoryDuration;
	getPlanner().planner->getProfile()->profile(trajectory);

	// send trajectory
	sendTrajectory(trajectory);

	auto thumb = getPlanner().handInfo.getChains().begin();
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		// print every 10th robot state
		if (i % 10 == 0) {
			grasp::Waypoint w = grasp::Waypoint::lookup(*controller);

			context.write("State %d\n", i);
			h = w.state.cpos.data() + *getPlanner().handInfo.getJoints().begin();
			context.write("state: %f %f %f %f\n", h[0], h[1], h[2], h[3]);
			h = w.command.cpos.data() + *getPlanner().handInfo.getJoints().begin();
			context.write("command: %f %f %f %f\n", h[0], h[1], h[2], h[3]);

			//Controller::State state = lookupState();
			//Controller::State cmd = lookupCommand();

			//grasp::RealSeq handCommand; handCommand.assign(getPlanner().handInfo.getJoints().size(), REAL_ZERO);
			//grasp::RealSeq handstate; handstate.assign(getPlanner().handInfo.getJoints().size(), REAL_ZERO);
			//for (auto i = getPlanner().handInfo.getJoints(thumb).begin(); i != getPlanner().handInfo.getJoints(thumb).end(); ++i) {
			//	const U32 k = i - getPlanner().handInfo.getJoints(thumb).begin();
			//	handCommand[k] = cmd.cpos[i];
			//	handstate[k] = state.cpos[i];
			//}
			//context.write("State %d\n", i);
			//context.write("Thumb state pose: [%f %f %f %f]\n", handstate[0], handstate[1], handstate[2], handstate[3]);
			//context.write("Thumb command pose: [%f %f %f %f]\n", handCommand[0], handCommand[1], handCommand[2], handCommand[3]);
		}
	}
	// sleep
//	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));

	return true;
}


grasp::ConfigMat34 ActiveSenseDemo::getPoseFromConfig(const grasp::ConfigMat34& config, int jointIdx)
{
	grasp::ConfigMat34 poseOut = config;

	//Show pose lambda
	typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
	ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
		demoOwner->context.debug("ActiveSense: %s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};

	// Construct state from configuration input
	golem::Controller::State state = this->controller->createState();
	state.cpos.set(config.c.data(), config.c.data() + std::min(config.c.size(), static_cast<size_t>(this->info.getJoints().size())));

	// Forward transform
	golem::WorkspaceJointCoord wjc;
	demoOwner->controller->jointForwardTransform(state.cpos, wjc);

	// Setting SE(3) pose
	poseOut.w = wjc[this->info.getJoints().begin() + jointIdx - 1];

	//showPose("sensor pose", config.w);

	return poseOut;

}

/////////////////////////////// SANDBOX TRAJECTORY INTERPOLATION /////////////////////////////////////
/// \brief ActiveSenseDemo::sandBoxTrajInterp
/// \param trajectory
//

void ActiveSenseDemo::drawBox(const golem::Vec3& p, const golem::Bounds::Ptr& box, bool visible){

    golem::Mat34 pose = box->getPose();
    pose.p = p;


    box->setPose(pose);
    if(visible) {
        demoRenderer.setColour(golem::RGBA(125, 255, 100, 200));
    }
    else{
        demoRenderer.setColour(golem::RGBA(125, 222, 222, 32));
    }
    demoRenderer.addSolid(*box);
    demoRenderer.setColour(golem::RGBA(0, 0, 0, 50));
    demoRenderer.addWire(*box);
    //demoRenderer.addAxes3D(box->getPose(),golem::Vec3(0.001));
    //    const golem::Mat34 base(config.frame.toMat34());
    //    golem::WorkspaceJointCoord joints;
    //    manipulator->getJointFrames(config.config, base, joints);

    //    box->setPose(golem::Mat34(joints[manipulator->getHandInfo().getJoints().begin()+3]));
    //    demoRenderer.setColour(golem::RGBA(125, 222, 222, 255));
    //    demoRenderer.addSolid(*box);
    //    demoRenderer.addAxes3D(box->getPose(),golem::Vec3(0.2));
}

void ActiveSenseDemo::sandBoxTrajInterp(const golem::Controller::State::Seq& trajectory) {
    //       RenderBlock renderBlock(*this);
    //        ScopeGuard removeItem([&]() {
    //            UI::removeCallback(*this, getCurrentHandler());
    //            createRender();
    //        });
    grasp::Manipulator::Waypoint::Seq path = convertToManipulatorWayPoints(trajectory);
    Collision::Result collisionResult;
    size_t eval_size = 25;
    //golem::Real prob = activeSense->getOnlineModel2().computeValue(path,collisionResult, eval_size);
    //active_sense::Model::Voxel::Seq& voxels = collisionResult.voxels;
    //context.debug("Current Distance %lf\n", prob);

    golem::WorkspaceJointCoord joints;
    golem::Real hi_final = path.back().getDistance();
    golem::Controller::State state = lookupState();

    grasp::Manipulator::Config config = manipulator->getConfig(state);//manipulator->interpolate(path, hi_final);
    manipulator->getJointFrames(config.config, config.frame.toMat34(), joints);

    for (Configspace::Index j = manipulator->getHandInfo().getJoints().begin(); j < manipulator->getHandInfo().getJoints().end(); ++j)
        demoRenderer.addAxes3D(joints[j], golem::Vec3(0.05,0.05,-0.05));



    //Bounds description
    golem::BoundingBox::Desc boundDesc;
    boundDesc.dimensions.set(0.01, 0.01, 0.01);
    boundDesc.pose.setId();
    //boundDesc.pose.p.z -= 0.025;rm

    //golem::BoundingBox::Ptr box = descBox->create();

    //               {
    //                  activeSense->getOnlineModel2().draw(demoRenderer,scene);
    //                   option("\x0D", "Showing Manipulator interpolated Press <Enter> to move along...");
    //               }

    //RenderBlock renderBlock(*this);

//    for(int i = 0; i < this->activeSense->getViewHypotheses().size(); i++){
//        golem::Real visible_voxels = activeSense->getOnlineModel2().computeValue(this->activeSense->getViewHypotheses()[i], collisionResult);
//        //context.debug("View %d sees %lf voxels\n", i, visible_voxels);
//    }
//    {
//        golem::CriticalSectionWrapper cswData(scene.getCS());
//        for(int i = 0; i < voxels.size(); i++){

//            // if( i%1 == 0 ) {

//            boundDesc.dimensions.set(voxels[i].size,voxels[i].size,voxels[i].size);

//            golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));

//            drawBox(golem::Vec3(voxels[i].point.x(),voxels[i].point.y(),voxels[i].point.z()),descBox->create(), voxels[i].is_visible);
//            //context.debug("Adding voxel %d\n", i);
//            // }
//        }
//    }


    option("\x0D", "Showing Manipulator interpolated Press <Enter> to move along...");



    //golem::CriticalSectionWrapper csw(scene.getCS());
    context.debug("path length %d traj length %d\n", path.size(), trajectory.size());
    float lo = path.front().getDistance();
    float hi = path.back().getDistance();
    float nintervals = eval_size;
    float delta = (hi-lo)/nintervals;
    context.debug("hi %lf lo %lf\n", hi, lo);
    float c = golem::numeric_const<Real>::ZERO;


//    for (float i = lo; i <= hi; golem::kahanSum(i,c,delta)){
//        //manipulator->getConfig(trajectory[10])
//        //manipulator->interpolate(path, pl)
//        //manipulator->getConfig(*i)
//        demoRenderer.reset();
//        Manipulator::Config config = manipulator->interpolate(path, i);



//        manipulatorAppearance.draw(*manipulator, config , demoRenderer);



//        context.debug("Current Distance %lf hi %lf\n",i,hi);
//        //pl+= 10;
//        // enable GUI interaction and refresh
//        UI::addCallback(*this, getCurrentHandler());
//        //createRender();
//        // prompt user
//        EnableKeyboardMouse enableKeyboardMouse(*this);
//        option("\x0D", "Showing Manipulator interpolated Press <Enter> to move along...");
//    }

}
/////////////////////////////// SANDBOX TRAJECTORY INTERPOLATION /////////////////////////////////////
bool ActiveSenseDemo::gotoPoseConfig(const grasp::ConfigMat34& config, const Real& linthr, const golem::Real& angthr) {
    // current state
    golem::Controller::State begin = controller->createState();
    controller->lookupState(SEC_TM_REAL_MAX, begin);
    //context.debug("STATE[1]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
    // target
    golem::Controller::State end = begin;
    end.cpos.set(config.c.data(), config.c.data() + std::min(config.c.size(), (size_t)info.getJoints().size()));
    // find trajectory
    golem::Controller::State::Seq trajectory;
    grasp::RBDist err = findTrajectory(begin, &end, nullptr, getPlanner().trajectoryDuration, trajectory);

    //sandBoxTrajInterp(trajectory);

    if (err.lin >= linthr || err.ang >= angthr)
    {
        context.write("Error treshold exceeded: lin: %f ang: %f", err.lin, err.ang);
        return false;
    }

	processTrajectory(trajectory);
    sendTrajectory(trajectory);
    // wait for end
    controller->waitForEnd();
    // sleep
    Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));

    return true;
}

void ActiveSenseDemo::scanPoseActive(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel, const std::string& handler, ScanPoseCommand scanPoseCommand, grasp::Manager::Data::Ptr dataPtr) {
    if( this->activeSense->getParameters().useSimCam )
        scanPoseActiveFile(scannedImageItems, itemLabel, handler);
    else
        scanPoseActiveSensor(scannedImageItems,itemLabel, handler,scanPoseCommand, dataPtr);

}

void ActiveSenseDemo::scanPoseActiveSensor(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel, const std::string& handler, ScanPoseCommand scanPoseCommand, grasp::Manager::Data::Ptr dataPtr) {
    data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(handler.empty()? to<Sensor>(sensorCurrentPtr)->getSnapshotHandler() : handler);
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
        context.write("ActiveSenseDemo::scanPoseActive: **** USING DEFORMATION MAP ****\n");
        camera->getCurrentCalibration()->enableDeformationMap(true);
    }

    grasp::Manager::Data* data;
    auto data_it = this->dataCurrentPtr;

    if( dataPtr.get() )
        data = to<grasp::Manager::Data>(dataPtr);
    else
        data = to<grasp::Manager::Data>(data_it);


    for (bool stop = false; !stop;) {
        stop = scanPoseCommand == nullptr || !scanPoseCommand();
        RenderBlock renderBlock(*this);
        {
            golem::CriticalSectionWrapper cswData(scene.getCS());
            grasp::data::Item::Ptr itemImage = capture->capture(*to<Camera>(sensorCurrentPtr), [&](const grasp::TimeStamp*) -> bool { return true; });

            const data::Item::Map::iterator ptr = data->itemMap.insert(data->itemMap.end(), data::Item::Map::value_type(dataItemLabel,itemImage ));

            Data::View::setItem(data->itemMap, ptr, data->getView());
            scannedImageItems.push_back(ptr);
        }
    }

    context.write("Done!\n");
}


void ActiveSenseDemo::scanPoseActiveFile(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel, const std::string& handler) {



    auto data_it = this->dataCurrentPtr;


    if(data_it == this->dataMap.end()){
        context.debug("ActiveSense: no re-recorded pointclouds!\n");
        throw Message(Message::LEVEL_ERROR,"ActiveSense: no re-recorded pointclouds!");
    }

    grasp::Manager::Data* data;


    data = to<grasp::Manager::Data>(data_it);

    auto item_it = data->itemMap.find(itemLabel);

    if(item_it == data->itemMap.end()){
        context.debug("ActiveSense: could not find pre-recorded item %s!\n",itemLabel.c_str());
        throw Message(Message::LEVEL_ERROR,"ActiveSense: could not find pre-recorded item %s!\n",itemLabel.c_str());
    }

    {
        golem::CriticalSectionWrapper csw(scene.getCS());
        Data::View::setItem(data->itemMap, item_it, data->getView());
    }

    scannedImageItems.push_back(item_it);



    context.write("Done!\n");
}


void ActiveSenseDemo::perform2(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, const bool testTrajectory, const bool controlRecording)
{
    // doesn't interfere with recording

    if (trajectory.size() < 2)
        throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): At least two waypoints required");

    golem::Controller::State::Seq initTrajectory;
    findTrajectory(lookupState(), &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

    golem::Controller::State::Seq completeTrajectory = initTrajectory;
    completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

    // create trajectory item
    data::Item::Ptr itemTrajectory;

	data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
    if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
    data::Handler* handler = is<data::Handler>(handlerPtr);
    if (!handler)
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
    itemTrajectory = handler->create();
    data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
    if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());

    trajectoryIf->setWaypoints(grasp::Waypoint::make(completeTrajectory, completeTrajectory));

    // block displaying the current item
    RenderBlock renderBlock(*this);

    // test trajectory
    if (testTrajectory) {
        // insert trajectory to data with temporary name
        const std::string itemLabelTmp = item + dataDesc->sepName + makeString("%f", context.getTimer().elapsed());
        ScopeGuard removeItem([&]() {
            //                        UI::removeCallback(*this, getCurrentHandler());
            //                        {
            //                            golem::CriticalSectionWrapper csw(scene.getCS());
            //                            to<Data>(dataCurrentPtr)->itemMap.erase(itemLabelTmp);
            //                        }
            //            createRender();
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
        context.write("Demo::perform2:\n\n\n\n******** CONTROLLING RECORDING: STARTED ********\n\n\n\n");
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
        context.write("Demo::perform2:\n\n\n\n******** CONTROLLING RECORDING: STOPPED ********\n\n\n\n");
        // stop recording
        recordingStop(getPlanner().trajectoryIdlePerf);
        recordingWaitToStop();

        // insert trajectory
        {
            golem::CriticalSectionWrapper csw(scene.getCS());
            data::Data::Map::iterator data = dataMap.find(recorderData);
            if (data == dataMap.end())
                throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unable to find Data %s", recorderData.c_str());
            data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
        }
    }

    context.write("Performance finished!\n");
}


grasp::Manipulator::Waypoint::Seq ActiveSenseDemo::convertToManipulatorWayPoints(const grasp::Waypoint::Seq& waypoints){
    // create path
    return manipulator->create(waypoints, [=](const Manipulator::Config& l, const Manipulator::Config& r) -> Real { return poseCovInv.dot(RBDist(l.frame, r.frame)); });

}

grasp::Manipulator::Waypoint::Seq ActiveSenseDemo::convertToManipulatorWayPoints(const data::Item::Ptr &itemTrajectory){

    data::Trajectory* trajectoryInterface = is<data::Trajectory>(itemTrajectory.get());

    if(!trajectoryInterface)
        throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());

    const grasp::Waypoint::Seq& waypoints = trajectoryInterface->getWaypoints();

    return convertToManipulatorWayPoints(waypoints);


}

grasp::Manipulator::Waypoint::Seq ActiveSenseDemo::convertToManipulatorWayPoints(const golem::Controller::State::Seq &trajectory){


    data::Item::Ptr itemTrajectory = convertToTrajectory(trajectory);

    return convertToManipulatorWayPoints(itemTrajectory);


}

golem::Controller::State::Seq ActiveSenseDemo::completeTrajectory2(data::Trajectory& trajectory){
    //data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());
    //trajectory->getWaypoints();

    // play
    Controller::State::Seq controllerTraj;
    trajectory.createTrajectory(controllerTraj);

    golem::Controller::State::Seq initTrajectory;
    findTrajectory(lookupState(), &controllerTraj.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

    golem::Controller::State::Seq completeTrajectory = initTrajectory;
    completeTrajectory.insert(completeTrajectory.end(), controllerTraj.begin(), controllerTraj.end());

    return completeTrajectory;


}

grasp::Manipulator::Waypoint::Seq ActiveSenseDemo::completeTrajectory(data::Trajectory& trajectory){
    //data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());
    //trajectory->getWaypoints();

    // play

    return convertToManipulatorWayPoints(completeTrajectory2(trajectory));


}

grasp::data::ItemTrajectory::Ptr ActiveSenseDemo::convertToTrajectory(const golem::Controller::State::Seq &trajectory){

    // create trajectory item
    data::Item::Ptr itemTrajectory;

    data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
    if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
    data::Handler* handler = is<data::Handler>(handlerPtr);
    if (!handler)
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
    itemTrajectory = handler->create();
    data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
    if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "ActiveSenseDemo::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());

    trajectoryIf->setWaypoints(grasp::Waypoint::make(trajectory, trajectory));


    return itemTrajectory;
}



//----------------------End Active Sense--------------------------------------------------------

} // namespace pacman
//------------------------------------------------------------------------------

//int main(int argc, char *argv[]) {
//
//    //starts Demo (internally calls createRender once)
//    return pacman::ActiveSenseDemo::Desc().main(argc, argv);
//}
