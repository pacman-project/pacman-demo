/** @file Grasp.cpp
 *
 * Adapated from Demo Grasp by Ermano Arruda
 *
 * @author	Marek Kopicki
 * @author Ermano Arruda
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */


#include "pacman/Bham/Demo/Demo.h"

using namespace golem;
using namespace grasp;

namespace pacman {


//-----------------------------------------------------------------------------

void DemoDR55::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	ActiveSenseDemo::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	scanPassingPose.xmlData(xmlcontext->getContextFirst("passing scan_pose"));
	objPassingPose.xmlData(xmlcontext->getContextFirst("passing passing_pose"));

	

	golem::XMLData("camera", passingCamera, xmlcontext->getContextFirst("passing"));
	golem::XMLData("handler_scan", passingImageHandler, xmlcontext->getContextFirst("passing"));
	golem::XMLData("handler_processing", passingPointCurvHandler, xmlcontext->getContextFirst("passing"));
	golem::XMLData("handler_grasp", passingGraspHandler, xmlcontext->getContextFirst("passing"));
	golem::XMLData("handler_trajectory", passingTrajectoryHandler, xmlcontext->getContextFirst("passing"));
	golem::XMLData("item_obj", passingObjItem, xmlcontext->getContextFirst("passing"));
	golem::XMLData("item_grasp", passingGraspModelItem, xmlcontext->getContextFirst("passing"));
	golem::XMLData("item_cquery", passingCQueryItem, xmlcontext->getContextFirst("passing"));
	golem::XMLData("item_trajectory", passingGraspTrajectoryItem, xmlcontext->getContextFirst("passing"));
	

}

//------------------------------------------------------------------------------

DemoDR55::DemoDR55(Scene &scene) : ActiveSenseDemo(scene) {
}

DemoDR55::~DemoDR55() {
}

void DemoDR55::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::DemoDR55::Desc."));

	// create object
	ActiveSenseDemo::create(desc); // throws

	objPassingPose = desc.objPassingPose;
	scanPassingPose = desc.scanPassingPose;

	// Contact model item
	passingGraspModelItem = desc.passingGraspModelItem;
	passingObjItem = desc.passingObjItem;
	passingCQueryItem = desc.passingCQueryItem;
	passingGraspTrajectoryItem = desc.passingGraspTrajectoryItem;


	grasp::Sensor::Map::const_iterator cameraPtr = sensorMap.find(desc.passingCamera);
	passingCamera = cameraPtr != sensorMap.end() ? is<Camera>(cameraPtr->second.get()) : nullptr;
	if (!passingCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::DemoDR55::create(): unknown passing camera: %s", desc.passingCamera.c_str());
	

	grasp::data::Handler::Map::const_iterator passingImageHandlerPtr = handlerMap.find(desc.passingImageHandler);
	passingImageHandler = passingImageHandlerPtr != handlerMap.end() ? passingImageHandlerPtr->second.get() : nullptr;
	if (!passingImageHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::DemoDR55::create(): unknown passing image handler: %s", desc.passingImageHandler.c_str());

	grasp::data::Handler::Map::const_iterator passingPointCurvHandlerPtr = handlerMap.find(desc.passingPointCurvHandler);
	passingPointCurvHandler = passingPointCurvHandlerPtr != handlerMap.end() ? passingPointCurvHandlerPtr->second.get() : nullptr;
	if (!passingPointCurvHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::DemoDR55::create(): unknown passing points curv handler: %s", desc.passingPointCurvHandler.c_str());
	

	grasp::data::Handler::Map::const_iterator passingGraspHandlerPtr = handlerMap.find(desc.passingGraspHandler);
	passingGraspHandler = passingGraspHandlerPtr != handlerMap.end() ? passingGraspHandlerPtr->second.get() : nullptr;
	if (!passingGraspHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::DemoDR55::create(): unknown passing grasp contact query handler: %s", desc.modelGraspHandler.c_str());

	grasp::data::Handler::Map::const_iterator passingTrajectoryHandlerPtr = handlerMap.find(desc.passingTrajectoryHandler);
	passingTrajectoryHandler = passingTrajectoryHandlerPtr != handlerMap.end() ? passingTrajectoryHandlerPtr->second.get() : nullptr;
	if (!passingTrajectoryHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::DemoDR55::create(): unknown passing trajectory query handler: %s", desc.passingTrajectoryHandler.c_str());



	setMenus();
}

void DemoDR55::executeCmd(const std::string& command) {
	MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
	if (cmd == menuCmdMap.end())
		throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
	cmd->second();
}

void DemoDR55::setMenus(){
	ActiveSenseDemo::setMenus();


	menuCmdMap.insert(std::make_pair("KD", [=]() {
		// debug mode
		const bool stopAtBreakPoint = option("YN", "Debug mode (Y/N)...") == 'Y';
		const auto breakPoint = [=](const char* str) {
			if (stopAtBreakPoint) {
				if (option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y')
					throw Cancel("Demo cancelled");
			}
			else {
				context.write("%s\n", str);
				(void)waitKey(0);
			}
		};

		// command keys
		std::string itemTransCmd("IT");
		std::string itemConvCmd("IC");
		std::string dataSaveCmd("DS");
		std::string itemRemoveCmd("IR");

		// estimate pose
		if (to<Data>(dataCurrentPtr)->queryVertices.empty() || to<Data>(dataCurrentPtr)->queryVertices.empty()) {
			breakPoint("Dishwasher pose estimation");
			estimatePose(Data::MODE_QUERY);
		}

		context.write("Create a predictive contact model for the grasp [%s]...\n", modelGraspItem.c_str());
		if (option("YN", "Create a new contact model? (Y/N)") == 'Y') {
			dataItemLabel = modelGraspItem;
			executeCmd(itemTransCmd);
			modelGraspItem = dataItemLabel;
		}
		context.write("done.\n");

		// run demo
		for (;;) {
			
			// Grasp with Active Sense and lift the object
			// Outcome (Exepected state of the robot at the end): 
			// at the end the object should have grasped and lifted an object 

			this->setArmsToDefault();

			this->graspWithActiveSense();

			this->executePassing();

			this->executePlacement(stopAtBreakPoint);
		}
		context.write("Done!\n");

	}));


	menuCmdMap.insert(std::make_pair("KT", [=]() {

		grasp::ConfigMat34 pose = getPoseFromConfig(this->objPassingPose, 7);

		bool success = this->gotoPoseWS2(pose);

		context.write("Success? %d\n", success);

	}));

	menuCmdMap.insert(std::make_pair("KP", [&]() {


		const golem::U32 currentPlannerIndex = plannerIndex;
		plannerIndex = 1;
		ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });


		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Trajectory* trajectory = is<data::Trajectory>(item->second.get());
		// play
		Controller::State::Seq seq;
		trajectory->createTrajectory(seq);

		// select collision object
		CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
		// perform
		performAndProcess(dataCurrentPtr->first, item->first, seq);
		// done!
		createRender();
		context.write("Done!\n");
	}));



}

//------------------------------------------------------------------------------

void DemoDR55::render() const {
	ActiveSenseDemo::render();
}

void DemoDR55::setArmsToDefault(){
	//TODO: Send arms to default position
}

void DemoDR55::executePlacement(bool stopAtBreakPoint){
	const auto breakPoint = [=](const char* str) {
		if (stopAtBreakPoint) {
			if (option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y')
				throw Cancel("Demo cancelled");
		}
		else {
			context.write("%s\n", str);
			(void)waitKey(0);
		}
	};


	// grasp and scan object
	breakPoint("Object grasp and point cloud capture");
	grasp::data::Item::Map::iterator ptr = objectGraspAndCapture(stopAtBreakPoint);

	//breakPoint("Action planning");

	// compute features and add to data bundle
	ptr = objectProcess(ptr);
	// wrist frame
	const Mat34 frame = forwardTransformArm(grasp::Waypoint::lookup(*controller).state);
	// create query densities
	createQuery(ptr->second, frame, &to<Data>(dataCurrentPtr)->clusterCounter);
	// generate wrist pose solutions
	generateSolutions();
	// select best trajectory
	selectTrajectory();

	breakPoint("Action execution");
	// execute trajectory
	performTrajectory(stopAtBreakPoint);
}

void DemoDR55::executePassing(){

	auto addItem = [&](const std::string& itemLabel, grasp::data::Item::Ptr& item){
		// add item to data bundle
		data::Item::Map::iterator ptr;
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<grasp::Manager::Data>(dataCurrentPtr)->itemMap.erase(itemLabel);
			ptr = to<grasp::Manager::Data>(dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(dataCurrentPtr)->getView());
		}

		return ptr;
	};

	auto transformItems = [&](const grasp::data::Item::List& itemList, grasp::data::Handler* handler){

		grasp::data::Transform* transform = is<grasp::data::Transform>(handler);
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", handler->getID().c_str());

		grasp::data::Item::Ptr transformedItem = transform->transform(itemList);
		return transformedItem;
	};

	auto convertToTrajectory = [&]() {};

	//**** Moves right hand to the passing pose
		{
			const golem::U32 currentPlannerIndex = plannerIndex;
			plannerIndex = 0; // Explicitly setting right side planner
			ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });
			
			grasp::ConfigMat34 pose = getPoseFromConfig(this->objPassingPose, 7);
			bool success = this->gotoPoseWS2(pose);
		}

	
	//**** Send left arm to scanning pose
	gotoPoseLeft(this->scanPassingPose);

	//**** Perform scan
	grasp::data::Item::List itemList;
	scanPoseActiveSensor(itemList, this->passingObjItem, this->passingImageHandler->getID());

	//**** Transform scan to point curv
	// Point Curv Transform
	grasp::data::Item::Ptr pointCurvItem  = transformItems(itemList, this->passingPointCurvHandler);

	data::Item::Map::iterator ptrObjItem = addItem(this->passingObjItem, pointCurvItem);

	//**** Transform point curv with contact model to create contact query
	// Contact query
	grasp::data::Item::List list; list.push_back(ptrObjItem);

	grasp::data::Item::Ptr queryGraspItem = transformItems(itemList, this->passingGraspHandler);

	data::Item::Map::iterator ptrQueryGraspItem = addItem(this->passingCQueryItem, queryGraspItem);

	//**** Convert contact query into trajectory
	// pick up handler
	grasp::data::Convert* convert = grasp::is<grasp::data::Convert>(ptrQueryGraspItem);
	if (!convert)
		throw Message(Message::LEVEL_ERROR, "Input item does not support Convert interface");
	// convert
	data::Item::Ptr graspTrajectory = convert->convert(*passingTrajectoryHandler);

	data::Item::Map::iterator ptrGraspTrajectory = addItem(this->passingGraspTrajectoryItem, graspTrajectory);


	//**** Execute passing grasp
	data::Trajectory* trajectory = is<data::Trajectory>(ptrGraspTrajectory->second.get());
	// play
	Controller::State::Seq seq;
	trajectory->createTrajectory(seq);

	// select collision object (TODO: Set rack point cloud)
	CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
	// perform and process (prevent right hand from opening)
	performAndProcess(dataCurrentPtr->first, ptrGraspTrajectory->first, seq);	

}

}// namespace pacman

//------------------------------------------------------------------------------

 int main(int argc, char *argv[]) {
 	return pacman::DemoDR55::Desc().main(argc, argv);
 }
