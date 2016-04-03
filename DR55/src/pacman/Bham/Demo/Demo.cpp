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
	dtfLeftPose.xmlData(xmlcontext->getContextFirst("passing dft_pose_left"));
	dtfRightPose.xmlData(xmlcontext->getContextFirst("passing dft_pose_right"));

	

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
	dtfLeftPose = desc.dtfLeftPose;
	dtfRightPose = desc.dtfRightPose;

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

			//this->setArmsToDefault(stopAtBreakPoint);

			//this->graspWithActiveSense();

			this->executePassing(stopAtBreakPoint);

			this->executePlacement(stopAtBreakPoint);
		}
		context.write("Done!\n");

	}));


	menuCmdMap.insert(std::make_pair("KT", [=]() {

		grasp::ConfigMat34 pose = getPoseFromConfig(this->objPassingPose, 7);

		bool success = this->gotoPoseWS2(pose);
		//this->gotoPose3(this->objPassingPose);

		//context.write("Success? %d\n", success);

	}));

	menuCmdMap.insert(std::make_pair("KL", [=]() {

		gotoPoseLeft(this->scanPassingPose);

	}));

	menuCmdMap.insert(std::make_pair("KS", [=]() {

		grasp::data::Item::List itemList;
		scanFromSensor(itemList, this->passingCamera, this->passingObjItem, this->passingImageHandler->getID());
		
		context.debug("Handler ID: %s\n", this->passingImageHandler->getID().c_str());

	}));

	menuCmdMap.insert(std::make_pair("KB", [=]() {

		moveRightWristBackwards(0.2, golem::SEC_TM_REAL_ZERO);

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

void DemoDR55::scanFromSensor(grasp::data::Item::List& scannedImageItems, Camera* camera, const std::string& itemLabel, const std::string& handler, ScanPoseCommand scanPoseCommand, grasp::Manager::Data::Ptr dataPtr) {
	data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(handler.empty() ? camera->getSnapshotHandler() : handler);
	if (handlerSnapshotPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", camera->getSnapshotHandler().c_str());
	data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", camera->getSnapshotHandler().c_str());

	this->dataItemLabel = itemLabel;

	const bool isEnabledDeformationMap = camera && camera->getCurrentCalibration()->isEnabledDeformationMap();
	ScopeGuard guard([&]() { if (camera) camera->getCurrentCalibration()->enableDeformationMap(isEnabledDeformationMap); });
	if (camera && camera->getCurrentCalibration()->hasDeformationMap())
	{
		context.write("ActiveSenseDemo::scanPoseActive: **** USING DEFORMATION MAP ****\n");
		camera->getCurrentCalibration()->enableDeformationMap(true);
	}

	grasp::Manager::Data* data;
	auto data_it = this->dataCurrentPtr;

	if (dataPtr.get())
		data = to<grasp::Manager::Data>(dataPtr);
	else
		data = to<grasp::Manager::Data>(data_it);


	for (bool stop = false; !stop;) {
		stop = scanPoseCommand == nullptr || !scanPoseCommand();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			grasp::data::Item::Ptr itemImage = capture->capture(*camera, [&](const grasp::TimeStamp*) -> bool { return true; });

			const data::Item::Map::iterator ptr = data->itemMap.insert(data->itemMap.end(), data::Item::Map::value_type(dataItemLabel, itemImage));

			Data::View::setItem(data->itemMap, ptr, data->getView());
			scannedImageItems.push_back(ptr);
		}
	}

	context.write("Done!\n");
}



void DemoDR55::setArmsToDefault(bool stopAtBreakPoint){
	//TODO: Send arms to default position
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

	setRightArmDeault(stopAtBreakPoint);
	setLeftArmDeault(stopAtBreakPoint);
}

void DemoDR55::setLeftArmDeault(bool stopAtBreakPoint){
	//TODO: Send arms to default position
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

	gotoPoseLeft(this->dtfLeftPose);

	breakPoint("Went to default pose left arm");

}

void DemoDR55::setRightArmDeault(bool stopAtBreakPoint){
	//TODO: Send arms to default position
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

	gotoPose3(this->dtfRightPose, golem::SEC_TM_REAL_ZERO, true);
	breakPoint("Went to default pose right arm");


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

void DemoDR55::moveRightWristBackwards(const double horizontalDistance, const SecTmReal duration){

	const golem::U32 currentPlannerIndex = plannerIndex;
	plannerIndex = 0;
	ScopeGuard guard([&]() { plannerIndex = currentPlannerIndex; });

	// vertically by verticalDistance; to hand zero config
	Mat34 pose = getWristPose(6);
	pose.p.y -= std::max(0.0, horizontalDistance);
	this->gotoPoseWS2(pose);


}

void DemoDR55::executePassing(bool stopAtBreakPoint){

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

	auto addItem = [&](const std::string& itemLabel, grasp::data::Item::Ptr& item){
		// add item to data bundle
		data::Item::Map::iterator ptr;
		RenderBlock renderBlock(*this);
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
			
			//this->gotoWristPose(this->objPassingPose, 0);
			grasp::ConfigMat34 pose = getPoseFromConfig(this->objPassingPose, 7);
			this->gotoPoseWS2(pose);
		}

	
	//**** Send left arm to scanning pose
	gotoPoseLeft(this->scanPassingPose);

	breakPoint("Gone to scan pose left arm");

	//**** Perform scan
	grasp::data::Item::List itemList;
	scanFromSensor(itemList, this->passingCamera, this->passingObjItem, this->passingImageHandler->getID());

	breakPoint("Performed scan");

	//**** Transform scan to point curv
	// Point Curv Transform
	grasp::data::Item::Ptr pointCurvItem  = transformItems(itemList, this->passingPointCurvHandler);

	data::Item::Map::iterator ptrObjItem = addItem(this->passingObjItem, pointCurvItem);

	breakPoint("Transformed scan to point curv");

	//**** Transform point curv with contact model to create contact query
	//Getting grasp contact model
	data::Item::Map::iterator ptrGraspContactModel = to<grasp::Manager::Data>(dataCurrentPtr)->itemMap.find(this->passingGraspModelItem);
	if (ptrGraspContactModel == to<grasp::Manager::Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "Grasp contact model item with label %s was not found in data bundle", this->passingGraspModelItem.c_str());

	grasp::data::Item::List list; 
	list.push_back(ptrObjItem); list.push_back(ptrGraspContactModel);
	
	// Contact query
	grasp::data::Item::Ptr queryGraspItem = transformItems(list, this->passingGraspHandler);

	data::Item::Map::iterator ptrQueryGraspItem = addItem(this->passingCQueryItem, queryGraspItem);

	breakPoint("Generated contact query");
	//**** Convert contact query into trajectory
	// pick up handler
	grasp::data::Convert* convert = grasp::is<grasp::data::Convert>(ptrQueryGraspItem);
	if (!convert)
		throw Message(Message::LEVEL_ERROR, "Input item does not support Convert interface");
	// convert
	data::Item::Ptr graspTrajectory = convert->convert(*passingTrajectoryHandler);

	data::Item::Map::iterator ptrGraspTrajectory = addItem(this->passingGraspTrajectoryItem, graspTrajectory);

	breakPoint("Converted contact query to trajectory");
	//**** Execute passing grasp
	data::Trajectory* trajectory = is<data::Trajectory>(ptrGraspTrajectory->second.get());
	// play
	Controller::State::Seq seq;
	trajectory->createTrajectory(seq);

	// select collision object (TODO: Set rack point cloud)
	CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
	// perform and process (prevent right hand from opening)

	{
		const golem::U32 currentPlannerIndex = plannerIndex;
		plannerIndex = 1; // Explicitly setting left side planner
		ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });

		performAndProcess(dataCurrentPtr->first, ptrGraspTrajectory->first, seq);
		breakPoint("Executed trajectory");
	}

	releaseRightHand(1.0, 2.0);
	moveRightWristBackwards(0.2, golem::SEC_TM_REAL_ZERO);

}

}// namespace pacman

//------------------------------------------------------------------------------

 int main(int argc, char *argv[]) {
 	return pacman::DemoDR55::Desc().main(argc, argv);
 }
