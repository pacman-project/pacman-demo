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

	auto executeCmd = [&](const std::string command) {
		MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
		if (cmd == menuCmdMap.end())
			throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
		cmd->second();
	};


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
			// Contact query
			//grasp::data::Transform* transform = is<grasp::data::Transform>(queryGraspHandler);
			//if (!transform)
			//	throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", queryGraspHandler->getID().c_str());


			//grasp::data::Item::List list;
			//grasp::data::Item::Map::iterator mptr = to<Data>(dataCurrentPtr)->itemMap.find(modelGraspItem);
			//if (mptr == to<Data>(dataCurrentPtr)->itemMap.end())
			//	throw Message(Message::LEVEL_ERROR, "DemoDR55::estimatePose(): Does not find %s.", modelGraspItem.c_str());
			//list.insert(list.end(), mptr);

			//// retrieve model point cloud
			//grasp::data::Item::Map::iterator modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
			//if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
			//	throw Message(Message::LEVEL_ERROR, "DemoDR55::estimatePose(): Does not find %s.", modelItem.c_str());
			//list.insert(list.end(), modelPtr); // grasp on model
			//grasp::data::Item::Ptr queryGraspItemPtr = transform->transform(list);

			//

			//// insert processed object, remove old one
			//grasp::data::Item::Map::iterator queryContactPtr;
			//RenderBlock renderBlock0(*this);
			//{
			//	golem::CriticalSectionWrapper cswData(getCS());
			//	to<Data>(dataCurrentPtr)->itemMap.erase(queryGraspItem);
			//	queryContactPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(queryGraspItem, queryGraspItemPtr));
			//	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryContactPtr, to<Data>(dataCurrentPtr)->getView());
			//}
			//context.write("Transform: handler %s, inputs %s, %s...\n", queryGraspHandler->getID().c_str(), queryContactPtr->first.c_str(), modelItem.c_str());

			// Grasp with Active Sense and lift the object
			// Outcome (Exepected state of the robot at the end): 
			// at the end the object should have grasped and lifted an object 
			this->graspWithActiveSense();

			breakPoint("Did it grasp?");

			grasp::ConfigMat34 pose = getPoseFromConfig(this->objPassingPose, 7);
			this->gotoPoseWS2(pose);


			// grasp and scan object
			//breakPoint("Object grasp and point cloud capture");
			//grasp::data::Item::Map::iterator ptr = objectCapture(Data::MODE_QUERY, objectItem);

			//// wrist frame
			//const Mat34 frame = forwardTransformArm(grasp::Waypoint::lookup(*controller).state);
			//// create query densities
			//createQuery(ptr->second, frame, &to<Data>(dataCurrentPtr)->clusterCounter);
			//// generate wrist pose solutions
			//generateSolutions();
			//// select best trajectory
			//selectTrajectory();

			//breakPoint("Action execution");
			//// execute trajectory
			//performTrajectory(stopAtBreakPoint);
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


	//menuCmdMap.insert(std::make_pair("TE", [&]() {
	//	grasp::ConfigMat34::Range range = selectPoseRange(poseMap);
	//	grasp:ConfigMat34::Seq seq;
	//	for (ConfigMat34::Map::const_iterator i = range.first; i != range.second; ++i) seq.push_back(i->second);
	//	if (seq.empty()) throw Cancel("No poses");
	//	// select and go
	//	size_t index = 1;

	//	Menu::selectIndex(seq, index, "pose");
	//	gotoPose(seq[index - 1]);
	//	// done!
	//	createRender();
	//	context.write("Done!\n");
	//}));

}

//------------------------------------------------------------------------------

void DemoDR55::render() const {
	ActiveSenseDemo::render();
}



}// namespace pacman

//------------------------------------------------------------------------------

 int main(int argc, char *argv[]) {
 	return pacman::DemoDR55::Desc().main(argc, argv);
 }
