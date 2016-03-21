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


#include "pacman/Bham/ActiveSenseGrasp/Demo/BaseDemo.h"

using namespace golem;
using namespace grasp;

namespace pacman {

//-----------------------------------------------------------------------------

void BaseDemo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("bundle", bundle, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("camera", camera, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("handler", imageHandler, xmlcontext->getContextFirst("image"));
	golem::XMLData("item", imageItem, xmlcontext->getContextFirst("image"));

	golem::XMLData("handler", processHandler, xmlcontext->getContextFirst("process"));
	golem::XMLData("item", processItem, xmlcontext->getContextFirst("process"));

	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));

	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));

	golem::XMLData("handler", trjHandler, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("item", trjItem, xmlcontext->getContextFirst("trajectory"));

	grasp::XMLData(detectFilterDesc, xmlcontext->getContextFirst("detection"));
	golem::XMLData("thread_chunk_size", detectThreadChunkSize, xmlcontext->getContextFirst("detection"));
	try {
		XMLData(detectRegionDesc, detectRegionDesc.max_size(), xmlcontext->getContextFirst("detection"), "bounds");
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	golem::XMLData("min_size", detectMinSize, xmlcontext->getContextFirst("detection"));
	golem::XMLData("delta_size", detectDeltaSize, xmlcontext->getContextFirst("detection"));
	golem::XMLData("delta_depth", detectDeltaDepth, xmlcontext->getContextFirst("detection"));

	poseScanSeq.clear();
	XMLData(poseScanSeq, poseScanSeq.max_size(), xmlcontext->getContextFirst("scan"), "pose");
	poseActionSeq.clear();
	XMLData(poseActionSeq, poseActionSeq.max_size(), xmlcontext->getContextFirst("action"), "pose");
}

//------------------------------------------------------------------------------

BaseDemo::BaseDemo(Scene &scene) : Player(scene), camera(nullptr), imageHandler(nullptr), processHandler(nullptr), modelHandler(nullptr), queryHandler(nullptr), trjHandler(nullptr) {
}

BaseDemo::~BaseDemo() {
}

void BaseDemo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::BaseDemo::Desc."));

	// create object
	Player::create(desc); // throws

	grasp::Sensor::Map::const_iterator cameraPtr = sensorMap.find(desc.camera);
	camera = cameraPtr != sensorMap.end() ? is<CameraDepth>(cameraPtr->second.get()) : nullptr;
	Assert::valid(camera != nullptr, "pacman::BaseDemo::create(): unknown depth camera: %s", desc.camera.c_str());
	
	bundle = desc.bundle;

	grasp::data::Handler::Map::const_iterator imageHandlerPtr = handlerMap.find(desc.imageHandler);
	imageHandler = imageHandlerPtr != handlerMap.end() ? imageHandlerPtr->second.get() : nullptr;
	Assert::valid(imageHandler != nullptr, "pacman::BaseDemo::create(): unknown image handler: %s", desc.imageHandler.c_str());
	imageItem = desc.imageItem;

	grasp::data::Handler::Map::const_iterator processHandlerPtr = handlerMap.find(desc.processHandler);
	processHandler = processHandlerPtr != handlerMap.end() ? processHandlerPtr->second.get() : nullptr;
	Assert::valid(processHandler != nullptr, "pacman::BaseDemo::create(): unknown process handler: %s", desc.processHandler.c_str());
	processItem = desc.processItem;

	grasp::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	Assert::valid(modelHandler != nullptr, "pacman::BaseDemo::create(): unknown contact model handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;

	grasp::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	Assert::valid(queryHandler != nullptr, "pacman::BaseDemo::create(): unknown contact query handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;

	grasp::data::Handler::Map::const_iterator trjHandlerPtr = handlerMap.find(desc.trjHandler);
	trjHandler = trjHandlerPtr != handlerMap.end() ? trjHandlerPtr->second.get() : nullptr;
	Assert::valid(trjHandler != nullptr, "pacman::BaseDemo::create(): unknown trajectory handler: %s", desc.trjHandler.c_str());
	trjItem = desc.trjItem;

	detectFilterDesc = desc.detectFilterDesc;
	detectThreadChunkSize = desc.detectThreadChunkSize;
	detectRegionDesc = desc.detectRegionDesc;
	detectMinSize = desc.detectMinSize;
	detectDeltaSize = desc.detectDeltaSize;
	detectDeltaDepth = desc.detectDeltaDepth;

	poseScanSeq = desc.poseScanSeq;
	poseActionSeq = desc.poseActionSeq;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  G                                       menu Grasp Demo\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("G", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo...";
	}));
	menuCmdMap.insert(std::make_pair("GD", [=]() {
		// select data bundle
		data::Data::Map::iterator dataPtr = dataMap.find(bundle);
		if (dataPtr == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "unknown data bundle: %s", bundle.c_str());
		setCurrentDataPtr(dataPtr);

		// debug mode
		const bool stopAtBreakPoint = option("YN", "Debug mode (Y/N)...") == 'Y';
		const auto breakPoint = [=](const char* str) {
			if (!stopAtBreakPoint && str)
				context.write("%s....\n", str);
			if ( stopAtBreakPoint && str ? option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y' : waitKey(5) == 27)
				throw Cancel("Demo cancelled");
		};

		Cloud::PointSeq prev, next;
		Cloud::PointSeqQueue pointSeqQueue(detectFilterDesc.window);

		ScopeGuard cleanup([&]() {
			camera->set(Camera::CMD_STOP);
			camera->setProperty(camera->getImageProperty());
			UI::addCallback(*this, getCurrentHandler());
		});

		for (;;) {
			// remove temp items
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper cswData(scene.getCS());
				demoRenderer.reset();
				to<Data>(dataCurrentPtr)->itemMap.erase(imageItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(processItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(queryItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(trjItem);
			}

			// capture depth images
			data::Item::List list;
			for (Pose::Seq::const_iterator j = poseScanSeq.begin(); j != poseScanSeq.end(); ++j) {
				UI::addCallback(*this, getCurrentHandler());
				UI::removeCallback(*this, getCurrentHandler());

				// go to object detection pose
				breakPoint("Going to detection pose");
				gotoPose(*j);
				breakPoint("Waiting for object to appear");

				// detect changes only at the first pose
				const bool detection = j == poseScanSeq.begin();

				// detect changes
				if (detection) {
					const Mat34 cameraFrame = camera->getFrame();

					// Detection region in global coordinates
					golem::Bounds::Seq detectRegion;
					for (golem::Bounds::Desc::Seq::const_iterator i = detectRegionDesc.begin(); i != detectRegionDesc.end(); ++i)
						detectRegion.push_back((*i)->create());
					// Move bounds to camera frame
					Mat34 cameraFrameInv;
					cameraFrameInv.setInverse(cameraFrame);
					for (golem::Bounds::Seq::const_iterator i = detectRegion.begin(); i != detectRegion.end(); ++i)
						(*i)->multiplyPose(cameraFrameInv, (*i)->getPose());

					// start camera
					camera->setProperty(camera->getDepthProperty());
					if (!camera->set(Camera::CMD_VIDEO))
						throw Message(Message::LEVEL_ERROR, "unable to start camera");

					pointSeqQueue.clear();
					next.clear();
					prev.clear();
					U32 prevSize = 0;

					for (bool accept = false; !accept;) {
						breakPoint(nullptr);

						// clear image queue
						golem::SecTmReal timeStamp = context.getTimer().elapsed();
						// retrieve a next frame
						for (bool stop = false; !stop;)
							camera->waitAndPeek([&](const Image::List& images, bool result) {
							if (!result || images.empty())
								throw Message(Message::LEVEL_ERROR, "unable to capture image");
							// search from the back, i.e. the latest frames
							for (Image::List::const_reverse_iterator i = images.rbegin(); i != images.rend(); ++i)
								if (timeStamp < (*i)->timeStamp || next.empty()) {
									timeStamp = (*i)->timeStamp;
									Image::assertData((*i)->cloud);
									next = *(*i)->cloud;
									stop = true;
								}
						});

						// clip region
						try {
							if (!detectRegion.empty())
								Cloud::regionClip(context, detectRegion, detectThreadChunkSize, next, true);
							// transform points
							Cloud::transform(cameraFrame, next, next);
						}
						catch (const Message&) {}

						// filter
						pointSeqQueue.push_back(next);
						if (!pointSeqQueue.full())
							continue;
						Cloud::filter(context, detectFilterDesc, detectThreadChunkSize, pointSeqQueue, next);

						// debug accept?
						const bool debugAccept = waitKey(0) == 'D';

						// count and render points
						U32 nextSize = 0;
						{
							golem::CriticalSectionWrapper csw(scene.getCS());
							demoRenderer.reset();
							for (Cloud::PointSeq::const_iterator i = next.begin(); i != next.end(); ++i)
								if (!Cloud::isNanXYZ(*i)) {
									demoRenderer.addPoint(Vec3(i->x, i->y, i->z), RGBA(i->r, i->g, i->b, 255));
									++nextSize;
								}
						}
						if (!debugAccept && nextSize < detectMinSize)
							continue;

						if (prev.size() == next.size()) {
							size_t sharedSize = 0;
							Real deltaDepth = REAL_ZERO;
							for (size_t i = 0; i < next.size(); ++i) {
								const Cloud::Point p = prev[i], n = next[i];
								if (Cloud::isNanXYZ(p) || Cloud::isNanXYZ(n))
									continue;
								++sharedSize;
								deltaDepth += Math::sqrt(Math::sqr(p.x - n.x) + Math::sqr(p.y - n.y) + Math::sqr(p.z - n.z));
							}
							if (debugAccept || (nextSize - sharedSize < detectDeltaSize && deltaDepth / sharedSize < detectDeltaDepth)) {
								breakPoint(makeString("Object detected (size=%u, delta_size=%u, delta_depth=%f)", nextSize, nextSize - sharedSize, deltaDepth / sharedSize).c_str());
								accept = true;
							}
						}
						if (!accept) {
							prev = next;
							prevSize = nextSize;
						}
					}
				}
				else
					breakPoint(nullptr);

				// capture image
				data::Capture* capture = is<data::Capture>(imageHandler);
				Assert::valid(capture != nullptr, "Handler %s does not support Capture interface", imageHandler->getID().c_str());

				// capture image
				data::Item::Ptr item = capture->capture(*camera, [&](const grasp::TimeStamp*) -> bool { return true; });
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper cswData(scene.getCS());
					demoRenderer.reset();
					data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(imageItem, item));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
					// insert image to the processing list
					list.push_back(ptr);
				}
			}

			// process images
			breakPoint("Processing images");
			data::Transform* processTransform = is<data::Transform>(processHandler);
			Assert::valid(processTransform != nullptr, "Handler %s does not support Transform interface", processHandler->getID().c_str());

			data::Item::Ptr processItem = processTransform->transform(list);
			data::Item::Map::iterator processPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper cswData(scene.getCS());
				processPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->processItem, processItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, processPtr, to<Data>(dataCurrentPtr)->getView());
			}

			// process images
			breakPoint("Performing query");
			data::Item::Map::iterator modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
			if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "unable to find model item: %s", modelItem.c_str());
			list.clear();
			list.push_back(modelPtr);
			list.push_back(processPtr);
			data::Transform* queryTransform = is<data::Transform>(queryHandler);
			Assert::valid(queryTransform != nullptr, "Handler %s does not support Transform interface", queryHandler->getID().c_str());

			data::Item::Ptr queryItem = queryTransform->transform(list);
			data::Item::Map::iterator queryPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper cswData(scene.getCS());
				queryPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->queryItem, queryItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryPtr, to<Data>(dataCurrentPtr)->getView());
			}

			// select trajectory
			breakPoint("Selecting trajectory");
			data::Convert* queryConvert = is<data::Convert>(&*queryItem);
			Assert::valid(queryConvert != nullptr, "Item %s does not support Convert interface", this->queryItem.c_str());

			data::Item::Map::iterator trjPtr;
			data::Item::Ptr trjItem;
			try {
				InputBlock inputBlock(*this);
				trjItem = queryConvert->convert(*trjHandler);
			}
			catch (const Message& msg) {
				context.write("%s\n", msg.what());
				continue;
			}
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper cswData(scene.getCS());
				trjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->trjItem, trjItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, trjPtr, to<Data>(dataCurrentPtr)->getView());
			}

			// export trajectory
			UI::removeCallback(*this, getCurrentHandler());
			data::Trajectory* trajectory = is<data::Trajectory>(&*trjItem);
			Controller::State::Seq seq;
			{
				InputBlock inputBlock(*this);
				trajectory->createTrajectory(seq);
			}
			// select collision object
			CollisionBounds::Ptr collisionBounds;
			const data::Point3D* point = is<const data::Point3D>(&*processItem);
			if (point) {
				collisionBounds.reset(new CollisionBounds(*getPlanner().planner, [=](size_t i, Vec3& p) -> bool {
					if (i < point->getNumOfPoints()) p = point->getPoint(i); return i < point->getNumOfPoints();
				}, &demoRenderer, &scene.getCS()));

				golem::CriticalSectionWrapper csw(scene.getCS());
				for (size_t i = point->getNumOfPoints(); i > 0;)
					demoRenderer.addPoint(Vec3((const grasp::data::Point3D::Vec3&)point->getPoint(--i)), RGBA::BLACK);
			}
			// find global trajectory & perform
			perform(dataCurrentPtr->first, this->trjItem, seq, stopAtBreakPoint);
			UI::removeCallback(*this, getCurrentHandler());
			// goto remaining poses
			for (Pose::Seq::const_iterator i = poseActionSeq.begin(); i != poseActionSeq.end(); ++i) {
				Pose pose = *i;
				// use hand configuration
				const Real* c = pose.flags == "open" ? seq.front().cpos.data() : pose.flags == "close" ? seq.back().cpos.data() : nullptr;
				if (c) std::copy(c + *getPlanner().handInfo.getJoints().begin(), c + *getPlanner().handInfo.getJoints().end(), pose.c.data() + *getPlanner().handInfo.getJoints().begin());
				// go to pose
				breakPoint("Going to action pose");
				gotoPose(pose);
			}

			// done!
			context.write("Done!\n");
		}
	}));
}

//------------------------------------------------------------------------------

void BaseDemo::render() const {
	Player::render();
	demoRenderer.render();
}

//------------------------------------------------------------------------------

void BaseDemo::gotoPose(const Pose& pose) {
	// current state
    golem::Controller::State begin = grasp::Waypoint::lookup(*controller).state;
	//context.debug("STATE[1]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
	
	// target
	golem::Controller::State end = begin;
	if (pose.configspace)
		end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));

	Mat34 wpose;
	if (pose.position || pose.orientation) {
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(begin.cpos, wcc);
		wpose.multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
		if (pose.position) wpose.p = pose.w.p;
		if (pose.orientation) wpose.R = pose.w.R;
	}

	// find trajectory
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, pose.configspace ? &end : nullptr, pose.position || pose.orientation ? &wpose : nullptr, pose.dt, trajectory);
	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	//begin = lookupState();
	//context.debug("STATE[2]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
	//begin = lookupState();
	//context.debug("STATE[3]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
}

//------------------------------------------------------------------------------

void XMLData(BaseDemo::Pose &val, golem::XMLContext* xmlcontext, bool create) {
	if (create) {
		grasp::XMLData((ConfigMat34&)val, xmlcontext, true);
		golem::XMLData("dt", val.dt, xmlcontext, true);
		golem::XMLData("flags", val.flags, xmlcontext, true);
	}
	else {
		val.configspace = false;
		val.position = false;
		val.orientation = false;

		try {
			//grasp::XMLData((ConfigMat34&)val, xmlcontext, false);
			golem::XMLDataSeq(val.c, "c", xmlcontext, create, golem::REAL_ZERO);
			val.configspace = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData(val.w.p, xmlcontext);
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Quat quat;
			golem::XMLData(quat, xmlcontext);
			val.w.R.fromQuat(quat);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real angle;
			golem::Vec3 axis;
			golem::XMLDataAngleAxis(angle, axis, xmlcontext);
			val.w.R.fromAngleAxis(angle, axis);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real roll, pitch, yaw;
			golem::XMLDataEuler(roll, pitch, yaw, xmlcontext);
			val.w.R.fromEuler(roll, pitch, yaw);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData(val.w.R, xmlcontext);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Twist twist;
			golem::Real theta;
			golem::XMLData(twist, theta, xmlcontext);
			val.w.fromTwist(twist, theta);
			val.orientation = true;
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}

		if (!val.configspace && !val.position && !val.orientation)
			throw Message(Message::LEVEL_ERROR, "XMLData(): %s: invalid pose description", xmlcontext->getName().c_str());

		try {
			golem::XMLData("dt", val.dt, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData("flags", val.flags, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
	}
}

}// namespace pacman

//------------------------------------------------------------------------------

// int main(int argc, char *argv[]) {
// 	return BaseDemo::Desc().main(argc, argv);
// }
