#include <pacman/Bham/Demo/Demo.h>
#include <Grasp/Core/Import.h>
#include <Golem/Phys/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

data::Data::Ptr Demo::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new Demo::Data(context));
	static_cast<Demo::Data*>(data.get())->create(*this);
	return data;
}

Demo::Data::Data(golem::Context &context) : grasp::Player::Data(context), owner(nullptr) {
}

void Demo::Data::create(const Desc& desc) {
	Player::Data::create(desc);

	modelVertices.clear();
	modelTriangles.clear();
}

void Demo::Data::setOwner(Manager* owner) {
	grasp::Player::Data::setOwner(owner);
	this->owner = is<Demo>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
	dataName = this->owner->dataName;
}

void Demo::Data::createRender() {
	Player::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->csRenderer);
		owner->modelRenderer.reset();
		owner->modelRenderer.setColour(owner->modelColourSolid);
		owner->modelRenderer.addSolid(modelVertices.data(), (U32)modelVertices.size(), modelTriangles.data(), (U32)modelTriangles.size());
		owner->modelRenderer.setColour(owner->modelColourWire);
		owner->modelRenderer.addWire(modelVertices.data(), (U32)modelVertices.size(), modelTriangles.data(), (U32)modelTriangles.size());
	}
}

void Demo::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const data::Handler::Map& handlerMap) {
	data::Data::load(prefix, xmlcontext, handlerMap);

	try {
		dataName.clear();
		golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (dataName.length() > 0) {
			FileReadStream frs((prefix + sepName + dataName).c_str());
			frs.read(modelVertices, modelVertices.end());
			frs.read(modelTriangles, modelTriangles.end());
		}
	}
	catch (const std::exception&) {

	}
}

void Demo::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	data::Data::save(prefix, xmlcontext);

	if (dataName.length() > 0) {
		golem::XMLData("data_name", const_cast<std::string&>(dataName), xmlcontext, true);
		FileWriteStream fws((prefix + sepName + dataName).c_str());
		fws.write(modelVertices.begin(), modelVertices.end());
		fws.write(modelTriangles.begin(), modelTriangles.end());
	}
}

//------------------------------------------------------------------------------

void Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	modelScanPose.xmlData(xmlcontext->getContextFirst("model scan_pose"));
	golem::XMLData(modelColourSolid, xmlcontext->getContextFirst("model colour solid"));
	golem::XMLData(modelColourWire, xmlcontext->getContextFirst("model colour wire"));

	golem::XMLData("sensor", graspSensorForce, xmlcontext->getContextFirst("grasp"));
	golem::XMLData(graspThresholdForce, xmlcontext->getContextFirst("grasp threshold"));
	golem::XMLData("event_time_wait", graspEventTimeWait, xmlcontext->getContextFirst("grasp"));
	golem::XMLData("close_duration", graspCloseDuration, xmlcontext->getContextFirst("grasp"));
	graspPoseOpen.xmlData(xmlcontext->getContextFirst("grasp pose_open"));
	graspPoseClosed.xmlData(xmlcontext->getContextFirst("grasp pose_closed"));

	golem::XMLData("camera", objectCamera, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler_scan", objectHandlerScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler", objectHandler, xmlcontext->getContextFirst("object"));
	golem::XMLData("item_scan", objectItemScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("item", objectItem, xmlcontext->getContextFirst("object"));
	objectScanPoseSeq.clear();
	XMLData(objectScanPoseSeq, objectScanPoseSeq.max_size(), xmlcontext->getContextFirst("object"), "scan_pose");
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : 
	Player(scene), modelCamera(nullptr), modelHandler(nullptr), graspSensorForce(nullptr), objectCamera(nullptr), objectHandlerScan(nullptr), objectHandler(nullptr)
{}

pacman::Demo::~Demo() {
}

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::Demo::Desc."));

	// create object
	Player::create(desc); // throws

	dataName = desc.dataName;

	grasp::Sensor::Map::const_iterator modelCameraPtr = sensorMap.find(desc.modelCamera);
	modelCamera = modelCameraPtr != sensorMap.end() ? is<Camera>(modelCameraPtr->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	grasp::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;
	modelScanPose = desc.modelScanPose;
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	grasp::Sensor::Map::const_iterator graspSensorForcePtr = sensorMap.find(desc.graspSensorForce);
	graspSensorForce = graspSensorForcePtr != sensorMap.end() ? is<FT>(graspSensorForcePtr->second.get()) : nullptr;
	if (!graspSensorForce)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown grasp F/T sensor: %s", desc.graspSensorForce.c_str());
	graspThresholdForce = desc.graspThresholdForce;
	graspEventTimeWait = desc.graspEventTimeWait;
	graspCloseDuration = desc.graspCloseDuration;
	graspPoseOpen = desc.graspPoseOpen;
	graspPoseClosed = desc.graspPoseClosed;

	grasp::Sensor::Map::const_iterator objectCameraPtr = sensorMap.find(desc.objectCamera);
	objectCamera = objectCameraPtr != sensorMap.end() ? is<Camera>(objectCameraPtr->second.get()) : nullptr;
	if (!objectCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object capture camera: %s", desc.objectCamera.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerScanPtr = handlerMap.find(desc.objectHandlerScan);
	objectHandlerScan = objectHandlerScanPtr != handlerMap.end() ? objectHandlerScanPtr->second.get() : nullptr;
	if (!objectHandlerScan)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object (scan) data handler: %s", desc.objectHandlerScan.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerPtr = handlerMap.find(desc.objectHandler);
	objectHandler = objectHandlerPtr != handlerMap.end() ? objectHandlerPtr->second.get() : nullptr;
	if (!objectHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown object (process) data handler: %s", desc.objectHandler.c_str());
	objectItemScan = desc.objectItemScan;
	objectItem = desc.objectItem;
	objectScanPoseSeq = desc.objectScanPoseSeq;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo, (E)stimate model pose, (A)ttach object, add (M)odel, test (Query)...";
	}));
	
	// model pose estimation
	menuCmdMap.insert(std::make_pair("PE", [=]() {
		// run robot
		gotoPose(modelScanPose);
		// block keyboard and mouse interaction
		InputBlock inputBlock(*this);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			to<Data>(dataCurrentPtr)->modelVertices.clear();
			to<Data>(dataCurrentPtr)->modelTriangles.clear();
		}
		// obtain snapshot handler
		data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(modelCamera->getSnapshotHandler());
		if (handlerSnapshotPtr == handlerMap.end())
			throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", modelCamera->getSnapshotHandler().c_str());
		// capture and insert data
		data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
		if (!capture)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", modelCamera->getSnapshotHandler().c_str());
		data::Item::Map::iterator ptr;
		data::Item::Ptr item = capture->capture(*modelCamera, [&](const grasp::TimeStamp*) -> bool { return true; });
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// generate features
		data::Transform* transform = is<data::Transform>(handlerSnapshotPtr);
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", modelCamera->getSnapshotHandler().c_str());
		data::Item::List list;
		list.insert(list.end(), ptr);
		item = transform->transform(list);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// estimate pose
		data::Model* model = is<data::Model>(ptr);
		if (!model)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Model interface", ptr->first.c_str());
		grasp::ConfigMat34 robotPose;
		golem::Mat34 modelPose;
		Vec3Seq modelVertices;
		TriangleSeq modelTriangles;
		model->model(robotPose, modelPose, &modelVertices, &modelTriangles);
		// create triangle mesh
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->modelVertices = modelVertices;
			to<Data>(dataCurrentPtr)->modelTriangles = modelTriangles;
		}
		//grasp::Contact3D::Triangle::Seq modelMesh;
		//for (grasp::TriangleSeq::const_iterator j = modelTriangles.begin(); j != modelTriangles.end(); ++j)
		//	to<Data>(dataCurrentPtr)->modelMesh.push_back(Contact3D::Triangle(modelVertices[j->t1], modelVertices[j->t3], modelVertices[j->t2]));
		// done
		context.write("Model pose estimation completed!\n");
	}));
	
	// model attachement
	menuCtrlMap.insert(std::make_pair("PA", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (C)apture/(L)oad object...";
	}));
	menuCmdMap.insert(std::make_pair("PAC", [=]() {
		// grasp and scan object
		const data::Item::Ptr item = objectGraspAndCapture();
		// compute features and add to data bundle
		grasp::data::Item::Map::iterator ptr = objectProcessAndAdd(item);
		// attach object to the robot's end-effector
		objectAttach(ptr->second);
	}));
	menuCmdMap.insert(std::make_pair("PAL", [=]() {
		data::Item::Ptr item;

		// attach object to the robot's end-effector
		objectAttach(item);
	}));

	// main demo
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// estimate pose
		if (to<Data>(dataCurrentPtr)->modelVertices.empty() || to<Data>(dataCurrentPtr)->modelTriangles.empty())
			menuCmdMap["PE"]();

		// run demo
		for (;;) {
			// grasp and scan object
			const data::Item::Ptr item = objectGraspAndCapture();
			// compute features and add to data bundle
			(void)objectProcessAndAdd(item);
		}
	}));
}

//------------------------------------------------------------------------------

// 1. move robot to grasp open pose
// 2. wait for force event; reset bias and wait for change > threshold (set in xml)
// 3. grasp object (closed pose - generate in virtual env; strong enough for both plate and cup; only fingers should move)
// full hand grasp, fingers spread out; thumb in centre
// 4. move through scan poses and capture object, add as objectScan
// return ptr to Image Item
grasp::data::Item::Ptr pacman::Demo::objectGraspAndCapture() {
	data::Item::Ptr item;

	//attach object
	//add model

	// TODO SZ
	return item;
}

//------------------------------------------------------------------------------

// Process object image and add to data bundle
grasp::data::Item::Map::iterator pacman::Demo::objectProcessAndAdd(const grasp::data::Item::Ptr& item) {
	grasp::data::Item::Map::iterator ptr;
	return ptr;
}

//------------------------------------------------------------------------------

// Attach object to the robot's end-effector
void pacman::Demo::objectAttach(const grasp::data::Item::Ptr& item) {

}

//------------------------------------------------------------------------------

void pacman::Demo::render() const {
	Player::render();
	golem::CriticalSectionWrapper cswRenderer(csRenderer);
	modelRenderer.render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
