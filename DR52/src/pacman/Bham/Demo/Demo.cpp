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
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : Player(scene), modelCamera(nullptr), modelHandler(nullptr) {
}

pacman::Demo::~Demo() {
}

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws

	dataName = desc.dataName;

	grasp::Sensor::Map::const_iterator camera = sensorMap.find(desc.modelCamera);
	modelCamera = camera != sensorMap.end() ? is<Camera>(camera->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	
	grasp::data::Handler::Map::const_iterator handler = handlerMap.find(desc.modelHandler);
	modelHandler = handler != handlerMap.end() ? handler->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	
	modelItem = desc.modelItem;
	modelScanPose = desc.modelScanPose;
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo, load and estimate (M)odel, ...";
	}));
	
	// model pose scan and estimation
	menuCmdMap.insert(std::make_pair("PM", [=]() {
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
		context.write("Model load and estimation completed!\n");
	}));
	
	// main demo
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// estimate pose
		if (to<Data>(dataCurrentPtr)->modelVertices.empty() || to<Data>(dataCurrentPtr)->modelTriangles.empty())
			menuCmdMap["PM"]();

		// run demo
	}));
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
