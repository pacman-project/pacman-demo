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

	drainerPose.setId();
	drainerVertices.clear();
	drainerTriangles.clear();
	drainerModelTriangles.clear();
}

void Demo::Data::setOwner(Manager* owner) {
	grasp::Player::Data::setOwner(owner);
	this->owner = is<Demo>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
}

void Demo::Data::createRender() {
	Player::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->csRenderer);
		owner->drainerRenderer.reset();
		owner->drainerRenderer.setColour(owner->drainerColourSolid);
		owner->drainerRenderer.addSolid(drainerVertices.data(), (U32)drainerVertices.size(), drainerTriangles.data(), (U32)drainerTriangles.size());
		owner->drainerRenderer.setColour(owner->drainerColourWire);
		owner->drainerRenderer.addWire(drainerVertices.data(), (U32)drainerVertices.size(), drainerTriangles.data(), (U32)drainerTriangles.size());
	}
}

void Demo::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const data::Handler::Map& handlerMap) {
	data::Data::load(prefix, xmlcontext, handlerMap);

	FileReadStream frs((prefix + this->owner->dataName).c_str());
	frs.read(drainerPose);
	frs.read(drainerVertices, drainerVertices.end());
	frs.read(drainerTriangles, drainerTriangles.end());
	frs.read(drainerModelTriangles, drainerModelTriangles.end());
}

void Demo::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	data::Data::save(prefix, xmlcontext);

	FileWriteStream fws((prefix + this->owner->dataName).c_str());
	fws.write(drainerPose);
	fws.write(drainerVertices.begin(), drainerVertices.end());
	fws.write(drainerTriangles.begin(), drainerTriangles.end());
	fws.write(drainerModelTriangles.begin(), drainerModelTriangles.end());
}

//------------------------------------------------------------------------------

void Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", drainerCamera, xmlcontext->getContextFirst("drainer"));
	golem::XMLData("handler", drainerHandler, xmlcontext->getContextFirst("drainer"));
	golem::XMLData("item", drainerItem, xmlcontext->getContextFirst("drainer"));
	drainerScanPose.xmlData(xmlcontext->getContextFirst("drainer scan_pose"));
	golem::XMLData(drainerColourSolid, xmlcontext->getContextFirst("drainer colour solid"));
	golem::XMLData(drainerColourWire, xmlcontext->getContextFirst("drainer colour wire"));
}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : Player(scene), drainerCamera(nullptr), drainerHandler(nullptr) {
}

pacman::Demo::~Demo() {
}

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws

	dataName = desc.dataName;

	grasp::Sensor::Map::const_iterator camera = sensorMap.find(desc.drainerCamera);
	drainerCamera = camera != sensorMap.end() ? is<Camera>(camera->second.get()) : nullptr;
	if (!drainerCamera)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown drainer pose estimation camera: %s", desc.drainerCamera.c_str());
	
	grasp::data::Handler::Map::const_iterator handler = handlerMap.find(desc.drainerHandler);
	drainerHandler = handler != handlerMap.end() ? handler->second.get() : nullptr;
	if (!drainerHandler)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown drainer data handler: %s", desc.drainerHandler.c_str());
	
	drainerItem = desc.drainerItem;
	drainerScanPose = desc.drainerScanPose;
	drainerColourSolid = desc.drainerColourSolid;
	drainerColourWire = desc.drainerColourWire;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo, load and estimate (M)odel...";
	}));
	
	// drainer pose scan and estimation
	menuCmdMap.insert(std::make_pair("PM", [=]() {
		// block keyboard and mouse interaction
		InputBlock inputBlock(*this);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->drainerModelTriangles.clear();
		}
		// run robot
		gotoPose(drainerScanPose);
		// obtain snapshot handler
		data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(drainerCamera->getSnapshotHandler());
		if (handlerSnapshotPtr == handlerMap.end())
			throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", drainerCamera->getSnapshotHandler().c_str());
		// capture and insert data
		data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
		if (!capture)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", drainerCamera->getSnapshotHandler().c_str());
		data::Item::Map::iterator ptr;
		data::Item::Ptr item = capture->capture(*drainerCamera, [&](const grasp::TimeStamp*) -> bool { return true; });
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(drainerItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(drainerItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// generate features
		data::Transform* transform = is<data::Transform>(handlerSnapshotPtr);
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", drainerCamera->getSnapshotHandler().c_str());
		data::Item::List list;
		list.insert(list.end(), ptr);
		item = transform->transform(list);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->itemMap.erase(drainerItem);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(drainerItem, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// estimate pose
		data::Model* model = is<data::Model>(ptr);
		if (!model)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Model interface", ptr->first.c_str());
		grasp::ConfigMat34 robotPose;
		golem::Mat34 drainerPose;
		Vec3Seq drainerVertices;
		TriangleSeq drainerTriangles;
		model->model(robotPose, drainerPose, &drainerVertices, &drainerTriangles);
		// create triangle mesh
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(csData);
			to<Data>(dataCurrentPtr)->drainerVertices = drainerVertices;
			to<Data>(dataCurrentPtr)->drainerTriangles = drainerTriangles;
			for (grasp::TriangleSeq::const_iterator j = drainerTriangles.begin(); j != drainerTriangles.end(); ++j)
				to<Data>(dataCurrentPtr)->drainerModelTriangles.push_back(Contact3D::Triangle(drainerVertices[j->t1], drainerVertices[j->t3], drainerVertices[j->t2]));
		}
		// done
		context.write("Model load and estimation completed!\n");
	}));
	
	// main demo
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// estimate pose
		if (to<Data>(dataCurrentPtr)->drainerModelTriangles.empty())
			menuCmdMap["PM"]();

		// run demo
	}));
}

//------------------------------------------------------------------------------

void pacman::Demo::render() const {
	Player::render();
	golem::CriticalSectionWrapper cswRenderer(csRenderer);
	drainerRenderer.render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
