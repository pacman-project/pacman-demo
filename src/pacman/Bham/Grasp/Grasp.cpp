#include <pacman/Bham/Grasp/GraspImpl.h>
#include <Golem/Phys/Data.h>
#include <Golem/Tools/Data.h>
#include <memory>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

void BhamGraspImpl::spin() {
	try {
		main();
	}
	catch (grasp::Interrupted&) {
	}
}

//-----------------------------------------------------------------------------

BhamGraspImpl::BhamGraspImpl(golem::Scene &scene) : ShapePlanner(scene) {
}

bool BhamGraspImpl::create(const grasp::ShapePlanner::Desc& desc) {
	(void)ShapePlanner::create(desc);

	scene.setHelp(
		scene.getHelp() +
		"  A                                       PaCMan operations\n"
	);
	
	return true;
}

void BhamGraspImpl::add(const std::string& id, const Point3D::Seq& points, const ShunkDexHand::Pose::Seq& trajectory) {
}

void BhamGraspImpl::remove(const std::string& id) {
}

void BhamGraspImpl::list(std::vector<std::string>& idSeq) const {
}

void BhamGraspImpl::estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories) {
}

void BhamGraspImpl::function(TrialData::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'A':
		switch (waitKey("IE", "Press a key to (I)mport, (E)xport data...")) {
		case 'I':
		{
			// import data
			std::string path;
			readString("Enter file path: ", path);
			context.write("Importing data from: %s\n", path.c_str());
			break;
		}
		case 'E':
		{
			// export data
			std::string path;
			readString("Enter file path: ", path);
			context.write("Exporting data from: %s\n", path.c_str());
			break;
		}
		};
		context.write("Done!\n");
		break;
	};

	ShapePlanner::function(dataPtr, key);
}

void BhamGraspImpl::convert(const Point3D::Seq& src, ::grasp::Point::Seq& dst) const {
}

//-----------------------------------------------------------------------------

Context::Ptr context;
Universe::Ptr universe;

BhamGrasp* BhamGrasp::create(const std::string& path) {
	// Create XML parser and load configuration file
	XMLParser::Ptr pParser = XMLParser::load(path);

	// Find program XML root context
	XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
	if (pXMLContext == NULL)
		throw MsgApplication(Message::LEVEL_CRIT, "Unknown configuration file: %s", path.c_str());

	// Create program context
	golem::Context::Desc contextDesc;
	XMLData(contextDesc, pXMLContext);
	context = contextDesc.create(); // throws
		
	// Create Universe
	Universe::Desc universeDesc;
	XMLData(universeDesc, pXMLContext->getContextFirst("universe"));
	universe = universeDesc.create(*context);
		
	// Create scene
	Scene::Desc sceneDesc;
	XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
	Scene *pScene = universe->createScene(sceneDesc);
		
	// Launch universe
	universe->launch();

	// Setup Birmingham grasp interface
	BhamGraspImpl::Desc bhamGraspDesc;
	XMLData(bhamGraspDesc, context.get(), pXMLContext);

	BhamGraspImpl *pBhamGrasp = dynamic_cast<BhamGraspImpl*>(pScene->createObject(bhamGraspDesc)); // throws
	if (pBhamGrasp == NULL)
		throw Message(Message::LEVEL_CRIT, "BhamGrasp::create(): Unable to create Birmingham grasp interface");

	// Random number generator seed
	context->info("Random number generator seed %d\n", context->getRandSeed()._U32[0]);
	
	return pBhamGrasp;
}

//-----------------------------------------------------------------------------
