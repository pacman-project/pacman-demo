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

//-----------------------------------------------------------------------------

Context::Ptr pContext;
Universe::Ptr pUniverse;

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
	pContext = contextDesc.create(); // throws
		
	// Create Universe
	Universe::Desc universeDesc;
	XMLData(universeDesc, pXMLContext->getContextFirst("universe"));
	pUniverse = universeDesc.create(*pContext);
		
	// Create scene
	Scene::Desc sceneDesc;
	XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
	Scene *pScene = pUniverse->createScene(sceneDesc);
		
	// Launch universe
	pUniverse->launch();

	// Setup Birmingham grasp interface
	BhamGraspImpl::Desc bhamGraspDesc;
	XMLData(bhamGraspDesc, pContext.get(), pXMLContext);

	BhamGraspImpl *pBhamGrasp = dynamic_cast<BhamGraspImpl*>(pScene->createObject(bhamGraspDesc)); // throws
	if (pBhamGrasp == NULL)
		throw Message(Message::LEVEL_CRIT, "BhamGrasp::create(): Unable to create Birmingham grasp interface");

	// Random number generator seed
	pContext->info("Random number generator seed %d\n", pContext->getRandSeed()._U32[0]);
	
	return pBhamGrasp;
}

//-----------------------------------------------------------------------------
