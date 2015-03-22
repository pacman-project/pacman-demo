#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Tools/XMLData.h>
#include <pacman/Bham/Demo/Demo.h>

using namespace pacman;
using namespace golem;
using namespace grasp;


//------------------------------------------------------------------------------

void pacman::HypothesisSensor::Appearance::load(const golem::XMLContext* xmlcontext) {


	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData("show", frameShow, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData(shapeColour, xmlcontext->getContextFirst("shape"), false);
	golem::XMLData("show", shapeShow, xmlcontext->getContextFirst("shape"), false);


}

//------------------------------------------------------------------------------

void pacman::HypothesisSensor::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {


	appearance.load(xmlcontext->getContextFirst("appearance"));
	try {
		golem::XMLData(shapeDesc, shapeDesc.max_size(), xmlcontext->getContextFirst("appearance shape"), "bounds", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}



}

//------------------------------------------------------------------------------
pacman::HypothesisSensor::HypothesisSensor(Mat34 pose, golem::RGBA shapeColour) {


	pacman::HypothesisSensor::Desc desc;

	//Appearance of the Hypothesis Sensor
	desc.appearance.frameSize.set(0.05, 0.05, 0.05);
	desc.appearance.frameShow = true;
	desc.appearance.shapeShow = true;
	desc.appearance.shapeColour = shapeColour;

	//Bounds description
	golem::BoundingBox::Desc boundDesc;
	boundDesc.dimensions.set(0.025, 0.025, 0.025);
	boundDesc.pose.setId();
	boundDesc.pose.p.z -= 0.025;
	golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));

	desc.shapeDesc.push_back(descBox);

	//Local view frame 
	//Frame is attached on the frontal face of the cube representing this Hypothesis Sensor
	desc.viewFrame.setId();
	desc.viewFrame.p.z = 0.05;

	//Pose with respect to base frame
	this->pose = pose;
	this->create(desc);


}
pacman::HypothesisSensor::HypothesisSensor(golem::Context& context) : pose(Mat34::identity()), viewFrame(Mat34::identity()) {

}

pacman::HypothesisSensor::HypothesisSensor(const pacman::HypothesisSensor::Desc& desc) : pose(Mat34::identity()), viewFrame(Mat34::identity())
{
	this->create(desc);
}

void pacman::HypothesisSensor::create(const pacman::HypothesisSensor::Desc& desc) {

	this->viewFrame = desc.viewFrame;

	appearance = desc.appearance;
	shape.clear();
	shapeFrame.clear();
	for (golem::Bounds::Desc::Seq::const_iterator i = desc.shapeDesc.begin(); i != desc.shapeDesc.end(); ++i) {
		shape.push_back((*i)->create());
		shapeFrame.push_back(shape.back()->getPose());
	}
}


//------------------------------------------------------------------------------

void pacman::HypothesisSensor::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {


	const Mat34 frame = this->getFrame();
	if (appearance.shapeShow){
		for (size_t i = 0; i < shape.size(); ++i) {
			shape[i]->setPose(frame*shapeFrame[i]);
			renderer.setColour(appearance.shapeColour);
			renderer.addSolid(*shape[i]);
		}
	}
	if (appearance.frameShow) {
		renderer.addAxes3D(frame, appearance.frameSize);
	}
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene)
{

	this->setGLView(scene, this->getFrame());
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame)
{
	golem::CriticalSectionWrapper csw(scene.getCSOpenGL());
	OpenGL::Seq openGL = scene.getOpenGL();

	const golem::Mat34 frame = sensorFrame;
	OpenGL opengl = openGL[0];

	frame.p.get(opengl.viewPoint.v);
	frame.R.getColumn(2, opengl.viewDir);
	//frame.R.getColumn(0, opengl.viewUp); //Up right sensor
	frame.R.getColumn(0, -opengl.viewUp); //Upside down sensor

	opengl.viewDir.normalise();
	opengl.viewUp.normalise();


	scene.setOpenGL(opengl);
}



//ActiveSense------------------------------------------------------------------------------

//Constants and Enums
const std::string ActiveSense::DFT_IMAGE_ITEM_LABEL = "ActiveSense_ImageItem";
const std::string ActiveSense::DFT_POINT_CURV_ITEM_LABEL = "ActiveSense_PointCurvItem";
//END


ActiveSense::ActiveSense(golem::Context& context)
{
	rand.setRandSeed(context.getRandSeed());
}



void pacman::ActiveSense::removeItems(pacman::Demo& demo, grasp::data::Item::List& list)
{
	for (grasp::data::Item::List::const_iterator i = list.begin(); i != list.end(); ++i) {
		//Removing items
		grasp::UI::removeCallback(demo, &(*i)->second->getHandler());
		golem::CriticalSectionWrapper cswData(demo.csData);
		demo.dataCurrentPtr->second->itemMap.erase(*i);
	}

	
	//grasp::UI::addCallback(demo, demo.getCurrentHandler());
	//demo.createRender();
}

void ActiveSense::addItem(pacman::Demo& demo, const std::string& label, grasp::data::Item::Ptr item)
{
	golem::CriticalSectionWrapper cswData(demo.csData);
	const data::Item::Map::iterator ptr = to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(label, item));
	Manager::RenderBlock renderBlock(demo);
	{
		grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demo.dataCurrentPtr)->getView());
	}
}



void pacman::ActiveSense::processItems(pacman::Demo& demo, grasp::data::Item::List& list)
{

	typedef std::vector<std::pair<data::Handler*, data::Transform*>> TransformMap;
	TransformMap transformMap;
	typedef std::function<grasp::data::Item::Ptr(grasp::data::Item::List& list, TransformMap::value_type& transformPtr)> ReduceFunc;


	ReduceFunc reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr) -> grasp::data::Item::Ptr {
		
		demo.context.write("List Size: %d", list.size());
		// transform
		Manager::RenderBlock renderBlock(demo);
		
		UI::addCallback(demo, transformPtr.first);
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demo.csData);
			const data::Item::Map::iterator ptr = to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type("ActiveSense_objectCurv", item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demo.dataCurrentPtr)->getView());
		}
		return item;
	};
	
	//Used transforms: PointsCurv+PointsCurv
	//PredictorModel+PredictorModel
	//PredictorQuery+PredictorQuery

	grasp::data::Handler::Map::iterator itImage = demo.handlerMap.find("Image+Image");
	grasp::data::Handler::Map::iterator itPointCurv = demo.handlerMap.find("PointsCurv+PointsCurv");
	grasp::data::Handler::Map::iterator itPredModel = demo.handlerMap.find("PredictorModel+PredictorModel");
	grasp::data::Handler::Map::iterator itPredQuery = demo.handlerMap.find("PredictorQuery+PredictorQuery");
	grasp::data::Transform* transformImage = is<grasp::data::Transform>(itImage);
	grasp::data::Transform* transformPointsCurv = is<grasp::data::Transform>(itPointCurv);
	grasp::data::Transform* transformPredictorModel = is<grasp::data::Transform>(itPredModel);
	grasp::data::Transform* transformPredictorQuery = is<grasp::data::Transform>(itPredQuery);

	if (!transformImage || !transformPointsCurv || !transformPredictorModel || !transformPredictorQuery)
	{
		throw Cancel("We don't have all Transforms we need!");
	}
	
	transformMap.push_back(std::make_pair(itImage->second.get(), transformImage));

	transformMap.push_back(std::make_pair(itPointCurv->second.get(), transformPointsCurv));
	//Currently unused
	transformMap.push_back(std::make_pair(itPredModel->second.get(), transformPredictorModel));
	//Currently unused
	transformMap.push_back(std::make_pair(itPredQuery->second.get(), transformPredictorQuery));

	
	
	//PointCurv+PointCurv Transform
	grasp::data::Item::Ptr item = reduce(list, transformMap[1]);


	//Addd item
	std::string label = "ActiveSense_objectCurv";
	//this->addItem(demo, label, item);
	//this->removeItems(demo, list);
	


}

void pacman::ActiveSense::removeData(pacman::Demo& demo, grasp::Manager::Data::Ptr data)
{
	if (demo.dataMap.size() > 1) {
		grasp::Manager::RenderBlock renderBlock(demo);
		demo.context.write("Removing %s...\n", demo.dataCurrentPtr->first.c_str());
		{
			golem::CriticalSectionWrapper cswData(demo.csData);
			demo.dataMap.erase(demo.dataCurrentPtr++);
			if (demo.dataCurrentPtr == demo.dataMap.end()) demo.dataCurrentPtr = demo.dataMap.begin();
		}
		demo.scene.setOpenGL(grasp::to<grasp::Manager::Data>(demo.dataCurrentPtr)->getView().openGL);
		demo.context.write("%s\n", grasp::Manager::Data::toString(demo.dataCurrentPtr).c_str());
	}
}

grasp::Manager::Data::Ptr pacman::ActiveSense::createData(pacman::Demo& demo)
{
	//Create new Data Bundle
	grasp::Manager::Data::Ptr data;

	demo.readPath("Enter data path to create: ", demo.dataPath);
	if (!isExt(demo.dataPath, demo.dataExt))
		demo.dataPath += demo.dataExt;
	data = demo.createData();
	grasp::Manager::RenderBlock renderBlock(demo);
	demo.scene.getOpenGL(grasp::to<grasp::Manager::Data>(demo.dataCurrentPtr)->getView().openGL); // set current view
	demo.scene.getOpenGL(grasp::to<grasp::Manager::Data>(data)->getView().openGL); // set view of the new data
	{
		golem::CriticalSectionWrapper cswData(demo.csData);
		demo.dataMap.erase(demo.dataPath);
		demo.dataCurrentPtr = demo.dataMap.insert(demo.dataMap.begin(), grasp::Manager::Data::Map::value_type(demo.dataPath, data));
	}
	demo.context.write("Done: Created Data Bundle!\n");

	return data;
}

golem::Mat34 pacman::ActiveSense::computeGoal(pacman::Demo& demo, golem::Mat34& targetFrame, grasp::CameraDepth* camera)
{

	golem::Mat34 frame, invFrame, refPose;
	grasp::ConfigMat34 wrist, invWrist;
	frame.setId(), invFrame.setId(), wrist.w.setId(), invWrist.w.setId(), refPose.setId();

	//Reference frame
	refPose = demo.controller->getChains()[demo.armInfo.getChains().begin()]->getReferencePose();

	//Wrist pose (7th joint)
	camera->getConfig(wrist);
	//Inverse wrist
	invWrist.w.setInverse(wrist.w);

	//It should be the local frame of the camera
	frame = invWrist.w*camera->getFrame();
	invFrame.setInverse(frame);

	Mat34 goal = targetFrame*invFrame*refPose;


	return goal;
}

void pacman::ActiveSense::executeActiveSense(pacman::Demo& demo, const golem::Vec3& centroid, golem::U32 nsamples, const golem::Real& radius)
{


	/*
	//Initialization
	0. Create new Manager::Data and set its reference as dataCurrentPtr
	1. Retrieve current view - ItemImage1 <= Recorder::scanPose
	2. Insert ItemImage1 to dataCurrentPtr->itemMap
	3. Set currentCloud <= ItemImage1
	//While(not finish)
	4. Find points of interest(PoI) using Marek's algorithm on currentCloud => PoI //TODO: Ask Marek about PoI
	5. Generate sensor view hypotheses :
	viewHypotheses <= generateViewHypothesis(PoI, currentCloud) //TODO: Ask Marek about PoI
	6. Select next best view with PoI and currentCloud = > viewPose:
	viewPose <= selectNextBestView(PoI, currentCloud, viewHypotheses) //TODO: Ask Marek about PoI
	7. Retrieve next best view :
	ItemImage02 <= Recorder::scanPose //Added ItemImage02 to dataCurrentPtr->itemMap
	8. Apply Image + Image Transform on dataCurrentPtr->itemMap :
	currentCloud <= AlignTransform(currentCloud, ItemImage02)
	9. Delete ItemImage02 from dataCurrentPtr->itemMap
	19. If It's OK to finish:
	finish <= True
	//endWhile
	*/

	//Getting the camera
	grasp::Sensor::Map::iterator camIt = demo.sensorMap.find("OpenNI+OpenNI");
	if (camIt == demo.sensorMap.end()){
		//demo.context.write("OpenNI+OpenNI is not available\n");
		throw Cancel("OpenNI+OpenNI is not available");
	}
	
	demo.sensorCurrentPtr = camIt;

	grasp::CameraDepth* camera = grasp::is<grasp::CameraDepth>(demo.sensorCurrentPtr);
	if (camera)
	{
		demo.context.write("Camera good!\n");
	}

	//Creating new Data Bundle
	grasp::Manager::Data::Ptr data = createData(demo);


	//Getting view hypotheses (TODO: Compute just one initial view based on current Points of Interest)
	//pacman::HypothesisSensor::Seq::const_iterator hypothesis = this->viewHypotheses.begin();
	size_t index = 0, size = nsamples;

	//Setting up scan command responsible for taking the camera to the selected hyptheses
	std::function<bool()> scanCommand = [&]() -> bool {
		//TODO: Integrate previous measurements into the current point cloud

		demo.context.write("Going to scan pose #%d/%d\n", index + 1, size);
		
		//Random generator. TODO: Add different types of generators
		pacman::HypothesisSensor::Ptr hypothesis = this->generateNextRandomView(centroid, radius);

		{
			golem::CriticalSectionWrapper csw(this->getCSViewHypotheses());
			viewHypotheses.push_back(hypothesis);
		}


		Mat34 goal = this->computeGoal(demo, hypothesis->getFrame(), camera);
		demo.gotoPoseWS(goal);
		//(*hypothesis)->setGLView(demo.scene);

		//TODO: Instead of going through the list, make it compute the next best view using Points of Interest.
		
		return false;
	};

	//Scanning viewHypotheses poses
	
	//demo.// select collision object
	//	CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
	//Performs first scan using current robot pose
	demo.scanPoseActive();
	//TODO: Define stopping criteria
	for (int i = 0; i < size; i++)
	{
		demo.scanPoseActive(scanCommand);
		
		{	
			golem::CriticalSectionWrapper cswData(demo.csData);
			data::Item::Map& itemMap = is<grasp::Manager::Data>(demo.dataCurrentPtr)->itemMap;
			grasp::data::Item::List itemList;
			for (grasp::data::Item::Map::iterator it = itemMap.begin(); it != itemMap.end(); it++)
			{
				if (it->first != ActiveSense::DFT_POINT_CURV_ITEM_LABEL && it->first == ActiveSense::DFT_IMAGE_ITEM_LABEL)
					itemList.push_back(it);
			}

			processItems(demo, itemList);	
		}
		
	}



}

pacman::HypothesisSensor::Ptr ActiveSense::generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius){

	Mat34 sensorPose;

	//Show pose lambda
	typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
	ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
		printf("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};


	golem::Vec3 randomVec;
	golem::U8 r, g, b, a;

	//Generate random color
	r = (U8)rand.nextUniform<float>(50, 255), g = (U8)rand.nextUniform<float>(50, 255), b = (U8)rand.nextUniform<float>(50, 255), a = 255;


	//Generate random orientation vector
	//randomVec.x = rand.nextUniform<float>(-1, 1);
	//randomVec.y = rand.nextUniform<float>(-1, 1);
	//randomVec.z = rand.nextUniform<float>(-1, 1);
	randomVec.next(rand);
	randomVec.setMagnitude(radius);

	//Generate a new pose
	sensorPose = golem::Mat34(golem::Mat34::identity());
	sensorPose.p = centroid + randomVec;

	//Generate sensor orientation
	golem::Vec3 f = -randomVec;
	golem::Vec3 up(0.0, 0.0, 1.0);
	golem::Vec3 n = f.cross(up);
	up = n.cross(f);

	//Make sure this is an orthonormal basis
	f.normalise(); up.normalise(); n.normalise();

	//Up right sensor
	/*sensorPose.R.setColumn(0, up);
	sensorPose.R.setColumn(1, n);
	sensorPose.R.setColumn(2, f);
	*/

	//Upside down sensor (The sensor is sticked upside down on Boris' wrist)

	sensorPose.R.setColumn(0, n);
	sensorPose.R.setColumn(1, -up);
	sensorPose.R.setColumn(2, f);


	//Creating uniformly generated random hypothesis sensor
	pacman::HypothesisSensor::Ptr s(new HypothesisSensor(sensorPose, golem::RGBA(r, g, b, a)));


	return s;

}

void ActiveSense::generateRandomViews(pacman::HypothesisSensor::Seq& viewHypotheses, const golem::Vec3& centroid, const golem::I32& nsamples, const golem::Real& radius)
{
	for (int i = 0; i < nsamples; i++)
	{
		this->viewHypotheses.push_back(this->generateNextRandomView(centroid, radius));
	}
}