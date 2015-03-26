#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Tools/XMLData.h>
#include <pacman/Bham/Demo/Demo.h>
#include <Grasp/Data/Image/Image.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <Grasp/Data/PredictorModel/PredictorModel.h>


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
const std::string ActiveSense::DFT_DATA_PATH = "./data/DFT_DebugData/debugData.xml";
//END


ActiveSense::ActiveSense(pacman::Demo* demoOwner) : ActiveSense()
{
	this->init(demoOwner);
}

void ActiveSense::init(pacman::Demo* demoOwner)
{
	this->demoOwner = demoOwner;
	rand.setRandSeed(this->demoOwner->context.getRandSeed());

	grasp::data::Handler::Map::iterator itImage = demoOwner->handlerMap.find("Image+Image");
	grasp::data::Handler::Map::iterator itPointCurv = demoOwner->handlerMap.find("PointsCurv+PointsCurv");
	grasp::data::Handler::Map::iterator itPredModel = demoOwner->handlerMap.find("PredictorModel+PredictorModel");
	grasp::data::Handler::Map::iterator itPredQuery = demoOwner->handlerMap.find("PredictorQuery+PredictorQuery");
	grasp::data::Transform* transformImage = is<grasp::data::Transform>(itImage);
	grasp::data::Transform* transformPointsCurv = is<grasp::data::Transform>(itPointCurv);
	grasp::data::Transform* transformPredictorModel = is<grasp::data::Transform>(itPredModel);
	grasp::data::Transform* transformPredictorQuery = is<grasp::data::Transform>(itPredQuery);

	if (!transformImage || !transformPointsCurv || !transformPredictorModel || !transformPredictorQuery)
	{
		throw Cancel("We don't have all Transforms we need!");
	}
	else
	{
		demoOwner->context.write("All transforms good!");
	}


	//Currently unused
	transformMap.push_back(std::make_pair(itImage->second.get(), transformImage));

	transformMap.push_back(std::make_pair(itPointCurv->second.get(), transformPointsCurv));

	//Currently unused
	transformMap.push_back(std::make_pair(itPredModel->second.get(), transformPredictorModel));
	//Currently unused
	transformMap.push_back(std::make_pair(itPredQuery->second.get(), transformPredictorQuery));


}

void pacman::ActiveSense::render() const
{
	debugRenderer.render();
}

void pacman::ActiveSense::removeItems(grasp::data::Item::List& list)
{
	for (grasp::data::Item::List::const_iterator i = list.begin(); i != list.end(); ++i) {
		//Removing items
		grasp::UI::removeCallback(*demoOwner, &(*i)->second->getHandler());
		golem::CriticalSectionWrapper cswData(demoOwner->csData);
		demoOwner->dataCurrentPtr->second->itemMap.erase(*i);
	}


	//grasp::UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
	//demoOwner->createRender();
}

grasp::data::Item::Map::iterator ActiveSense::addItem(const std::string& label, grasp::data::Item::Ptr item)
{
	golem::CriticalSectionWrapper cswData(demoOwner->csData);
	grasp::data::Item::Map::iterator ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(label, item));
	Manager::RenderBlock renderBlock(*demoOwner);
	{
		grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());
	}

	return ptr;
}



grasp::data::Item::Map::iterator pacman::ActiveSense::processItems(grasp::data::Item::List& list)
{




	ReduceFunc reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr) -> grasp::data::Item::Map::iterator {

		demoOwner->context.write("\n---List Size: %d---\n", list.size());
		// transform
		Manager::RenderBlock renderBlock(*demoOwner);

		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->csData);
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(ActiveSense::DFT_POINT_CURV_ITEM_LABEL, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		return ptr;
	};

	//Used transforms: PointsCurv+PointsCurv
	//PredictorModel+PredictorModel
	//PredictorQuery+PredictorQuery


	//PointCurv+PointCurv Transform
	grasp::data::Item::Map::iterator itemPtr = reduce(list, transformMap[1]);


	//Adding PointCurv to results
	this->result.pointCurvs.push_back(itemPtr);

	return itemPtr;
}

void pacman::ActiveSense::removeData(grasp::data::Data::Map::iterator data)
{

	demoOwner->setCurrentDataPtr(data);

	if (demoOwner->dataMap.size() > 1) {
		grasp::Manager::RenderBlock renderBlock(*demoOwner);
		demoOwner->context.write("Removing %s...\n", demoOwner->dataCurrentPtr->first.c_str());
		{
			golem::CriticalSectionWrapper cswData(demoOwner->csData);
			demoOwner->dataMap.erase(demoOwner->dataCurrentPtr++);
			if (demoOwner->dataCurrentPtr == demoOwner->dataMap.end()) demoOwner->dataCurrentPtr = demoOwner->dataMap.begin();
		}
		demoOwner->scene.setOpenGL(grasp::to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView().openGL);
		demoOwner->context.write("%s\n", grasp::Manager::Data::toString(demoOwner->dataCurrentPtr).c_str());
	}
}

grasp::Manager::Data::Map::iterator pacman::ActiveSense::createData()
{
	//Create new Data Bundle
	grasp::Manager::Data::Ptr data;

	demoOwner->readPath("Enter data path to create: ", this->dataPath);
	if (!isExt(this->dataPath, demoOwner->dataExt))
		this->dataPath += demoOwner->dataExt;
	data = demoOwner->createData();
	grasp::Manager::RenderBlock renderBlock(*demoOwner);
	demoOwner->scene.getOpenGL(grasp::to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView().openGL); // set current view
	demoOwner->scene.getOpenGL(grasp::to<grasp::Manager::Data>(data)->getView().openGL); // set view of the new data
	{
		golem::CriticalSectionWrapper cswData(demoOwner->csData);
		demoOwner->dataMap.erase(demoOwner->dataPath);
		demoOwner->dataCurrentPtr = demoOwner->dataMap.insert(demoOwner->dataMap.begin(), grasp::Manager::Data::Map::value_type(this->dataPath, data));
	}
	demoOwner->context.write("Done: Created Data Bundle!\n");

	return demoOwner->dataCurrentPtr;
}


golem::Vec3 pacman::ActiveSense::computeCentroid(grasp::data::Item::Map::const_iterator itemImage)
{
	golem::Vec3 centroid(0.0, 0.0, 0.0);


	grasp::data::ItemImage* image = is<grasp::data::ItemImage>(itemImage);

	if (!image)
	{
		throw Cancel("This is not an ItemImage!\n");
	}

	this->demoOwner->context.write("Computing Centroid: %lf %lf %lf CloudSize %d \n", centroid.x, centroid.y, centroid.z, image->cloud->size());

	//Computes centroid ignoring NaNs
	int count = 0;
	for (int i = 0; i < image->cloud->size(); i++)
	{
		// Ignoring NaNs
		if (image->cloud->at(i).x != image->cloud->at(i).x || image->cloud->at(i).y != image->cloud->at(i).y || image->cloud->at(i).z != image->cloud->at(i).z)
			continue;

		centroid.x += image->cloud->at(i).x;
		centroid.y += image->cloud->at(i).y;
		centroid.z += image->cloud->at(i).z;
		count++;

	}
	centroid /= float(count);
	this->demoOwner->context.write("RESULT Centroid ==>>> %lf %lf %lf\n", centroid.x, centroid.y, centroid.z);

	return centroid;
}

grasp::CameraDepth* ActiveSense::getOwnerOPENNICamera()
{

	//Getting the camera
	grasp::Sensor::Map::iterator camIt = demoOwner->sensorMap.find("OpenNI+OpenNI");
	if (camIt == demoOwner->sensorMap.end()){
		//demoOwner->context.write("OpenNI+OpenNI is not available\n");
		throw Cancel("OpenNI+OpenNI is not available");
	}

	demoOwner->sensorCurrentPtr = camIt;

	grasp::CameraDepth* camera = grasp::is<grasp::CameraDepth>(demoOwner->sensorCurrentPtr);

	return camera;
}


golem::Mat34 pacman::ActiveSense::computeGoal(golem::Mat34& targetFrame, grasp::CameraDepth* camera)
{

	golem::Mat34 frame, invFrame, refPose;
	grasp::ConfigMat34 wrist, invWrist;
	frame.setId(), invFrame.setId(), wrist.w.setId(), invWrist.w.setId(), refPose.setId();


	//Reference frame
	refPose = demoOwner->controller->getChains()[demoOwner->armInfo.getChains().begin()]->getReferencePose();

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

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestViewRandom()
{
	golem::Vec3 autoCentroid;

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

	grasp::CameraDepth* camera = this->getOwnerOPENNICamera();
	if (camera)
		demoOwner->context.write("Camera good!\n");



	//Creating new Data Bundle
	//grasp::Manager::Data::Map::iterator data = createData();


	//Getting view hypotheses (TODO: Compute just one initial view based on current Points of Interest)
	//pacman::HypothesisSensor::Seq::const_iterator hypothesis = this->viewHypotheses.begin();
	size_t index = 0, size = this->params.nviews;

	//Setting up scan command responsible for taking the camera to the selected hyptheses
	std::function<bool()> scanCommand = [&]() -> bool {
		//TODO: Integrate previous measurements into the current point cloud

		demoOwner->context.write("Going to scan pose #%d/%d\n", index + 1, size);

		//Random generator. TODO: Add different types of generators
		pacman::HypothesisSensor::Ptr hypothesis = this->generateNextRandomView(this->params.centroid, this->params.radius);

		{
			golem::CriticalSectionWrapper csw(this->getCSViewHypotheses());
			viewHypotheses.push_back(hypothesis);
		}


		Mat34 goal = this->computeGoal(hypothesis->getFrame(), camera);
		demoOwner->gotoPoseWS(goal);
		//(*hypothesis)->setGLView(demoOwner->scene);

		return false;
	};


	//Scanned Items List
	grasp::data::Item::List scannedImageItems;

	//Performs first scan using current robot pose
	demoOwner->scanPoseActive(scannedImageItems);

	if (!this->params.useManualCentroid)
	{
		demoOwner->context.write("Automatically Computing Centroid From First View\n");
		this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
	}

	CollisionBounds::Ptr collisionBounds = demoOwner->selectCollisionBounds();

	//Scanning viewHypotheses poses
	//TODO: Define stopping criteria
	for (int i = 0; i < size; i++)
	{
		demoOwner->scanPoseActive(scannedImageItems, scanCommand);
	}

	return processItems(scannedImageItems);
	/*{
	golem::CriticalSectionWrapper cswData(demoOwner->csData);
	data::Item::Map& itemMap = is<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap;
	grasp::data::Item::List itemList;
	for (grasp::data::Item::Map::iterator it = itemMap.begin(); it != itemMap.end(); it++)
	{
	if (it->first != ActiveSense::DFT_POINT_CURV_ITEM_LABEL && it->first == ActiveSense::DFT_IMAGE_ITEM_LABEL)
	itemList.push_back(it);


	}

	processItems(itemList);
	}*/



}
grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestViewContactBased()
{
	grasp::CameraDepth* camera = this->getOwnerOPENNICamera();
	if (camera)
		demoOwner->context.write("Camera good!\n");

	//Creating new Data Bundle
	//grasp::Manager::Data::Map::iterator data = createData();


	//Getting view hypotheses (TODO: Compute just one initial view based on current Points of Interest)
	//pacman::HypothesisSensor::Seq::const_iterator hypothesis = this->viewHypotheses.begin();
	size_t index = 0, size = this->params.nviews;
	pacman::HypothesisSensor::Ptr hypothesis(nullptr);
	//Setting up scan command responsible for taking the camera to the selected hyptheses
	std::function<bool()> scanCommand = [&]() -> bool {

		demoOwner->context.write("Going to scan pose #%d/%d\n", ++index, size);


		if (hypothesis.get())
		{
			Mat34 goal = this->computeGoal(hypothesis->getFrame(), camera);
			demoOwner->gotoPoseWS(goal);
		}


		return false;
	};


	//Scanned Items List
	grasp::data::Item::List scannedImageItems;

	//Performs first scan using current robot pose
	demoOwner->scanPoseActive(scannedImageItems);

	if (!this->params.useManualCentroid)
	{
		demoOwner->context.write("Automatically Computing Centroid From First View\n");
		this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
	}

	CollisionBounds::Ptr collisionBounds(nullptr);
	this->pointCurvItem = processItems(scannedImageItems);
	this->hasPointCurv = true;

	grasp::data::ItemPredictorModel::Map::iterator ptr;
	for (int i = 0; i < size; i++)
	{
		if (hasPointCurv)
		{

			demoOwner->context.write("FEEDBACK COMPUTATION\n");
			if (!hasPredModel)
			{
				if (hasTrajectory)
					this->setPredModelItem(this->computeTransformPredModel(this->trajectoryItem, this->pointCurvItem));
				else if (hasPredQuery)
					this->setPredModelItem(this->computePredModelFeedBack(this->predQueryItem, this->pointCurvItem));
				else
					throw Cancel("Failed to find one of the following required items: trajectoryItem, predQueryItem");
			}

			ptr = this->computeFeedBackTransform(predModelItem, pointCurvItem);
			//this->setPredModelItem(ptr);

			hypothesis = this->selectNextBestView(ptr);
		}

		if (hypothesis.get())
		{
			Mat34 goal;
			goal.setId();
			//int hi = 0;
			collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
			bool success = false;
			while (hypothesis.get() && !success)
			{
				goal = this->computeGoal(hypothesis->getFrame(), camera);

				try {
					success = demoOwner->gotoPoseWS(goal);

					if (!success)
					{
						demoOwner->context.write("Couldnt got to selected hypothesis, sampling again.\n");
						hypothesis = this->selectNextBestView(ptr);
					}
				}
				catch (const golem::Message& e)
				{
					demoOwner->context.write("Couldnt go to selected hypothesis due exception, sampling again.\n");
					hypothesis = this->selectNextBestView(ptr);
				}
				
				
			
			}
			collisionBounds.release();
		}
		
		

		
		demoOwner->scanPoseActive(scannedImageItems);
		

		//Integrate views into the current pointCurv
		this->pointCurvItem = processItems(scannedImageItems);

	}



	demoOwner->context.write("\nNextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n");
	return this->pointCurvItem;


}

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestView()
{
	if (this->params.method == EMethod::RANDOM)
	{
		return nextBestViewRandom();
	}
	else if (this->params.method == EMethod::CONTACT_BASED)
	{
		return nextBestViewContactBased();
	}
	else
	{
		throw Cancel("This is not a valid method for ActiveSense Next Best View Algorithm");
	}
}
pacman::HypothesisSensor::Ptr ActiveSense::generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius, bool heightBias){

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
	if (!heightBias)
	{
		randomVec.next(rand);
		randomVec.setMagnitude(radius);
	}
	else
	{

		const Real phi = rand.nextUniform<Real>(Math::degToRad<Real>(this->params.minPhi), Math::degToRad<Real>(this->params.maxPhi));
		const Real cos = Math::cos(rand.nextUniform<Real>(Math::degToRad<Real>(this->params.minTheta), Math::degToRad<Real>(this->params.maxTheta)));
		const Real sin = Math::sqrt(numeric_const<Real>::ONE - cos*cos);

		randomVec.set(cos, sin * Math::cos(phi), sin * Math::sin(phi));
		randomVec.setMagnitude(radius);
	}



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


	showPose("Generated SensorPose:", sensorPose);
	//Creating uniformly generated random hypothesis sensor
	pacman::HypothesisSensor::Ptr s(new HypothesisSensor(sensorPose, golem::RGBA(r, g, b, a)));


	return s;

}

pacman::HypothesisSensor::Ptr ActiveSense::selectNextBestView(grasp::data::Item::Map::iterator predModelPtr)
{
	golem::CriticalSectionWrapper csw(this->csViewHypotheses);


	//Random generator. TODO: Add different types of generators
	generateRandomViews(this->params.centroid, this->params.nsamples, this->params.radius);

	int index = 0;

	Real maxValue(-golem::REAL_MAX);
	for (int i = 0; i < this->viewHypotheses.size(); i++)
	{
		ActiveSense::ValueTuple valueTuple = this->computeValue(this->viewHypotheses[i], predModelPtr);

		demoOwner->context.write("\n\nH[%d] Value: %f\n", i, valueTuple.second);

		if (valueTuple.second > maxValue)
		{
			index = i;
			maxValue = valueTuple.second;
		}
	}
	if (index >= this->viewHypotheses.size())
		demoOwner->context.write("There's a problem here! %d", index);
	demoOwner->context.write("\n\n\nBest View Was H[%d] Value: %f\n\n\n", index, maxValue);


	return this->getViewHypothesis(index);
}

void ActiveSense::generateRandomViews(const golem::Vec3& centroid, const golem::I32& nsamples, const golem::Real& radius, bool heightBias)
{
	this->viewHypotheses.clear();
	for (int i = 0; i < nsamples; i++)
	{
		this->viewHypotheses.push_back(this->generateNextRandomView(centroid, radius, heightBias));
	}
}

/**
Executes the following steps:
1) Transforms (predModelItem,pointCurvItem) into predQuery;
2) Converts predQuery into a Trajectory (traj)
3) feedback step => Transforms (pointCurvItem,traj) into a predModelItem

Output: predModelItem result of the feedBack transform
*/

//TODO: Receive a list of Predictor Model
pacman::ActiveSense::ValueTuple pacman::ActiveSense::computeValue(HypothesisSensor::Ptr hypothesis, grasp::data::Item::Map::iterator input)
{
	// training data
	typedef std::vector<const grasp::data::ItemPredictorModel::Data::Map*> TrainingData;
	TrainingData trainingData;

	demoOwner->context.write("getting ItemPredictorModel\n");
	// collect data (TODO: Receive a list of ItemPredictorModel
	const grasp::data::ItemPredictorModel* model = is<const grasp::data::ItemPredictorModel>(input);
	if (model)
	{
		trainingData.push_back(&model->modelMap);
	}
	else
	{
		demoOwner->context.write("This is not an ItemPredictorModel\n");
	}




	//Theta function returns the angle between the sensor image plane and the contact surface normal
	std::function<golem::Real(const golem::Mat34&, const golem::Mat34&)> thetaFunc = [](const golem::Mat34& sensorFrame, const golem::Mat34& contactFrame) -> golem::Real
	{
		golem::Vec3 v(0.0, 0.0, 0.0);
		sensorFrame.R.getColumn(3, v); //viewDir of hypothesis sensor

		golem::Vec3 n(0.0, 0.0, 0.0);
		contactFrame.R.getColumn(3, n); //Surface Normal vector of contact point
		n.normalise();
		v.normalise();

		//Angle between two planes (angle between the normal vectors of two planes)
		//Theta is minimum when the image plane is perpendicular to the surface plane the sensor is looking at.
		Real theta = golem::Math::acos((-n).dot(v));

		return theta;
	};

	//Sigma function is an heuristics that tells that good view points are those with lowest theta
	std::function<golem::Real(const Real&)> sigmaFunc = [](const Real& theta) -> golem::Real
	{
		Real c1(-0.5), c2(-0.5), c3(10);

		return c1*theta*theta + c2*theta + c3;
	};


	//Value function for a single joint
	std::function<golem::Real(const grasp::Contact3D::Seq&, const golem::Mat34&)> valFunc = [&](const grasp::Contact3D::Seq& graspContacts, const golem::Mat34& sensorFrame) -> golem::Real
	{
		Real value(0.0);
		grasp::Contact3D last = graspContacts.back();


		int maxK = 2, k = 0;

		Real cdf(0.0);
		for (grasp::Contact3D::Seq::const_iterator it = graspContacts.begin(); it != graspContacts.end(); it++)
		{
			golem::Mat34 frame;
			frame.p = it->point;
			frame.R.fromQuat(it->orientation);



			//Just renders a few axes
			if (++k < maxK) debugRenderer.addAxes3D(frame, golem::Vec3(0.01));



			Real theta = thetaFunc(sensorFrame, frame);
			Real sigma = sigmaFunc(theta);
			Real weight = it->weight;
			cdf += weight;
			value += weight*sigma;
			//demoOwner->context.write("Theta %lf Weight %lf Sigma %f CDF %lf LastCDF %lf \n", theta, weight, sigma, it->cdf, last.cdf);

		}
		value /= cdf;

		return value;
	};

	golem::Real maxValue(-golem::REAL_MAX), currValue(0.0);
	std::string maxType = "None";
	//demoOwner->context.write("Computing Value\n");
	for (TrainingData::const_iterator i = trainingData.begin(); i != trainingData.end(); ++i){
		//For each graspType's mapping of joint->contacts do...
		for (grasp::data::ItemPredictorModel::Data::Map::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j) {

			//j->first => GraspType
			//j->second.contacts => Map between joints x contact3D 
			//where contact3D is contains (point,orientation,frame,weight)

				{
					golem::CriticalSectionWrapper csw(csRenderer);
					debugRenderer.reset();
				}

			currValue = golem::Real(0.0);
			//For each joint's sequence of contact3D regardless of the joint associated with it, do...
			for (grasp::Contact3D::Map::const_iterator k = j->second.contacts.begin(); k != j->second.contacts.end(); k++)
			{
				// k->first => jointToString()
				//demoOwner->context.write("---JOINT %s---\n", k->first);
				//Adds individual contributions of each joint

				currValue += valFunc(k->second, hypothesis->getFrame());
				//demoOwner->context.write("currValue %f CurrentMax %f\n", currValue, maxValue);
			}

			if (currValue >= maxValue)
			{
				maxType = j->first;
				maxValue = currValue;
			}

		}
	}


	return std::make_pair(maxType, maxValue);
}

grasp::CollisionBounds::Ptr pacman::ActiveSense::selectCollisionBounds(bool draw, grasp::data::Item::Map::const_iterator input) {
	CollisionBounds::Ptr collisionBounds;

	// select collision object
	demoOwner->context.write("ActiveSense: Select collision object...\n");

	const data::Location3D* location = is<const data::Location3D>(input->second.get());

	if (location) {
		// create collision bounds
		collisionBounds.reset(new CollisionBounds(*demoOwner->planner, [=](size_t i, Vec3& p) -> bool {

			if (i < location->getNumOfLocations()) p = location->getLocation(i); return i < location->getNumOfLocations();
		}, draw ? &demoOwner->objectRenderer : nullptr, draw ? &demoOwner->csRenderer : nullptr));
		// draw locations
		golem::CriticalSectionWrapper csw(demoOwner->csRenderer);
		for (size_t i = 0; i < location->getNumOfLocations(); ++i)
			demoOwner->objectRenderer.addPoint(location->getLocation(i), golem::RGBA::BLACK);
	}
	else
		demoOwner->context.write("Object collisions unsupported\n");


	return collisionBounds;
}

grasp::data::Item::Map::iterator ActiveSense::convertToTrajectory(grasp::data::Item::Map::iterator predQueryModel)
{

	grasp::data::Convert* convert = grasp::is<grasp::data::Convert>(predQueryModel);
	if (!convert)
		throw Message(Message::LEVEL_ERROR, "Input item does not support Convert interface");

	// find matching handlers
	typedef std::vector<data::Handler*> HandlerSet;
	HandlerSet handlerSet;
	for (data::Handler::Map::const_iterator i = demoOwner->handlerMap.begin(); i != demoOwner->handlerMap.end(); ++i)
		if (std::find(convert->getHandlerTypes().begin(), convert->getHandlerTypes().end(), i->second->getType()) != convert->getHandlerTypes().end())
			handlerSet.push_back(i->second.get());


	// pick up handler
	data::Handler::Map::iterator handlerPtr = demoOwner->handlerMap.find("Trajectory+Trajectory");

	// convert
	data::Item::Ptr traj = convert->convert(*handlerPtr->second);
	grasp::data::Item::Map::iterator output = addItem("handle", traj);

	//Adding Trajectory to results
	this->result.trajectories.push_back(output);
	this->setTrajectoryItem(output);

	return output;

}
grasp::data::Item::Map::iterator ActiveSense::computeFeedBackTransform(grasp::data::Item::Map::iterator predModelItem, grasp::data::Item::Map::iterator pointCurvItem)
{
	std::function<grasp::data::Item::Map::iterator(grasp::data::Item::List&, TransformMap::value_type&, const std::string&)> reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr, const std::string& itemLabel) -> grasp::data::Item::Map::iterator {

		demoOwner->context.write("\n---[computeFeedBackTransform]: List Size: %d---\n", list.size());
		// transform
		Manager::RenderBlock renderBlock(*demoOwner);

		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->csData);
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		return ptr;
	};

	grasp::data::Item::List list;
	list.push_back(predModelItem);
	list.push_back(pointCurvItem);

	demoOwner->context.write("Computing FeedBackQuery!\n");
	//Transforms predModel and pointCurv into a predictorQuery item
	grasp::data::Item::Map::iterator predQuery = reduce(list, transformMap[3], "FeedBack_predQuery");
	this->result.predQueries.push_back(predQuery);

	//Setting current PredQuery
	this->setPredQueryItem(predQuery);

	grasp::data::Item::Map::iterator predModelNew = computePredModelFeedBack(predQuery, pointCurvItem);

	//list.push_back(predModelItem);
	//this->removeItems(list);


	return predModelNew;

}

grasp::data::Item::Map::iterator pacman::ActiveSense::computePredModelFeedBack(grasp::data::Item::Map::iterator predQuery, grasp::data::Item::Map::iterator pointCurvItem)
{

	demoOwner->context.write("Computing Trajectory!\n");
	grasp::data::Item::Map::iterator traj = convertToTrajectory(predQuery);


	return computeTransformPredModel(pointCurvItem, traj);
}

grasp::data::Item::Map::iterator pacman::ActiveSense::computeTransformPredModel(grasp::data::Item::Map::iterator trajItem, grasp::data::Item::Map::iterator pointCurvItem)
{
	std::function<grasp::data::Item::Map::iterator(grasp::data::Item::List&, TransformMap::value_type&, const std::string&)> reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr, const std::string& itemLabel) -> grasp::data::Item::Map::iterator {

		demoOwner->context.write("\n---List Size: %d---\n", list.size());
		// transform
		Manager::RenderBlock renderBlock(*demoOwner);

		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->csData);
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		return ptr;
	};



	grasp::data::Item::List list;

	list.clear();
	//Feedback
	list.push_back(pointCurvItem);
	list.push_back(trajItem);


	demoOwner->context.write("Computing predModelNew!");

	grasp::data::Item::Map::iterator predModelNew = reduce(list, transformMap[2], "predModelNew");


	return predModelNew;
}

//--------------------------------------------------------------------------------
void pacman::ActiveSenseController::initActiveSense(pacman::Demo* demoOwner)
{
	this->activeSense.init(demoOwner);


	Mat34 sensorPose;

	/*Up right sensor
	sensorPose.R.setColumn(0,golem::Vec3(0.0, 0.0, 1.0));
	sensorPose.R.setColumn(1,golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(2,golem::Vec3(0.0, 1.0, 0.0));
	*/

	//Upside down sensor
	sensorPose.R.setColumn(0, golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(1, golem::Vec3(0.0, 0.0, -1.0));
	sensorPose.R.setColumn(2, golem::Vec3(0.0, 1.0, 0.0));

	sensorPose.p.set(0.5, -0.5, 0.025);

	//Example sensor hypothesis is going to be used just as an object in the scene
	dummyObject = pacman::HypothesisSensor::Ptr(new HypothesisSensor(sensorPose));

	//Generating sensor hypotheses around dummyObject's center
	// this->activeSense.generateRandomViews(this->activeSense.getViewHypotheses(), dummyObject->getPose().p, 2, 0.15);
}
