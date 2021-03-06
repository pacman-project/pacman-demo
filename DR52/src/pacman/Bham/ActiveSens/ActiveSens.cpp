#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Tools/XMLData.h>
#include <pacman/Bham/ActiveSensDemo/ActiveSensDemo.h>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/ContactModel/ContactModel.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>

using namespace pacman;
using namespace golem;
using namespace grasp;


//------------------------------------------------------------------------------

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
		demoOwner->context.debug("ActiveSense: All transforms good!\n");
	}


	//Currently unused (Generates unorganized pointclouds)
	transformMap.push_back(std::make_pair(itImage->second.get(), transformImage));

	transformMap.push_back(std::make_pair(itPointCurv->second.get(), transformPointsCurv));

	transformMap.push_back(std::make_pair(itPredModel->second.get(), transformPredictorModel));

	transformMap.push_back(std::make_pair(itPredQuery->second.get(), transformPredictorQuery));


	this->generateViews();

	this->allowInput = false;
}

/** Load from xml context */
void ActiveSense::Parameters::load(const golem::XMLContext* xmlcontext)
{
	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("active_sense parameters");


	std::string selectionMethodStr, alternativeSelectionMethodStr, generationMethodStr, coverageMethodStr, stoppingCriteriaStr;

	XMLData("sensor_id", this->sensorId, pxmlcontext);
	XMLData("selection_method", selectionMethodStr, pxmlcontext);
	XMLData("alternative_selection_method", alternativeSelectionMethodStr, pxmlcontext);
	XMLData("generation_method", generationMethodStr, pxmlcontext);
	XMLData("enable_regeneration", this->regenerateViews, pxmlcontext);
	XMLData("use_manual_centroid", this->useManualCentroid, pxmlcontext);
	XMLData("use_height_bias", this->useHeightBias, pxmlcontext);
	XMLData("radius", this->radius, pxmlcontext);
	XMLData("nsamples", this->nsamples, pxmlcontext);
	XMLData("nviews", this->nviews, pxmlcontext);
	XMLData("coverage_threshold", this->coverageThr, pxmlcontext);
	XMLData("coverage_method", coverageMethodStr, pxmlcontext);
	XMLData("stopping_criteria", stoppingCriteriaStr, pxmlcontext);
	XMLData("enable_coverage_filter_plane", this->filterPlane, pxmlcontext);
	XMLData("min_phi", this->minPhi, pxmlcontext);
	XMLData("max_phi", this->maxPhi, pxmlcontext);
	XMLData("min_theta", this->minTheta, pxmlcontext);
	XMLData("max_theta", this->maxTheta, pxmlcontext);

	XMLData(this->centroid, pxmlcontext->getContextFirst("centroid"), false);

	if (!coverageMethodStr.compare("area_based"))
		this->coverageMethod = ECoverageMethod::M_AREA_BASED;
	else if (!coverageMethodStr.compare("volume_based"))
		this->coverageMethod = ECoverageMethod::M_VOLUME_BASED;
	else
		this->coverageMethod = ECoverageMethod::M_NONE;

	if (!stoppingCriteriaStr.compare("number_of_views"))
		this->stoppingCriteria = EStoppingCriteria::C_NVIEWS;
	else if (!stoppingCriteriaStr.compare("coverage"))
		this->stoppingCriteria = EStoppingCriteria::C_COVERAGE;
	else if (!stoppingCriteriaStr.compare("number_of_views_and_coverage"))
		this->stoppingCriteria = EStoppingCriteria::C_NVIEWS_COVERAGE;
	else
		this->stoppingCriteria = EStoppingCriteria::C_NONE;

	if (!selectionMethodStr.compare("contact_based"))
		this->selectionMethod = ESelectionMethod::S_CONTACT_BASED;
	else if (!selectionMethodStr.compare("contact_based_v2"))
		this->selectionMethod = ESelectionMethod::S_CONTACT_BASED2;
	else if (!selectionMethodStr.compare("random"))
		this->selectionMethod = ESelectionMethod::S_RANDOM;
	else if (!selectionMethodStr.compare("sequential"))
		this->selectionMethod = ESelectionMethod::S_SEQUENTIAL;
	else
		this->selectionMethod = ESelectionMethod::S_NONE;

	if (!alternativeSelectionMethodStr.compare("contact_based"))
		this->alternativeSelectionMethod = ESelectionMethod::S_CONTACT_BASED;
	else if (!alternativeSelectionMethodStr.compare("contact_based_v2"))
		this->alternativeSelectionMethod = ESelectionMethod::S_CONTACT_BASED2;
	else if (!alternativeSelectionMethodStr.compare("random"))
		this->alternativeSelectionMethod = ESelectionMethod::S_RANDOM;
	else if (!alternativeSelectionMethodStr.compare("sequential"))
		this->alternativeSelectionMethod = ESelectionMethod::S_SEQUENTIAL;
	else
		this->alternativeSelectionMethod = ESelectionMethod::S_NONE;

	if (!generationMethodStr.compare("random_sphere"))
		this->generationMethod = EGenerationMethod::G_RANDOM_SPHERE;
	else if (!generationMethodStr.compare("fixed"))
		this->generationMethod = EGenerationMethod::G_FIXED;
	else
		this->generationMethod = EGenerationMethod::G_NONE;

	dropOffPose.xmlData(pxmlcontext->getContextFirst("drop_off_pose"));

	if (this->generationMethod == EGenerationMethod::G_FIXED)
	{
		configSeq.clear();
		XMLData(configSeq, configSeq.max_size(), const_cast<golem::XMLContext*>(pxmlcontext), "pose");

		printf("Loaded %d fixed poses:\n", configSeq.size());

		for (int i = 0; i < configSeq.size(); i++)
		{
			HypothesisSensor::Config config;
			config = configSeq[i];
			printf("pose%3d: ", i+1);
			const size_t n = config.c.size();
			size_t n0 = n;
			for (size_t j = n - 1; j >= 0; --j)
			{
				if (config.c[j] != 0)
				{
					n0 = j+1;
					break;
				}
			}
			for (size_t j = 0; j < n0; ++j)
				printf("%f ", static_cast<float>(config.c[j]));
			printf("\n");
		}
	}

	printf("ActiveSense: method %d usmc %d ushb %d radius %f nsamples %d nviews %d\n", this->selectionMethod, this->useManualCentroid, this->useHeightBias, this->radius, this->nsamples, this->nviews);
	printf("ActiveSense: minPhi %f maxPhi %f minTheta %f maxTheta %f!\n", this->minPhi, this->maxPhi, this->minTheta, this->maxTheta);
	printf("ActiveSense: centroid %f %f %f\n", this->centroid.v1, this->centroid.v2, this->centroid.v3);
}

/** Load from xml context */
void ActiveSense::load(const golem::XMLContext* xmlcontext)
{
	this->params.load(xmlcontext);

}

/** Load from xml context */
void ActiveSenseController::load(const golem::XMLContext* xmlcontext)
{
	this->activeSense->load(xmlcontext);

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
		golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
		demoOwner->dataCurrentPtr->second->itemMap.erase(*i);
	}


	//grasp::UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
	//demoOwner->createRender();
}

grasp::data::Item::Map::iterator ActiveSense::addItem(const std::string& label, grasp::data::Item::Ptr item)
{
	Manager::RenderBlock renderBlock(*demoOwner);
	golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
	grasp::data::Item::Map::iterator ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(label, item));
	
	{
		grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());
	}

	return ptr;
}



grasp::data::Item::Map::iterator pacman::ActiveSense::processItems(grasp::data::Item::List& list)
{

	ReduceFunc reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr) -> grasp::data::Item::Map::iterator {

		demoOwner->context.debug("\nActiveSense: [PointCurv+PointCurv]---List Size: %d---\n", list.size());
		// transform
		Manager::RenderBlock renderBlock(*demoOwner);

		
		golem::shared_ptr<grasp::Manager::InputBlock> inputBlock;
		if (!this->allowInput) inputBlock = golem::shared_ptr<grasp::Manager::InputBlock>(new grasp::Manager::InputBlock(*demoOwner));
		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(ActiveSense::DFT_POINT_CURV_ITEM_LABEL, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		inputBlock.release();
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
		demoOwner->context.debug("ActiveSense: Removing %s...\n", demoOwner->dataCurrentPtr->first.c_str());
		{
			golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
			demoOwner->dataMap.erase(demoOwner->dataCurrentPtr++);
			if (demoOwner->dataCurrentPtr == demoOwner->dataMap.end()) demoOwner->dataCurrentPtr = demoOwner->dataMap.begin();
		}
		demoOwner->scene.setOpenGL(grasp::to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView().openGL);
		demoOwner->context.debug("ActiveSense: %s\n", grasp::Manager::Data::toString(demoOwner->dataCurrentPtr).c_str());
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
		golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
		demoOwner->dataMap.erase(demoOwner->dataPath);
		demoOwner->dataCurrentPtr = demoOwner->dataMap.insert(demoOwner->dataMap.begin(), grasp::Manager::Data::Map::value_type(this->dataPath, data));
	}
	demoOwner->context.debug("ActiveSense: Done: Created Data Bundle!\n");

	return demoOwner->dataCurrentPtr;
}

golem::Vec3 pacman::ActiveSense::computeCentroid(CloudT::Ptr cloudIn)
{
	
	golem::Vec3 centroid(0.0, 0.0, 0.0);
	this->demoOwner->context.debug("ActiveSense: Computing Centroid: %lf %lf %lf CloudSize %d \n", centroid.x, centroid.y, centroid.z, cloudIn->size());
	//Computes centroid ignoring NaNs
	int count = 0;
	for (int i = 0; i < cloudIn->size(); i++)
	{
		// Ignoring NaNs
		if (cloudIn->at(i).x != cloudIn->at(i).x || cloudIn->at(i).y != cloudIn->at(i).y || cloudIn->at(i).z != cloudIn->at(i).z)
			continue;

		centroid.x += cloudIn->at(i).x;
		centroid.y += cloudIn->at(i).y;
		centroid.z += cloudIn->at(i).z;

		count++;

	}
	centroid /= float(count);
	this->demoOwner->context.debug("ActiveSense: RESULT Centroid ==>>> %lf %lf %lf\n", centroid.x, centroid.y, centroid.z);

	return centroid;


}

golem::Vec3 pacman::ActiveSense::computeCentroid2(grasp::Cloud::PointCurvSeq::Ptr cloudIn)
{

	golem::Vec3 centroid(0.0, 0.0, 0.0);
	this->demoOwner->context.debug("ActiveSense: Computing Centroid: %lf %lf %lf CloudSize %d \n", centroid.x, centroid.y, centroid.z, cloudIn->size());
	//Computes centroid ignoring NaNs
	int count = 0;
	for (int i = 0; i < cloudIn->size(); i++)
	{
		// Ignoring NaNs
		if (cloudIn->at(i).x != cloudIn->at(i).x || cloudIn->at(i).y != cloudIn->at(i).y || cloudIn->at(i).z != cloudIn->at(i).z)
			continue;

		centroid.x += cloudIn->at(i).x;
		centroid.y += cloudIn->at(i).y;
		centroid.z += cloudIn->at(i).z;

		count++;

	}
	centroid /= float(count);
	this->demoOwner->context.debug("ActiveSense: RESULT Centroid ==>>> %lf %lf %lf\n", centroid.x, centroid.y, centroid.z);

	return centroid;


}
golem::Vec3 pacman::ActiveSense::computeCentroid(grasp::data::Item::Map::const_iterator itemImage)
{
	golem::Vec3 centroid(0.0, 0.0, 0.0);


	grasp::data::ItemImage* image = is<grasp::data::ItemImage>(itemImage);

	if (!image)
	{
		throw Cancel("ActiveSense: This is not an ItemImage!\n");
	}

	centroid = computeCentroid(image->cloud);
	
	return centroid;
}

grasp::CameraDepth* ActiveSense::getOwnerOPENNICamera()
{
	return grasp::is<grasp::CameraDepth>(getOwnerSensor("OpenNI+OpenNI"));
}

grasp::Camera* pacman::ActiveSense::getOwnerSensor(const std::string& sensorId)
{

	//Getting the camera


	grasp::Sensor::Map::iterator camIt = demoOwner->sensorMap.find(sensorId);
	if (camIt == demoOwner->sensorMap.end()){
		demoOwner->context.debug("%s is not available\n", sensorId.c_str());
		throw Cancel("ActiveSense: sensor is not available");
	}

	demoOwner->sensorCurrentPtr = camIt;

	grasp::Camera* camera = grasp::is<grasp::Camera>(demoOwner->sensorCurrentPtr);

	return camera;
}


golem::Mat34 pacman::ActiveSense::computeGoal(golem::Mat34& targetFrame, grasp::Camera* camera)
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

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestView()
{
	this->visitedHypotheses.clear();
	this->result.pointCurvs.clear();
	this->result.predQueries.clear();
	this->result.trajectories.clear();

	for (int i = 0; i < this->viewHypotheses.size(); i++)
		this->viewHypotheses[i]->visited = false;

	grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

	if (camera)
		demoOwner->context.debug("ActiveSense: Camera good!\n");

	//Creating new Data Bundle
	//grasp::Manager::Data::Map::iterator data = createData();

	size_t index = 0, maxNumViews = this->params.nviews;
	pacman::HypothesisSensor::Ptr hypothesis;

	grasp::data::Item::List scannedImageItems; // scanned Items List

	//Adding current sensor pose to the sequence of visited hypotheses
	const size_t jointsSize = (size_t)demoOwner->info.getJoints().size();
	HypothesisSensor::Config config(RealSeq(jointsSize, 0.0));
	golem::Controller::State begin = demoOwner->controller->createState();// demoOwner->lookupState();
	demoOwner->controller->lookupState(golem::SEC_TM_REAL_MAX, begin);

	begin.cpos.get(config.c.data(), config.c.data() + std::min(config.c.size(), jointsSize));
	hypothesis = this->generateViewFrom(config);
	hypothesis->visited = true;
	visitedHypotheses.push_back(hypothesis);

	//Carry on as if nothing happened
	hypothesis = pacman::HypothesisSensor::Ptr();
	
	// performs first scan using current wrist pose
	demoOwner->scanPoseActive(scannedImageItems); 

	if (!this->params.useManualCentroid)
	{
		demoOwner->context.debug("ActiveSense: Automatically Computing Centroid From First View\n");
		this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
	}

	CollisionBounds::Ptr collisionBounds;
	this->pointCurvItem = processItems(scannedImageItems);
	this->hasPointCurv = true;

	demoOwner->context.debug("ActiveSense: FEEDBACK COMPUTATION\n");
	
	//If we dont have a predictor model we generate one using either a predQuery or a trajectory
	if (!hasPredModel && (this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED || this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2))
	{
	    if (hasPredQuery)
			 this->setPredModelItem(this->computePredModelFeedBack(this->predQueryItem, this->pointCurvItem));
		else if (hasTrajectory)
			this->setPredModelItem(this->computeTransformPredModel(this->trajectoryItem, this->pointCurvItem));
		else
			throw Cancel("ActiveSense: Failed to find one of the following required items: predQueryItem, trajectoryItem");
	}

	grasp::data::ItemContactModel::Map::iterator ptr;
	bool stop = maxNumViews > 1 ? false : true; // we could stop after the initial view
	bool found_contacts = true; //lets assume we have contacts
	int numViewsAcquired = 1; // we count the initial view
	while (!stop)
	{
		demoOwner->context.debug(
			"\n\nActiveSense: ************************ STARTING OBSERVATION #%d ************************\n\n",
			numViewsAcquired+1);

		if (params.selectionMethod == ESelectionMethod::S_CONTACT_BASED ||
			params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2)
		{
			try{

				ptr = computeFeedBackTransform(predModelItem, pointCurvItem);
				found_contacts = true; // ok, we indeed have contacts
			}
			catch (...){
				demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
				found_contacts = false; // we actually don't have contacts...
			}
			
		}

		Mat34 goal;
		goal.setId();
		collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
		bool gotToHypothesis = false;
		while (!gotToHypothesis)
		{

			hypothesis = selectNextBestView(ptr, found_contacts);
			

			goal = this->computeGoal(hypothesis->getFrame(), camera);

			try
			{
				if (this->params.generationMethod == EGenerationMethod::G_RANDOM_SPHERE)
					gotToHypothesis = demoOwner->gotoPoseWS(goal);
				if (this->params.generationMethod == EGenerationMethod::G_FIXED)
					gotToHypothesis = demoOwner->gotoPoseConfig(hypothesis->getConfig());

				if (!gotToHypothesis)
					demoOwner->context.debug("ActiveSense: Couldnt go to selected hypothesis due to large final pose error, trying next NBV.\n");
			}
			catch (const golem::Message& e)
			{
				demoOwner->context.debug("ActiveSense: Couldnt go to selected hypothesis due exception %s, trying next NBV.\n", e.what());
			}
		}
		collisionBounds.release();

		//Adding hypotheses to the set of visited hypotheses so far (used for not going to this hypothesis again later)
		hypothesis->visited = true;
		visitedHypotheses.push_back(hypothesis);

		demoOwner->scanPoseActive(scannedImageItems);

		//Integrate views into the current pointCurv
		this->pointCurvItem = processItems(scannedImageItems);

		//Number of views increment
		++numViewsAcquired;

		//Checking stopping criteria
		if (params.stoppingCriteria == EStoppingCriteria::C_NVIEWS)
		{
			if (numViewsAcquired >= maxNumViews)
			{
				stop = true;
				demoOwner->context.debug("ActiveSense: Stopping because used max number of views\n");
			}
		}	
		else if (params.stoppingCriteria == EStoppingCriteria::C_COVERAGE || params.stoppingCriteria == EStoppingCriteria::C_NVIEWS_COVERAGE)
		{
			std::vector< pcl::Vertices > polygons;
			grasp::data::ItemPointsCurv* image = grasp::is<grasp::data::ItemPointsCurv>(this->pointCurvItem);
			golem::Real coverage = 0.0;
			if (image)
				coverage = computeCoverage(image->cloud, polygons);
			else
				demoOwner->context.debug("ActiveSense: This is not an ItemImage.\n");
			
			if (coverage >= params.coverageThr)
			{
				stop = true;
				demoOwner->context.debug("ActiveSense: Stopping because exceeded coverage threshold\n");
			}
			else if (params.stoppingCriteria == EStoppingCriteria::C_NVIEWS_COVERAGE && numViewsAcquired >= maxNumViews)
			{
				stop = true;
				demoOwner->context.debug("ActiveSense: Stopping because used max number of views\n");
			}
		}
		else
		{
			demoOwner->context.debug("ActiveSense: Unknown stopping criteria, stopping now.\n");
			stop = true;
		}
	} // loop over views

	demoOwner->context.debug("\n\nActiveSense: ************************ COMPLETED AFTER %d OBSERVATION%s ************************\n\n",
		numViewsAcquired, numViewsAcquired==1 ? "" : "S" );

	//If we are using a selection method different from contact_based then we generate a queryModel, trajectory and a disposable (feedback)predictorModel as a final result
	if (maxNumViews == 1 || (this->params.selectionMethod != ESelectionMethod::S_CONTACT_BASED && this->params.selectionMethod != ESelectionMethod::S_CONTACT_BASED2))
	{
		ptr = this->computeFeedBackTransform(predModelItem, pointCurvItem);
	}

	demoOwner->context.debug("ActiveSense: NextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n", result.pointCurvs.size(), result.predQueries.size(), result.trajectories.size());

	return this->pointCurvItem;
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestView(grasp::data::Item::Map::iterator predModelPtr, bool found_contacts)
{
	HypothesisSensor::Ptr hypothesis;

	if ((params.selectionMethod == ESelectionMethod::S_CONTACT_BASED || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2) && !found_contacts)
	{
		hypothesis = selectNextBestView(params.alternativeSelectionMethod, predModelPtr);
	}
	else
	{
		hypothesis = selectNextBestView(params.selectionMethod, predModelPtr);
	}

	return hypothesis;
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestView(int selectionMethod, grasp::data::Item::Map::iterator predModelPtr)
{
	HypothesisSensor::Ptr hypothesis;

	if (selectionMethod == ESelectionMethod::S_CONTACT_BASED)
	{
		hypothesis = selectNextBestViewContactBased(predModelPtr);
	}
	else if (selectionMethod == ESelectionMethod::S_CONTACT_BASED2)
	{
		hypothesis = selectNextBestViewContactBased2(predModelPtr);
	}
	else if (selectionMethod == ESelectionMethod::S_RANDOM)
	{
		hypothesis = selectNextBestViewRandom();
	}
	else if (selectionMethod == ESelectionMethod::S_SEQUENTIAL)
	{
		hypothesis = selectNextBestViewSequential();
	}
	else
		throw Cancel("pacman::ActiveSense::selectNextBestView: unknown selectionMethod");

	if (hypothesis == nullptr)
		throw Cancel("ActiveSense::selectNextBestView: unable to get hypothesis for next best view");

	return hypothesis;
}


pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewRandom()
{
	golem::CriticalSectionWrapper csw(this->csViewHypotheses);
	
	if (this->params.regenerateViews) this->generateViews();

	golem::U32 index = 0;
	do{
		index = (golem::U32)(rand.nextUniform<float>()*this->viewHypotheses.size());

	} while (this->getViewHypothesis(index)->visited || hasViewed(this->getViewHypothesis(index)));


	return this->getViewHypothesis(index);
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewSequential()
{
	golem::CriticalSectionWrapper csw(this->csViewHypotheses);
	
	if (this->params.regenerateViews) this->generateViews();

	pacman::HypothesisSensor::Ptr ret = this->getViewHypothesis(this->seqIndex);

	this->seqIndex = (this->seqIndex + 1) % this->viewHypotheses.size();

	demoOwner->context.debug("\nActiveSense: Sequential View #%d\n\n", seqIndex);

	return ret;
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewContactBased(grasp::data::Item::Map::iterator predModelPtr)
{
	golem::CriticalSectionWrapper csw(this->csViewHypotheses);

	if (this->params.regenerateViews) this->generateViews();

	int index = 0;

	Real maxValue(-golem::REAL_MAX);
	for (int i = 0; i < this->viewHypotheses.size(); i++)
	{
		if (this->viewHypotheses[i]->visited)
		{
			demoOwner->context.debug("ActiveSense: H[%d] Value: ALREADY VISITED\n", i + 1);
			continue;
		}
		if (hasViewed(this->viewHypotheses[i]))
		{
			demoOwner->context.debug("ActiveSense: H[%d] Value: ALREADY VISITED WITHIN 5CM\n", i + 1);
			continue;
		}

		ActiveSense::ValueTuple valueTuple = this->computeValue(this->viewHypotheses[i], predModelPtr);

		demoOwner->context.debug("ActiveSense: H[%d] Value: %f\n", i+1, valueTuple.second);

		if (valueTuple.second > maxValue)
		{
			index = i;
			maxValue = valueTuple.second;
		}
	}

	demoOwner->context.debug("\nActiveSense: Best View Was H[%d] Value: %f\n\n", index+1, maxValue);

	return this->getViewHypothesis(index);
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewContactBased2(grasp::data::Item::Map::iterator predModelPtr)
{
	golem::CriticalSectionWrapper csw(this->csViewHypotheses);



	if (this->params.regenerateViews) this->generateViews();

	onlineModel.setTrainingData(predModelPtr);
	onlineModel.updateContacts();
	onlineModel.updateViewingAngles(visitedHypotheses);

	int index = 0;

	Real maxValue(-golem::REAL_MAX);
	Real value(0.0);
	demoOwner->context.debug("ActiveSense: Next Best View Contact Based V2\n");
	for (int i = 0; i < this->viewHypotheses.size(); i++)
	{
		if (this->viewHypotheses[i]->visited || hasViewed(this->viewHypotheses[i]))
			continue;

		value = onlineModel.computeValue(this->viewHypotheses[i]);

		demoOwner->context.debug("ActiveSense: H[%d] Value: %f\n", i+1, value);

		if (value > maxValue)
		{
			index = i;
			maxValue = value;
		}
	}

	demoOwner->context.debug("\nActiveSense: Best View Was H[%d] Value: %f\n\n", index+1, maxValue);


	return this->getViewHypothesis(index);
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::generateViewFrom(const HypothesisSensor::Config& config)
{
	HypothesisSensor::Config sensorConfig = config;

	//Show pose lambda
	typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
	ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
		demoOwner->context.debug("ActiveSense: %s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};


	golem::U8 r, g, b, a;
	//Generate random color
	r = 125, g = 122, b = 200, a = 32;

	golem::Controller::State state = demoOwner->controller->createState();
	state.cpos.set(config.c.data(), config.c.data() + std::min(config.c.size(), (size_t)demoOwner->info.getJoints().size()));

	// forward transform
	golem::WorkspaceJointCoord wjc;
	demoOwner->controller->jointForwardTransform(state.cpos, wjc);

	grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

	golem::Mat34 offsetPose;
	offsetPose.setId();
	offsetPose.p.z = -0.05;


	//Workspace pose is just for drawing purposes when generation_method="fixed"
	sensorConfig.w = wjc[demoOwner->info.getJoints().begin() + camera->getConfigJoint() - 1] * camera->getCurrentCalibration()->getParameters().pose * offsetPose;



	showPose("sensor pose", sensorConfig.w);

	pacman::HypothesisSensor::Ptr s(new HypothesisSensor(sensorConfig, golem::RGBA(r, g, b, a)));

	return s;

}
void pacman::ActiveSense::generateViewsFromSeq(const HypothesisSensor::Config::Seq& configSeq)
{
	demoOwner->context.debug("Generating %d sensor poses from fixed sequence...\n", configSeq.size());
	this->viewHypotheses.clear();
	for (int i = 0; i < configSeq.size(); i++)
	{
		this->viewHypotheses.push_back(this->generateViewFrom(configSeq[i]));
	}
}


pacman::HypothesisSensor::Ptr ActiveSense::generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius, bool heightBias){

	HypothesisSensor::Config sensorConfig;

	//Show pose lambda
	typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
	ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
		demoOwner->context.debug("ActiveSense: %s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
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

		randomVec.set(sin * Math::cos(phi), sin * Math::sin(phi), cos);
		randomVec.setMagnitude(radius);
	}



	//Generate a new pose
	sensorConfig = golem::Mat34(golem::Mat34::identity());
	sensorConfig.w.p = centroid + randomVec;

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

	sensorConfig.w.R.setColumn(0, n);
	sensorConfig.w.R.setColumn(1, -up);
	sensorConfig.w.R.setColumn(2, f);


	showPose("Generated SensorPose:", sensorConfig.w);
	//Creating uniformly generated random hypothesis sensor
	pacman::HypothesisSensor::Ptr s(new HypothesisSensor(sensorConfig, golem::RGBA(r, g, b, a)));


	return s;

}

void pacman::ActiveSense::generateRandomViews()
{
	this->viewHypotheses.clear();
	for (int i = 0; i < this->params.nsamples; i++)
	{
		this->viewHypotheses.push_back(this->generateNextRandomView(this->params.centroid, this->params.radius, this->params.useHeightBias));
	}
}

void pacman::ActiveSense::generateViews()
{
	if (this->params.generationMethod == EGenerationMethod::G_RANDOM_SPHERE)
		generateRandomViews();
	else if (this->params.generationMethod == EGenerationMethod::G_FIXED){
		generateViewsFromSeq(this->params.configSeq);
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
	typedef std::vector<const grasp::data::ItemContactModel::Data::Map*> TrainingData;
	TrainingData trainingData;

	//demoOwner->context.debug("ActiveSense: Getting ItemContactModel\n");
	// collect data (TODO: Receive a list of ItemContactModel
	const grasp::data::ItemContactModel* model = is<const grasp::data::ItemContactModel>(input);
	if (model)
	{
		trainingData.push_back(&model->dataMap);
	}
	else
	{
		demoOwner->context.debug("ActiveSense: This is not an ItemContactModel\n");
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
			//demoOwner->context.debug("Theta %lf Weight %lf Sigma %f CDF %lf LastCDF %lf \n", theta, weight, sigma, it->cdf, last.cdf);

		}
		value /= cdf;

		return value;
	};

	golem::Real maxValue(-golem::REAL_MAX), currValue(0.0);
	std::string maxType = "None";
	//demoOwner->context.debug("Computing Value\n");
	for (TrainingData::const_iterator i = trainingData.begin(); i != trainingData.end(); ++i){
		//For each graspType's mapping of joint->contacts do...
		for (grasp::data::ItemContactModel::Data::Map::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j) {

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
				//demoOwner->context.debug("---JOINT %s---\n", k->first);
				//Adds individual contributions of each joint

				currValue += valFunc(k->second, hypothesis->getFrame());
				//demoOwner->context.debug("currValue %f CurrentMax %f\n", currValue, maxValue);
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
	demoOwner->context.debug("ActiveSense: Select collision object...\n");

	const data::Point3D* location = is<const data::Point3D>(input->second.get());
	
	
	if (location) {
		demoOwner->context.debug("ActivSense: Number of Locations %d\n", location->getNumOfPoints());
		// create collision bounds
		collisionBounds.reset(new CollisionBounds(*demoOwner->planner, [=](size_t i, Vec3& p) -> bool {

			if (i < location->getNumOfPoints()) p = location->getPoint(i); return i < location->getNumOfPoints();
		}, draw ? &demoOwner->objectRenderer : nullptr, draw ? &demoOwner->scene.getCS() : nullptr));
		// draw locations
		golem::CriticalSectionWrapper csw(demoOwner->scene.getCS());
		for (size_t i = 0; i < location->getNumOfPoints(); ++i)
			demoOwner->objectRenderer.addPoint(location->getPoint(i), golem::RGBA::BLACK);
	}
	else
		demoOwner->context.debug("ActiveSense: Object collisions unsupported\n");


	return collisionBounds;
}

grasp::data::Item::Map::iterator ActiveSense::convertToTrajectory(grasp::data::Item::Map::iterator predQueryModel)
{
	grasp::Manager::RenderBlock renderBlock(*demoOwner);
	grasp::data::Convert* convert = grasp::is<grasp::data::Convert>(predQueryModel);
	if (!convert)
		throw Message(Message::LEVEL_ERROR, "Input item does not support Convert interface");

	// find matching handlers
	typedef std::vector<data::Handler*> HandlerSet;
	HandlerSet handlerSet;
	//for (data::Handler::Map::const_iterator i = demoOwner->handlerMap.begin(); i != demoOwner->handlerMap.end(); ++i)
	//	if (std::find(convert->getHandlerTypes().begin(), convert->getHandlerTypes().end(), i->second->getType()) != convert->getHandlerTypes().end())
	//		handlerSet.push_back(i->second.get());


	// pick up handler
	data::Handler::Map::iterator handlerPtr = demoOwner->handlerMap.find("Trajectory+TrajectoryActiveSense");
	if (handlerPtr == demoOwner->handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "ActiveSense::convertToTrajectory: Trajectory+TrajectoryActiveSense not in handler map");

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

		demoOwner->context.debug("ActiveSense: ---[computeFeedBackTransform]: List Size: %d---\n", list.size());
		// transform
		golem::shared_ptr<grasp::Manager::InputBlock> inputBlock;
		if (!this->allowInput) inputBlock = golem::shared_ptr<grasp::Manager::InputBlock>(new grasp::Manager::InputBlock(*demoOwner));

		Manager::RenderBlock renderBlock(*demoOwner);

		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		
		inputBlock.release();

		return ptr;
	};

	grasp::data::Item::List list;
	list.push_back(predModelItem);
	list.push_back(pointCurvItem);

	demoOwner->context.debug("ActiveSense: Computing FeedBackQuery!\n");
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

	demoOwner->context.debug("ActiveSense: Computing Trajectory!\n");
	grasp::data::Item::Map::iterator traj = convertToTrajectory(predQuery);

	{
		Manager::RenderBlock renderBlock(*demoOwner);
		golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
		// add trajectory item into current data bundle (so can execute grasp again)
		std::string trajectoryName("ActiveSens_Grasp");
		grasp::data::Item::Map& itemMap = to<Demo::Data>(demoOwner->dataCurrentPtr)->itemMap;
		itemMap.erase(trajectoryName);
		grasp::data::Item::Map::iterator ptr = itemMap.insert(itemMap.end(), grasp::data::Item::Map::value_type(trajectoryName, traj->second));
		Demo::Data::View::setItem(itemMap, ptr, to<Demo::Data>(demoOwner->dataCurrentPtr)->getView());
	}
	

	return computeTransformPredModel(pointCurvItem, traj);
}

grasp::data::Item::Map::iterator pacman::ActiveSense::computeTransformPredModel(grasp::data::Item::Map::iterator trajItem, grasp::data::Item::Map::iterator pointCurvItem)
{
	std::function<grasp::data::Item::Map::iterator(grasp::data::Item::List&, TransformMap::value_type&, const std::string&)> reduce = [&](grasp::data::Item::List& list, TransformMap::value_type& transformPtr, const std::string& itemLabel) -> grasp::data::Item::Map::iterator {

		demoOwner->context.debug("\nActiveSense: [TransformPredModel]---List Size: %d---\n", list.size());
		// transform
		golem::shared_ptr<grasp::Manager::InputBlock> inputBlock;
		if (!this->allowInput) inputBlock = golem::shared_ptr<grasp::Manager::InputBlock>(new grasp::Manager::InputBlock(*demoOwner));
		Manager::RenderBlock renderBlock(*demoOwner);

		UI::addCallback(*demoOwner, transformPtr.first);
		data::Item::Map::iterator ptr;
		grasp::data::Item::Ptr item = transformPtr.second->transform(list);
		{
			golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
			ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
			grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());


		}

		inputBlock.release();
		return ptr;
	};



	grasp::data::Item::List list;

	list.clear();
	//Feedback
	list.push_back(pointCurvItem);
	list.push_back(trajItem);


	demoOwner->context.debug("ActiveSense: Computing predModelNew");

	grasp::data::Item::Map::iterator predModelNew = reduce(list, transformMap[2], "predModelNew");


	return predModelNew;
}

/** Ad-hoc Tools for cloud processing ***/

void pacman::ActiveSense::removeInliers(grasp::Cloud::PointCurvSeq::Ptr cloudIn, const pcl::PointIndices::Ptr& inliersIn, grasp::Cloud::PointCurvSeq::Ptr cloudOut, bool negative)
{
	demoOwner->context.debug("ActiveSense: Removing plane inliers.\n");
	// Create the filtering object
	pcl::ExtractIndices<grasp::Cloud::PointCurv> extract;
	// Extract the inliers
	extract.setInputCloud(cloudIn);
	extract.setIndices(inliersIn);
	extract.setNegative(negative);
	extract.filter(*cloudOut);

	demoOwner->context.debug("ActiveSense: Point cloud representing the non planar component has %d data points.\n", cloudOut->width * cloudOut->height);

}

void pacman::ActiveSense::segmentPlane(grasp::Cloud::PointCurvSeq::Ptr cloudIn, const pcl::ModelCoefficients::Ptr& coefficientsOut, const pcl::PointIndices::Ptr& inliersOut)
{
	demoOwner->context.debug("ActiveSense: Plane segmentation.\n");

	// Create the segmentation object
	pcl::SACSegmentation<grasp::Cloud::PointCurv> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloudIn);
	seg.segment(*inliersOut, *coefficientsOut);

	demoOwner->context.debug("ActiveSense: PointCloud after plane segmentation has %d data points.\n", inliersOut->indices.size());


}
void pacman::ActiveSense::projectOnSphere(grasp::Cloud::PointCurvSeq::Ptr cloudIn, const pcl::ModelCoefficients::Ptr& coefficientsIn, grasp::Cloud::PointCurvSeq::Ptr cloudOut)
{
	demoOwner->context.debug("ActiveSense: Projecting object points on sphere.\n");

	pcl::copyPointCloud<grasp::Cloud::PointCurv, grasp::Cloud::PointCurv>(*cloudIn, *cloudOut);

	golem::Vec3 sphereCenter(coefficientsIn->values[0], coefficientsIn->values[1], coefficientsIn->values[2]);
	golem::Real sphereRadius = coefficientsIn->values[3];
	golem::Vec3 radiusVec(0.0, 0.0, 0.0);
	golem::Real factor = 0.0;
	for (int i = 0; i < cloudOut->points.size(); i++)
	{
		radiusVec.x = cloudOut->points[i].x - sphereCenter.x;
		radiusVec.y = cloudOut->points[i].y - sphereCenter.y;
		radiusVec.z = cloudOut->points[i].z - sphereCenter.z;
		factor = sphereRadius / sqrt(radiusVec.x*radiusVec.x + radiusVec.y*radiusVec.y + radiusVec.z*radiusVec.z);
		//Making its magnitude equals to sphereRadius
		radiusVec.x *= factor; radiusVec.y *= factor; radiusVec.z *= factor;

		//Projecting onto the sphere surface
		cloudOut->points[i].x = sphereCenter.x + radiusVec.x;
		cloudOut->points[i].y = sphereCenter.y + radiusVec.y;
		cloudOut->points[i].z = sphereCenter.z + radiusVec.z;
	}

	demoOwner->context.debug("ActiveSense: PointCloud after projection has %d data points.\n", cloudOut->points.size());


}
void pacman::ActiveSense::computeHull(grasp::Cloud::PointCurvSeq::Ptr cloudIn, grasp::Cloud::PointCurvSeq::Ptr cloudOut, std::vector< pcl::Vertices > &polygonOut, golem::Real &area, golem::Real& volume)
{
	demoOwner->context.debug("ActiveSense: Computing Convex Hull.\n");

	// Create a Concave Hull representation of the projected inliers
	pcl::ConvexHull<grasp::Cloud::PointCurv> chull;
	chull.setComputeAreaVolume(true);
	chull.setInputCloud(cloudIn);
	chull.reconstruct(*cloudOut, polygonOut);
	area = chull.getTotalArea();
	volume = chull.getTotalVolume();
	demoOwner->context.debug("ActiveSense: -----\n\nConvex hull has %d data points. Area is %lf, Volume is %lf-----\n\n", cloudOut->points.size(),area, volume);

}
golem::Real pacman::ActiveSense::computeCoverage(grasp::Cloud::PointCurvSeq::Ptr cloudIn, std::vector< pcl::Vertices >& polygonOut)
{
	demoOwner->context.debug("ActiveSense: Computing Coverage.\n");
	//Local
	golem::Vec3 centroid;
	grasp::Cloud::PointCurvSeq::Ptr tempCloud(new grasp::Cloud::PointCurvSeq);
	golem::Real coverage = 0.0, area = 0.0, volume = 0.0;



	if (params.filterPlane) {
		pcl::ModelCoefficients::Ptr coefficientsPlane(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		segmentPlane(cloudIn, coefficientsPlane, inliers);
		removeInliers(cloudIn, inliers, tempCloud);
	}
	else
	{
		pcl::copyPointCloud<grasp::Cloud::PointCurv, grasp::Cloud::PointCurv>(*cloudIn, *tempCloud);
	}


	centroid = computeCentroid2(tempCloud);

	pcl::ModelCoefficients::Ptr coefficientsSphere(new pcl::ModelCoefficients);
	coefficientsSphere->values.resize(4);
	coefficientsSphere->values[0] = centroid.x, coefficientsSphere->values[1] = centroid.y, coefficientsSphere->values[2] = centroid.z;
	coefficientsSphere->values[3] = 1.0;

	projectOnSphere(tempCloud, coefficientsSphere, tempCloud);

	computeHull(tempCloud, tempCloud, polygonOut, area, volume);

	if (params.coverageMethod == ECoverageMethod::M_AREA_BASED)
		coverage = area / (4.0*golem::REAL_PI); //currentArea/TotalSphereSurfaceArea (unit sphere)
	else if (ECoverageMethod::M_VOLUME_BASED)
		coverage = volume / ((4.0 / 3.0)*golem::REAL_PI); //currentArea/TotalSphereVolume(unit sphere)
	else
		throw Cancel("computeCoverage: coverage method unsupported");


	demoOwner->context.debug("ActiveSense: current coverage is %lf\n", coverage);
	return coverage;

}


void pacman::ActiveSense::executeTrajectory()
{
	demoOwner->context.debug("ActiveSense: Attempting to execute trajectory...\n");

	if (this->result.trajectories.size() <= 0)
		throw Cancel("ActiveSense: could not execute grasp because there's no trajectory!");

	data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());
	// play
	Controller::State::Seq seq;
	trajectory->createTrajectory(seq);
	// select collision object
	CollisionBounds::Ptr collisionBounds = this->selectCollisionBounds(true, this->result.pointCurvs.back());
	demoOwner->context.debug("ActiveSense: Performing trajectory! \n");
	// perform
	this->demoOwner->perform2(this->demoOwner->dataCurrentPtr->first, this->result.trajectories.back()->first, seq);

	pLastExecutedWaypoint.reset(new Controller::State(seq.front()));

	// done!
	this->demoOwner->createRender();
}

//--------------------------------------------------------------------------------
void pacman::ActiveSenseController::initActiveSense(pacman::Demo* demoOwner)
{
	this->activeSense->init(demoOwner);


	//Mat34 sensorPose;

	/*Up right sensor
	sensorPose.R.setColumn(0,golem::Vec3(0.0, 0.0, 1.0));
	sensorPose.R.setColumn(1,golem::Vec3(1.0, 0.0, 0.0));
	sensorPose.R.setColumn(2,golem::Vec3(0.0, 1.0, 0.0));
	*/

	//Upside down sensor
	//sensorPose.R.setColumn(0, golem::Vec3(1.0, 0.0, 0.0));
	//sensorPose.R.setColumn(1, golem::Vec3(0.0, 0.0, -1.0));
	//sensorPose.R.setColumn(2, golem::Vec3(0.0, 1.0, 0.0));

	//sensorPose.p.set(0.5, -0.5, 0.025);

}


