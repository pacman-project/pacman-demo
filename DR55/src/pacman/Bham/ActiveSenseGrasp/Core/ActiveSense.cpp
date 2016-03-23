/** @file ActiveSense.cpp
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#include <Golem/Tools/XMLData.h>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/ContactModel/ContactModel.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>

#include <Grasp/Data/Trajectory/Trajectory.h>

#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSense.h"
#include "pacman/Bham/ActiveSenseGrasp/Utils/PCLConversions.h"
#include "pacman/Bham/ActiveSenseGrasp/Demo/ActiveSenseGraspDemo.h"

#include "pacman/Bham/ActiveSenseGrasp/IO/IO_Adhoc.h"

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


ActiveSense::ActiveSense(pacman::ActiveSenseDemo* demoOwner) : ActiveSense()
{
    this->init(demoOwner);
}

void ActiveSense::init(pacman::ActiveSenseDemo* demoOwner)
{
    this->demoOwner = demoOwner;
    rand.setRandSeed(this->demoOwner->context.getRandSeed());
    for(auto it = demoOwner->handlerMap.begin(); it != demoOwner->handlerMap.end(); it++)
    {
        demoOwner->context.write("handler: %s\n", it->first.c_str());
    }
    grasp::data::Handler::Map::iterator itImage = demoOwner->handlerMap.find(this->params.imageHandler);
	grasp::data::Handler::Map::iterator itImage2 = demoOwner->handlerMap.find(this->params.imageHandlerNoCrop);
	grasp::data::Handler::Map::iterator itPointCurv = demoOwner->handlerMap.find(this->params.pointCurvHandler);
	grasp::data::Handler::Map::iterator itcontactModel = demoOwner->handlerMap.find(this->params.contactHandler);
	grasp::data::Handler::Map::iterator itcontactQuery = demoOwner->handlerMap.find(this->params.queryHandler);
    grasp::data::Transform* transformImage = is<grasp::data::Transform>(itImage);
    grasp::data::Transform* transformImage2 = is<grasp::data::Transform>(itImage);
    grasp::data::Transform* transformPointsCurv = is<grasp::data::Transform>(itPointCurv);
    grasp::data::Transform* transformPredictorModel = is<grasp::data::Transform>(itcontactModel);
    grasp::data::Transform* transformPredictorQuery = is<grasp::data::Transform>(itcontactQuery);

    if (!transformImage || !transformPointsCurv || !transformPredictorModel || !transformPredictorQuery || !transformImage2)
    {
        std::stringstream ss;
        ss << (transformImage? 1:0) <<" " << (transformPointsCurv? 1:0) << " " << (transformPredictorModel? 1:0) << " " << (transformPredictorQuery? 1:0);
        std::string msg = "We don't have all Transforms we need! " + ss.str();
        throw Cancel(msg.c_str());
    }
    else
    {
        demoOwner->context.debug("ActiveSense: All transforms good!\n");
    }


    //Currently unused (Generates unorganized pointclouds that cannot be transformed in PointsCurv)
    transformMap.push_back(std::make_pair(itImage->second.get(), transformImage));

    transformMap.push_back(std::make_pair(itPointCurv->second.get(), transformPointsCurv));

    transformMap.push_back(std::make_pair(itcontactModel->second.get(), transformPredictorModel));

    transformMap.push_back(std::make_pair(itcontactQuery->second.get(), transformPredictorQuery));
    transformMap.push_back(std::make_pair(itImage2->second.get(), transformImage2));


    demoOwner->context.debug("ActiveSense: generating views!\n");
    this->generateViews();

    this->allowInput = false;

    this->onlineModel2.init(demoOwner->collision, demoOwner->manipulator );
    demoOwner->context.debug("ActiveSense: GOOD!\n");

    this->out = NULL;
    this->experiment_trial = this->experiment_id = 0;
    this->experiment_alias = "no_alias";

    this->nbvViews = 0;
    this->safetyViews = 0;
}

/** Load from xml context */
void ActiveSense::Parameters::load(const golem::XMLContext* xmlcontext)
{
    golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("active_sense parameters");


    std::string selectionMethodStr, alternativeSelectionMethodStr, generationMethodStr, coverageMethodStr, stoppingCriteriaStr, safetyStoppingCriteriaStr;

    XMLData("sensor_id", this->sensorId, pxmlcontext);
    XMLData("selection_method", selectionMethodStr, pxmlcontext);
    XMLData("alternative_selection_method", alternativeSelectionMethodStr, pxmlcontext);
    XMLData("generation_method", generationMethodStr, pxmlcontext);
    XMLData("enable_regeneration", this->regenerateViews, pxmlcontext);
    XMLData("use_manual_centroid", this->useManualCentroid, pxmlcontext);
    XMLData("use_height_bias", this->useHeightBias, pxmlcontext);
    XMLData("show_sensor_hypotheses", this->showSensorHypotheses, pxmlcontext);
    XMLData("radius", this->radius, pxmlcontext);
    XMLData("nsamples", this->nsamples, pxmlcontext);
    XMLData("nviews", this->nviews, pxmlcontext);
    XMLData("maxnviews", this->maxnviews, pxmlcontext);
    XMLData("maxsafetynviews", this->maxnviews, pxmlcontext);
    XMLData("coverage_threshold", this->coverageThr, pxmlcontext);
    XMLData("coverage_method", coverageMethodStr, pxmlcontext);
    XMLData("stopping_criteria", stoppingCriteriaStr, pxmlcontext);
    XMLData("safety_stopping_criteria", safetyStoppingCriteriaStr, pxmlcontext);
    XMLData("entropy_threshold", this->entropyThr, pxmlcontext);
    XMLData("enable_coverage_filter_plane", this->filterPlane, pxmlcontext);
    XMLData("min_phi", this->minPhi, pxmlcontext);
    XMLData("max_phi", this->maxPhi, pxmlcontext);
    XMLData("min_theta", this->minTheta, pxmlcontext);
    XMLData("max_theta", this->maxTheta, pxmlcontext);
    XMLData("use_sim_cam", this->useSimCam, pxmlcontext);

    XMLData(this->centroid, pxmlcontext->getContextFirst("centroid"), false);


	golem::XMLData("contact_handler", this->contactHandler, pxmlcontext->getContextFirst("handler_map"));
	golem::XMLData("query_handler", this->queryHandler, pxmlcontext->getContextFirst("handler_map"));
	golem::XMLData("image_handler", this->imageHandler, pxmlcontext->getContextFirst("handler_map"));
	golem::XMLData("image_handler_no_crop", this->imageHandlerNoCrop, pxmlcontext->getContextFirst("handler_map"));
	golem::XMLData("point_curv_handler", this->pointCurvHandler, pxmlcontext->getContextFirst("handler_map"));
	golem::XMLData("trajectory_handler", this->trajectoryHandler, pxmlcontext->getContextFirst("handler_map"));

    CoverageMethodMap coverageMethodMap = getCoverageMethodMap();
    StoppingCriteriaMap stoppingCriteriaMap = getStoppingCriteriaMap();
    SafetyStoppingCriteriaMap safetyStoppingCriteriaMap = getSafetyStoppingCriteriaMap();
    SelectionMethodMap selectionMethodMap = getSelectionMethodMap();
    GenerationMethodMap generationMethodMap = getGenerationMethodMap();

    auto coverageMethod = coverageMethodMap.find(coverageMethodStr);
    if( coverageMethod != coverageMethodMap.end())
        this->coverageMethod = coverageMethod->second;
    else
        this->coverageMethod = ECoverageMethod::M_NONE;

    auto stopMethod = stoppingCriteriaMap.find(stoppingCriteriaStr);
    if (stopMethod != stoppingCriteriaMap.end())
        this->stoppingCriteria = stopMethod->second;
    else
        this->stoppingCriteria = EStoppingCriteria::C_NONE;

    auto safetyStopMethod = safetyStoppingCriteriaMap.find(safetyStoppingCriteriaStr);
    if (safetyStopMethod != safetyStoppingCriteriaMap.end())
        this->safetyStoppingCriteria = safetyStopMethod->second;
    else
        this->safetyStoppingCriteria = ESafetyStoppingCriteria::ES_NONE;

    auto selectionMethod = selectionMethodMap.find(selectionMethodStr);
    if ( selectionMethod != selectionMethodMap.end())
        this->selectionMethod = selectionMethod->second;
    else
        this->selectionMethod = ESelectionMethod::S_NONE;

    auto altSelctionMethod = selectionMethodMap.find(alternativeSelectionMethodStr);
    if (altSelctionMethod != selectionMethodMap.end())
        this->alternativeSelectionMethod = altSelctionMethod->second;
    else
        this->alternativeSelectionMethod = ESelectionMethod::S_NONE;

    auto generationMethod = generationMethodMap.find(generationMethodStr);
    if (generationMethod != generationMethodMap.end())
        this->generationMethod = generationMethod->second;
    else
        this->generationMethod = EGenerationMethod::G_NONE;

    dropOffPose.xmlData(pxmlcontext->getContextFirst("drop_off_pose"));

    if (this->generationMethod == EGenerationMethod::G_FIXED)
    {
        configSeq.clear();
        XMLData(configSeq, configSeq.max_size(), const_cast<golem::XMLContext*>(pxmlcontext), "pose");

        printf("Loaded %ld fixed poses:\n", configSeq.size());

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
    else if(this->generationMethod == EGenerationMethod::G_RANDOM_SPHERE){
        configSeq.clear();
        XMLData(configSeq, configSeq.max_size(), const_cast<golem::XMLContext*>(pxmlcontext), "pose");

        printf("Loaded %ld fixed poses:\n", configSeq.size());

        if(configSeq.size())
            configSeq.resize(1);

    }

    printf("ActiveSense: method %s usmc %d ushb %d radius %f nsamples %d nviews %d\n", selectionMethod->first.c_str(), this->useManualCentroid, this->useHeightBias, this->radius, this->nsamples, this->nviews);
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


/************************ Utils Player Commands *************************************************/

grasp::data::Item::Map::iterator ActiveSense::addItem(const std::string& label, grasp::data::Item::Ptr item)
{
    Manager::RenderBlock renderBlock(*demoOwner);
    grasp::data::Item::Map::iterator ptr;
    {
        golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
        ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(label, item));

        grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());
    }

    // enable GUI interaction and refresh
    UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
    demoOwner->createRender();

    return ptr;
}

grasp::data::Item::Map::iterator ActiveSense::addItem(const std::string& label, grasp::data::Item::Ptr& item, grasp::Manager::Data::Ptr dataBundle)
{
    Manager::RenderBlock renderBlock(*demoOwner);
    grasp::data::Item::Map::iterator ptr;
    {
        golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
        ptr = dataBundle->itemMap.insert(dataBundle->itemMap.end(), grasp::data::Item::Map::value_type(label, item));
        //grasp::Manager::Data* d = dataPtr.get();

        grasp::Manager::Data::View::setItem(dataBundle->itemMap, ptr, grasp::to<grasp::Manager::Data>(dataBundle)->getView());
    }

    // enable GUI interaction and refresh
    //UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
    //demoOwner->createRender();

    return ptr;
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

void pacman::ActiveSense::removeItem(grasp::data::Item::Map::iterator itemPtr)
{
    if (itemPtr == demoOwner->dataCurrentPtr->second->itemMap.end() )
        return;

    {
        grasp::UI::removeCallback(*demoOwner, &itemPtr->second->getHandler());
        golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
        demoOwner->dataCurrentPtr->second->itemMap.erase(itemPtr);
    }



    //grasp::UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
    demoOwner->createRender();
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

void pacman::ActiveSense::saveData(const grasp::Manager::Data& data) {
    std::string path = "./data/";
    std::string ext = ".xml";

    demoOwner->readPath("Enter data path to save: ", path, ext.c_str());
    if (getExt(path).empty())
        path += ext;

    data.save(path);

    demoOwner->getContext().debug("Done! Saved data!");
}

void pacman::ActiveSense::addData(grasp::Manager::Data::Ptr data){

    demoOwner->readPath("Enter data path to create: ", this->dataPath);
    if (!isExt(this->dataPath, demoOwner->dataExt))
        this->dataPath += demoOwner->dataExt;


    {
        golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
        demoOwner->dataMap.erase(demoOwner->dataPath);
        demoOwner->dataCurrentPtr = demoOwner->dataMap.insert(demoOwner->dataMap.begin(), grasp::Manager::Data::Map::value_type(this->dataPath, data));
    }



}

grasp::Manager::Data::Ptr pacman::ActiveSense::createData()
{
    //Create new Data Bundle
    grasp::Manager::Data::Ptr data;

    data = demoOwner->createData();
    grasp::Manager::RenderBlock renderBlock(*demoOwner);
    demoOwner->scene.getOpenGL(grasp::to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView().openGL); // set current view
    demoOwner->scene.getOpenGL(grasp::to<grasp::Manager::Data>(data)->getView().openGL); // set view of the new data

    demoOwner->context.debug("ActiveSense: Done: Created Data Bundle!\n");

    return data;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

/************************ Utils Cloud Processing *************************************************/

golem::Vec3 pacman::ActiveSense::computeCentroid(CloudT::Ptr cloudIn)
{

    golem::Vec3 centroid(0.0, 0.0, 0.0);
    this->demoOwner->context.debug("ActiveSense: Computing Centroid: %lf %lf %lf CloudSize %d \n", centroid.x, centroid.y, centroid.z, cloudIn->size());
    active_sense::PointCloudNormal::Ptr cloud_acs(new active_sense::PointCloudNormal());
    utils::convert(*cloudIn, *cloud_acs);

    Eigen::Vector3f centroid_ = active_sense::computeCentroid2(cloud_acs);
    centroid = golem::Vec3(centroid_.x(),centroid_.y(),centroid_.z());

    this->demoOwner->context.debug("ActiveSense: RESULT Centroid ==>>> %lf %lf %lf\n", centroid.x, centroid.y, centroid.z);

    return centroid;


}
golem::Vec3 pacman::ActiveSense::computeCentroid(grasp::data::Item::Map::const_iterator itemImage)
{

    golem::Vec3 centroid(0.0,0.0,0.0);

    grasp::data::ItemImage* image = is<grasp::data::ItemImage>(itemImage);

    if (!image)
    {
        throw Cancel("ActiveSense: This is not an ItemImage!\n");
    }

    centroid = computeCentroid(image->cloud);



    return centroid;
}


golem::Real pacman::ActiveSense::computeCoverage(grasp::Cloud::PointCurvSeq::Ptr cloudIn)
{
    demoOwner->context.debug("ActiveSense: Computing Coverage.\n");
    std::vector< pcl::Vertices > polygons;
    active_sense::PointCloudNormal::Ptr cloud_acs(new active_sense::PointCloudNormal());
    utils::convert(*cloudIn, *cloud_acs);
    demoOwner->context.debug("ActiveSense: Computing Coverage!!!.\n");
    golem::Real coverage = active_sense::computeCoverage(cloud_acs, polygons);

    demoOwner->context.debug("ActiveSense: current coverage is %lf\n", coverage);
    return coverage;

}
///////////////////////////////////////////////////////////////////////////////////////////////////

/************************ UTILS Player Robot Sensors *********************************************/

grasp::CameraDepth* ActiveSense::getOwnerOPENNICamera()
{
    return grasp::is<grasp::CameraDepth>(getOwnerSensor(params.sensorId));
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

///////////////////////////////////////////////////////////////////////////////////////////////////

golem::Mat34 pacman::ActiveSense::computeGoal(const golem::Mat34& targetFrame, grasp::Camera* camera)
{

    golem::Mat34 frame, invFrame, refPose;
    grasp::ConfigMat34 wrist, invWrist;
    frame.setId(), invFrame.setId(), wrist.w.setId(), invWrist.w.setId(), refPose.setId();


    //Reference frame
    refPose = demoOwner->controller->getChains()[demoOwner->getPlanner().armInfo.getChains().begin()]->getReferencePose();

    //Wrist pose (7th joint)
    camera->getConfig(wrist);
    //Inverse wrist
    invWrist.w.setInverse(wrist.w);

    //It should be the local frame of the camera
    frame = invWrist.w*camera->getFrame(); // camera->to->wrist
    invFrame.setInverse(frame); // wrist->to->camera

    // refPose: refPose->world
    // targetFrame: camera->refPose
    // targetFrame*invFrame: (wrist->camera) 1st[invFrame],(camera->refPose) 2nd[targetFrame] = wrist->refPose
    // targetFrame*invFrame*refPose: wrist->refPose, refPose->world = wrist->world

    // Because:
    // targetFrame = SomeWristFrame*frame*refPose
    golem::Mat34 goal = targetFrame*invFrame*refPose;


    return goal;
}

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestView()
{



    this->visitedHypotheses.clear();
    this->result.pointCurvs.clear();
    this->result.predQueries.clear();
    this->result.trajectories.clear();
    this->result.completeTrajectories.clear();
    demoOwner->context.debug("ActiveSense: Trial %d!\n",this->experiment_trial);




    if( this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3)
        this->onlineModel2.setToDefault();



    for (int i = 0; i < this->viewHypotheses.size(); i++)
        this->viewHypotheses[i]->visited = false;

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    if (camera)
        demoOwner->context.debug("ActiveSense: Camera good!\n");

    //Creating new Data Bundle
    //grasp::Manager::Data::Map::iterator data = createData();

    size_t index = 0, maxNumViews = this->params.nviews;
    pacman::HypothesisSensor::Ptr hypothesis = getViewHypothesis(index++);

    grasp::data::Item::List scannedImageItems; // scanned Items List

    //Adding current sensor pose to the sequence of visited hypotheses
    //    const size_t jointsSize = static_cast<size_t>(demoOwner->info.getJoints().size());
    //    HypothesisSensor::Config config(RealSeq(jointsSize, 0.0));
    //    golem::Controller::State begin = demoOwner->lookupState();

    //    begin.cpos.get(config.c.data(), config.c.data() + std::min(config.c.size(), jointsSize));
    //    hypothesis = this->generateViewFrom(config);
    //    hypothesis->visited = true;
    //    visitedHypotheses.push_back(hypothesis);
    //Finished added current sensor pose as visited

    // performs first scan using current wrist pose: scannedImageItems now has 1 grasp::PointSeq
    hypothesis->visited = true;
    visitedHypotheses.push_back(hypothesis);
    demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

    pacman::io_adhoc::log_out(this->out, "selected", this->experiment_id, this->experiment_trial, -1, false, hypothesis->id, -1, hypothesis->value);

    //Carry on as if nothing happened
    //hypothesis = pacman::HypothesisSensor::Ptr();

    if (!this->params.useManualCentroid)
    {
        demoOwner->context.debug("ActiveSense: Automatically Computing Centroid From First View\n");
        this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
    }

    CollisionBounds::Ptr collisionBounds;
    //printf("Processing image items\n");
    //uncomment me later
    this->pointCurvItem = processItems(scannedImageItems);
    this->hasPointCurv = true;
    if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
            || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
            || this->params.selectionMethod == ESelectionMethod::S_RANDOM){
        //printf("Updating online model2\n");
        this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
        this->onlineModel2.insertCloud(this->pointCurvItem);
        this->demoOwner->createRender();
    }
    //Testing 10/01/2016
    //scannedImageItems.clear();
    //scannedImageItems.push_back(this->pointCurvItem);

    //TODO:
    //Update contact_based_v3 online model
    //Should insert scan given current camera pose (this->getCameraPose())

    demoOwner->context.debug("ActiveSense: FEEDBACK COMPUTATION\n");

    //If we dont have a contact model we generate one using either a contactQuery or a trajectory
    if (!hascontactModel && (this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED
                             || this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2
                             || this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3))
    {
        if (hascontactQuery)
            this->setContactModelItem(this->computeContactModelFeedBack(this->contactQueryItem, this->pointCurvItem));
        else if (hasTrajectory)
            this->setContactModelItem(this->computeTransformcontactModel(this->trajectoryItem, this->pointCurvItem));
        else
            throw Cancel("ActiveSense: Failed to find one of the following required items: contactQueryItem, trajectoryItem");
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

        if (params.selectionMethod == ESelectionMethod::S_CONTACT_BASED
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
                || params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
                || this->params.selectionMethod == ESelectionMethod::S_RANDOM)
        {
            try{
                // Returns a new contact model, and if successful adds a new trajectory to result.trajectories list
                // and also a new Contact Query
                // Raises an exception in case trajectory generation fails, or no contacts are found
                ptr = computeFeedBackTransform(contactModelItem, pointCurvItem);
                found_contacts = true; // ok, we indeed have contacts
            }
            catch (const std::exception& m){
                demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
                //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
                found_contacts = false; // we actually don't have contacts...
            }

        }

        golem::Mat34 goal;
        goal.setId();
        collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
        bool gotToHypothesis = false;
        while (!gotToHypothesis)
        {

            hypothesis = selectNextBestView(ptr, found_contacts);

            //TODO:
            // - Select NBV based on contacts (if any) and trajectory (result->trajectories)


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
        if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3 || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN)
            this->onlineModel2.setCurrentSensorPose(this->getCameraPose());

        //Adding hypotheses to the set of visited hypotheses so far (used for not going to this hypothesis again later)
        hypothesis->visited = true;
        visitedHypotheses.push_back(hypothesis);
        //printf("ScanningPoseActive\n");
        demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

        //Integrate views into the current pointCurv
        //printf("Processing image items\n");
        //uncomment me later
        this->pointCurvItem = processItems(scannedImageItems);
        if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3 || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN){
            //printf("Updating online model2!!!!!!!!-------$$$\n");

            this->onlineModel2.insertCloud(this->pointCurvItem);
            this->demoOwner->createRender();
        }

        //Testing 10/01/2016
        //        scannedImageItems.clear();
        //        scannedImageItems.push_back(this->pointCurvItem );

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

            grasp::data::ItemPointsCurv* image = grasp::is<grasp::data::ItemPointsCurv>(this->pointCurvItem);
            golem::Real coverage = 0.0;
            if (image){
                coverage = computeCoverage(image->cloud);
            }
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
    //if (maxNumViews == 1 || !this->result.trajectories.size() || (this->params.selectionMethod != ESelectionMethod::S_CONTACT_BASED && this->params.selectionMethod != ESelectionMethod::S_CONTACT_BASED2 && this->params.selectionMethod != ESelectionMethod::S_CONTACT_BASED3))
    //uncoment me later
    //    {
    try {
        ptr = this->computeFeedBackTransform(contactModelItem, pointCurvItem);
    }catch (const std::exception& m){
        demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
        //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
        found_contacts = false; // we actually don't have contacts...
    }


    demoOwner->context.debug("ActiveSense: NextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n", result.pointCurvs.size(), result.predQueries.size(), result.trajectories.size());

    this->hasPointCurv = this->hascontactModel = this->hasTrajectory =  this->hascontactQuery = false;

    this->experiment_trial++;

    return this->pointCurvItem;
}

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestView2()
{

    if(!hascontactModel) {
        throw Cancel("ActiveSense: No contact model set!\n");
    }

    this->visitedHypotheses.clear();
    this->result.pointCurvs.clear();
    this->result.predQueries.clear();
    this->result.trajectories.clear();
    this->result.completeTrajectories.clear();
    demoOwner->context.debug("ActiveSense: Trial %d!\n",this->experiment_trial);


    bool found_safe_traj = false;


    if( this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3)
        this->onlineModel2.setToDefault();

    for (int i = 0; i < this->viewHypotheses.size(); i++)
        this->viewHypotheses[i]->visited = false;

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    if (camera)
        demoOwner->context.debug("ActiveSense: Camera good!\n");

    size_t index = 0, maxNumViews = this->params.nviews;
    pacman::HypothesisSensor::Ptr hypothesis = getViewHypothesis(index++);

    // Scanned Image Items List
    grasp::data::Item::List scannedImageItems;

    // Performs first scan using current wrist pose: scannedImageItems now has 1 grasp::PointSeq
    hypothesis->visited = true;
    visitedHypotheses.push_back(hypothesis);
    demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

    pacman::io_adhoc::log_out(this->out, "selected", this->experiment_id, this->experiment_trial, -1, false, hypothesis->id, -1, hypothesis->value);

    // Computes centroid from first scanned item
    if (!this->params.useManualCentroid)
    {
        demoOwner->context.debug("ActiveSense: Automatically Computing Centroid From First View\n");
        this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
    }

    // Collision bounds for the getPlanner()
    //(initialised when we need to plan a trajectory)
    CollisionBounds::Ptr collisionBounds;

    // Computes integrated point cloud with curvatures from the list of point clouds
    this->pointCurvItem = processItems(scannedImageItems);
    this->hasPointCurv = true;

    // Adding current pointCurvItem to our onlineModel
    if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
            || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
            || this->params.selectionMethod == ESelectionMethod::S_RANDOM){
        this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
        this->onlineModel2.insertCloud(this->pointCurvItem);
        this->demoOwner->createRender();
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

        // Find contact points and grasp trajectory
        bool new_traj = false;
        if (params.selectionMethod == ESelectionMethod::S_CONTACT_BASED
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
                || params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
                || this->params.selectionMethod == ESelectionMethod::S_RANDOM)
        {
            try{
                // Returns a new contact model, and if successful adds a new trajectory to result.trajectories list
                // and also a new Contact Query
                // Raises an exception in case trajectory generation fails, or no contacts are found
                int traj_size = result.trajectories.size();
                ptr = computeFeedBackTransform(contactModelItem, pointCurvItem);
                found_contacts = true; // ok, we indeed have contacts
                // we have a new trajectory?
                new_traj = traj_size < result.trajectories.size();

            }
            catch (const std::exception& m){
                demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
                //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
                found_contacts = false; // we actually don't have contacts...
            }

        }

        // If we have a new trajectory, we need to check for its safety
        if(new_traj){

            std::vector<std::pair<grasp::data::Item::Ptr, float> > bestTrajectories;
            data::Trajectory* trajectory = is<data::Trajectory>(result.trajectories.back());
            safetyExploration2(trajectory, scannedImageItems, bestTrajectories);

            // Get best trajectory
            auto best_traj = bestTrajectories.front();
            std::stringstream ss;
            ss << "Safest Trajectory has collision probability " << best_traj.second << ". Accept (Y/N)?";
            found_safe_traj = demoOwner->option("YN", ss.str().c_str()) == 'Y';

            if( found_safe_traj ){

                result.completeTrajectories.push_back(best_traj);

                break;
            }


        }

        // Select and go to next best view
        golem::Mat34 goal;
        goal.setId();
        collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
        bool gotToHypothesis = false;
        while (!gotToHypothesis)
        {

            hypothesis = selectNextBestView(ptr, found_contacts);

            //TODO:
            // - Select NBV based on contacts (if any) and trajectory (result->trajectories)


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

        //Number of views increment
        ++numViewsAcquired;

        // Perform scan on current view
        demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

        //Integrate views into the current pointCurv
        this->pointCurvItem = processItems(scannedImageItems);

        // Adding current pointCurvItem to our onlineModel
        if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3 || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN){
            //printf("Updating online model2!!!!!!!!-------$$$\n");
            this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
            this->onlineModel2.insertCloud(this->pointCurvItem);
            this->demoOwner->createRender();
        }


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

            grasp::data::ItemPointsCurv* image = grasp::is<grasp::data::ItemPointsCurv>(this->pointCurvItem);
            golem::Real coverage = 0.0;
            if (image){
                coverage = computeCoverage(image->cloud);
            }
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
    } // View selection loop

    demoOwner->context.debug("\n\nActiveSense: ************************ COMPLETED AFTER %d OBSERVATION%s ************************\n\n",
                             numViewsAcquired, numViewsAcquired==1 ? "" : "S" );

    if(!found_safe_traj){
        bool new_traj = false;
        try {
            int traj_size = result.trajectories.size();
            ptr = this->computeFeedBackTransform(contactModelItem, pointCurvItem);
            new_traj = traj_size < result.trajectories.size();
        }catch (const std::exception& m){
            demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
            //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
            found_contacts = false; // we actually don't have contacts...
        }

        // If we have a new trajectory, we need to check for its safety
        if(new_traj){

            std::vector<std::pair<grasp::data::Item::Ptr, float> > bestTrajectories;
            data::Trajectory* trajectory = is<data::Trajectory>(result.trajectories.back());
            safetyExploration2(trajectory, scannedImageItems, bestTrajectories);

            // Get best trajectory
            auto best_traj = bestTrajectories.front();
            std::stringstream ss;
            ss << "Safest Trajectory has collision probability " << best_traj.second << ". Accept (Y/N)?";
            found_safe_traj = demoOwner->option("YN", ss.str().c_str()) == 'Y';

            if( found_safe_traj ) {
                result.completeTrajectories.push_back(best_traj);

                demoOwner->context.debug("ActiveSense: Has found safe trajectory at last generated grasp");
            }


        }

    }
    else{

        demoOwner->context.debug("ActiveSense: finished early because has found safe trajectory");

    }



    demoOwner->context.debug("ActiveSense: NextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n", result.pointCurvs.size(), result.predQueries.size(), result.trajectories.size());

    this->hasPointCurv = this->hascontactModel = this->hasTrajectory =  this->hascontactQuery = false;

    this->experiment_trial++;

    return this->pointCurvItem;
}

grasp::data::Item::Map::iterator pacman::ActiveSense::nextBestView3()
{

    if(!hascontactModel) {
        throw Cancel("ActiveSense: No contact model set!\n");
    }

    this->visitedHypotheses.clear();
    this->result.pointCurvs.clear();
    this->result.predQueries.clear();
    this->result.trajectories.clear();
    this->result.completeTrajectories.clear();
    demoOwner->context.debug("ActiveSense: Trial %d!\n",this->experiment_trial);


    bool found_safe_traj = false;




    if( this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3)
        this->onlineModel2.setToDefault();

    for (int i = 0; i < this->viewHypotheses.size(); i++)
        this->viewHypotheses[i]->visited = false;

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    if (camera)
        demoOwner->context.debug("ActiveSense: Camera good!\n");

    size_t index = 0, maxNumViews = this->params.nviews;
    pacman::HypothesisSensor::Ptr hypothesis = getViewHypothesis(index++);

    // Scanned Image Items List
    grasp::data::Item::List scannedImageItems;

    // Performs first scan using current wrist pose: scannedImageItems now has 1 grasp::PointSeq
    hypothesis->visited = true;
    visitedHypotheses.push_back(hypothesis);
    demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

    pacman::io_adhoc::log_out(this->out, "selected", this->experiment_id, this->experiment_trial, -1, false, hypothesis->id, -1, hypothesis->value);

    // Computes centroid from first scanned item
    if (!this->params.useManualCentroid)
    {
        demoOwner->context.debug("ActiveSense: Automatically Computing Centroid From First View\n");
        this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
    }

    // Collision bounds for the getPlanner()
    //(initialised when we need to plan a trajectory)
    CollisionBounds::Ptr collisionBounds;

    // Computes integrated point cloud with curvatures from the list of point clouds
    this->pointCurvItem = processItems(scannedImageItems);
    this->hasPointCurv = true;

    // Adding current pointCurvItem to our onlineModel
    if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
            || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
            || this->params.selectionMethod == ESelectionMethod::S_RANDOM){
        this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
        this->onlineModel2.insertCloud(this->pointCurvItem);
        this->demoOwner->createRender();
    }

NBV:

    found_safe_traj = false;


    grasp::data::ItemContactModel::Map::iterator ptr;
    bool stop = maxNumViews > 1 ? false : true; // we could stop after the initial view
    bool found_contacts = true; //lets assume we have contacts
    int numViewsAcquired = 1; // we count the initial view

    if( this->nbvViews >= this->params.maxnviews){
        goto FINISH;
    }

    while (!stop)
    {
        demoOwner->context.debug(
                    "\n\nActiveSense: ************************ STARTING OBSERVATION #%d ************************\n\n",
                    numViewsAcquired+1);

        // Find contact points and grasp trajectory
        if (params.selectionMethod == ESelectionMethod::S_CONTACT_BASED
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2
                || params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3
                || params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN
                || this->params.selectionMethod == ESelectionMethod::S_RANDOM)
        {
            try{
                // Returns a new contact model, and if successful adds a new trajectory to result.trajectories list
                // and also a new Contact Query
                // Raises an exception in case trajectory generation fails, or no contacts are found
                ptr = computeFeedBackTransform(contactModelItem, pointCurvItem);
                found_contacts = true; // ok, we indeed have contacts

                onlineModel2.resetContactTree();
                onlineModel2.insertContacts(ptr);
                this->demoOwner->createRender();


            }
            catch (const std::exception& m){
                demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
                //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
                found_contacts = false; // we actually don't have contacts...
            }

        }


        // Select and go to next best view
        golem::Mat34 goal;
        goal.setId();
        collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
        bool gotToHypothesis = false;
        while (!gotToHypothesis)
        {

            hypothesis = selectNextBestView(ptr, found_contacts);

            //TODO:
            // - Select NBV based on contacts (if any) and trajectory (result->trajectories)


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

        //Number of views increment
        ++numViewsAcquired;
        this->nbvViews++;

        // Perform scan on current view
        demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());

        //Integrate views into the current pointCurv
        this->pointCurvItem = processItems(scannedImageItems);

        // Adding current pointCurvItem to our onlineModel
        if(this->params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3 || this->params.selectionMethod == ESelectionMethod::S_INFORMATION_GAIN){
            //printf("Updating online model2!!!!!!!!-------$$$\n");
            this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
            this->onlineModel2.insertCloud(this->pointCurvItem);
            this->demoOwner->createRender();
        }


        //Checking stopping criteria
        stop = checkStop(numViewsAcquired,maxNumViews);
    } // View selection loop


    demoOwner->context.debug("\n\nActiveSense: ************************ COMPLETED AFTER %d OBSERVATION%s ************************\n\n",
                             numViewsAcquired, numViewsAcquired==1 ? "" : "S" );


    try {

        ptr = this->computeFeedBackTransform(contactModelItem, pointCurvItem);

        onlineModel2.resetContactTree();
        onlineModel2.insertContacts(ptr);
        this->demoOwner->createRender();

    }catch (const std::exception& m){
        demoOwner->context.debug("ActiveSense: computeFeedBackTransform failed because %s\n", m.what());
        //demoOwner->context.debug("ActiveSense: Couldn't find contacts! Will use alternative view selection instead!\n");
        found_contacts = false; // we actually don't have contacts...
    }

    // If we have a new trajectory, we need to check for its safety
    if(result.trajectories.size()){

        std::vector<std::pair<grasp::data::Item::Ptr, float> > bestTrajectories;
        data::Trajectory* trajectory = getSafestTrajectoryToDate();//is<data::Trajectory>(result.trajectories.back());
        safetyExploration2(trajectory, scannedImageItems, bestTrajectories);

        // Get best trajectory
        auto best_traj = bestTrajectories.front();
        std::stringstream ss;
        ss << "Safest Trajectory has collision probability " << best_traj.second << ". Number NBV Views is " << this->nbvViews << " Accept (Y/N)?";
        found_safe_traj = demoOwner->option("YN", ss.str().c_str()) == 'Y';

        if( found_safe_traj ) {
            result.completeTrajectories.push_back(best_traj);

            demoOwner->context.debug("ActiveSense: Has found safe trajectory at last generated grasp");
        }
        else{
            result.trajectories.pop_front();
            goto NBV;
        }


    }


    FINISH:


    demoOwner->context.debug("ActiveSense: NextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n", result.pointCurvs.size(), result.predQueries.size(), result.trajectories.size());

    this->hasPointCurv = this->hascontactModel = this->hasTrajectory =  this->hascontactQuery = false;

    this->experiment_trial++;

    return this->pointCurvItem;
}

bool pacman::ActiveSense::checkStop(int numViewsAcquired, int maxNumViews) {

    bool stop = false;

    //Checking stopping criteria
    if (params.stoppingCriteria == EStoppingCriteria::C_NVIEWS)
    {
        if (numViewsAcquired >= maxNumViews)
        {
            stop = true;
            demoOwner->context.debug("ActiveSense: Stopping because used max number of views\n");
        }

        if(!result.trajectories.size())
            stop = false;

        if(numViewsAcquired >= this->params.maxnviews)
            stop = true;

    }
    else if (params.stoppingCriteria == EStoppingCriteria::C_COVERAGE || params.stoppingCriteria == EStoppingCriteria::C_NVIEWS_COVERAGE)
    {

        grasp::data::ItemPointsCurv* image = grasp::is<grasp::data::ItemPointsCurv>(this->pointCurvItem);
        golem::Real coverage = 0.0;
        if (image){
            coverage = computeCoverage(image->cloud);
        }
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


    return stop;

}

grasp::data::Item::Map::iterator pacman::ActiveSense::collectData()
{

    params.selectionMethod = S_SEQUENTIAL;

    grasp::Manager::Data::Ptr data = createData();

    std::string path = "./data/";
    std::string ext = ".xml";
    demoOwner->readPath("Enter data path to save: ", path, ext.c_str());
    if (getExt(path).empty())
        path += ext;

    this->visitedHypotheses.clear();
    this->result.pointCurvs.clear();
    this->result.predQueries.clear();
    this->result.trajectories.clear();
    this->result.completeTrajectories.clear();

    for (int i = 0; i < this->viewHypotheses.size(); i++)
        this->viewHypotheses[i]->visited = false;

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    if (camera)
        demoOwner->context.debug("ActiveSense: Camera good!\n");

    size_t index = 0, maxNumViews = this->params.nviews;
    pacman::HypothesisSensor::Ptr hypothesis = getViewHypothesis(index++);

    // Scanned Image Items List
    grasp::data::Item::List scannedImageItems;

    // Performs first scan using current wrist pose: scannedImageItems now has 1 grasp::PointSeq
    hypothesis->visited = true;
    visitedHypotheses.push_back(hypothesis);
    demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel(),"",nullptr,data);

    // Computes centroid from first scanned item
    if (!this->params.useManualCentroid)
    {
        demoOwner->context.debug("ActiveSense: Automatically Computing Centroid From First View\n");
        this->params.centroid = this->computeCentroid(*scannedImageItems.begin());
    }
    if( params.centroid.x != params.centroid.x || params.centroid.y != params.centroid.y || params.centroid.z != params.centroid.z)
        params.centroid.set(0,0,0);

    // Collision bounds for the getPlanner()
    //(initialised when we need to plan a trajectory)
    CollisionBounds::Ptr collisionBounds;



    grasp::data::ItemContactModel::Map::iterator ptr;
    bool stop = maxNumViews > 1 ? false : true; // we could stop after the initial view
    bool found_contacts = true; //lets assume we have contacts
    int numViewsAcquired = 1; // we count the initial view
    while (!stop)
    {
        demoOwner->context.debug(
                    "\n\nActiveSense: ************************ STARTING OBSERVATION #%d ************************\n\n",
                    numViewsAcquired+1);

        // Select and go to next best view
        golem::Mat34 goal;
        goal.setId();
        collisionBounds = this->selectCollisionBounds(true, *scannedImageItems.begin());
        bool gotToHypothesis = false;
        while (!gotToHypothesis)
        {

            hypothesis = selectNextBestView(ptr, found_contacts);

            //TODO:
            // - Select NBV based on contacts (if any) and trajectory (result->trajectories)


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

        //Number of views increment
        ++numViewsAcquired;

        // Perform scan on current view
        demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel(),"",nullptr,data);


        //        grasp::data::Item::Ptr item;
        //        try{
        //            grasp::data::Item::List tmpList;
        //            tmpList.push_back(scannedImageItems.back());
        //            item = transformMap[IMAGE].second->transform(tmpList);
        //        }
        //        catch(const Message m){
        //            demoOwner->context.debug("%s",m.what());

        //        }
        //        std::string lb = std::string("Normals")+hypothesis->getLabel();

        //        addItem(lb, item, data);




        stop = checkStop(numViewsAcquired,maxNumViews);


    } // View selection loop

    demoOwner->context.debug("\n\nActiveSense: ************************ COMPLETED AFTER %d OBSERVATION%s ************************\n\n",
                             numViewsAcquired, numViewsAcquired==1 ? "" : "S" );



    demoOwner->context.debug("ActiveSense: NextBestView Finished! Check this->result attribute member for: %d pointCurvs, %d predQueries and %d best trajectories\n", result.pointCurvs.size(), result.predQueries.size(), result.trajectories.size());

    this->hasPointCurv = this->hascontactModel = this->hasTrajectory =  this->hascontactQuery = false;


    data->save(path);

    return this->pointCurvItem;
}



pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestView(grasp::data::Item::Map::iterator contactModelPtr, bool found_contacts)
{
    HypothesisSensor::Ptr hypothesis;

    if ((params.selectionMethod == ESelectionMethod::S_CONTACT_BASED ||
         params.selectionMethod == ESelectionMethod::S_CONTACT_BASED2 ||
         params.selectionMethod == ESelectionMethod::S_CONTACT_BASED3) && !found_contacts)
    {
        hypothesis = selectNextBestView(params.alternativeSelectionMethod, contactModelPtr);
        pacman::io_adhoc::log_out(this->out, "selected", this->experiment_id, this->experiment_trial, params.alternativeSelectionMethod, found_contacts, hypothesis->id, -1, hypothesis->value);


    }
    else
    {
        hypothesis = selectNextBestView(params.selectionMethod, contactModelPtr);
        pacman::io_adhoc::log_out(this->out, "selected", this->experiment_id, this->experiment_trial, params.selectionMethod, found_contacts, hypothesis->id, -1, hypothesis->value);
    }



    return hypothesis;
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestView(int selectionMethod, grasp::data::Item::Map::iterator contactModelPtr)
{
    HypothesisSensor::Ptr hypothesis;

    if (selectionMethod == ESelectionMethod::S_CONTACT_BASED)
    {
        hypothesis = selectNextBestViewContactBased(contactModelPtr);
    }
    else if (selectionMethod == ESelectionMethod::S_CONTACT_BASED2)
    {
        hypothesis = selectNextBestViewContactBased2(contactModelPtr);
    }
    else if (selectionMethod == ESelectionMethod::S_CONTACT_BASED3)
    {
        hypothesis = selectNextBestViewContactBased3(contactModelPtr);
    }
    else if (selectionMethod == ESelectionMethod::S_RANDOM)
    {
        hypothesis = selectNextBestViewRandom();
    }
    else if (selectionMethod == ESelectionMethod::S_SEQUENTIAL)
    {
        hypothesis = selectNextBestViewSequential();
    }
    else if (selectionMethod == ESelectionMethod::S_INFORMATION_GAIN)
    {
        hypothesis = selectNextBestViewInfGain();
    }
    else
        throw Cancel("pacman::ActiveSense::selectNextBestView: unknown selectionMethod");

    if (hypothesis == nullptr)
        throw Cancel("ActiveSense::selectNextBestView: unable to get hypothesis for next best view");


    return hypothesis;
}


pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewRandom()
{
    demoOwner->context.debug("ActiveSense: Next Best View Random\n");
    golem::CriticalSectionWrapper csw(this->csViewHypotheses);

    if (this->params.regenerateViews) this->generateViews();

    golem::U32 index = 0;
    do{
        index = static_cast<golem::U32>(rand.nextUniform<float>()*this->viewHypotheses.size());

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

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewContactBased(grasp::data::Item::Map::iterator contactModelPtr)
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

        ActiveSense::ValueTuple valueTuple = this->computeValue(this->viewHypotheses[i], contactModelPtr);

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

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewContactBased2(grasp::data::Item::Map::iterator contactModelPtr)
{
    golem::CriticalSectionWrapper csw(this->csViewHypotheses);



    if (this->params.regenerateViews) this->generateViews();

    onlineModel.setTrainingData(contactModelPtr);
    onlineModel.updateContacts();
    onlineModel.updateViewingAngles(visitedHypotheses);

    int index = 0;

    Real minValue(golem::REAL_MAX);
    Real value(0.0);
    demoOwner->context.debug("ActiveSense: Next Best View Contact Based V2\n");
    for (int i = 0; i < this->viewHypotheses.size(); i++)
    {
        if (this->viewHypotheses[i]->visited || hasViewed(this->viewHypotheses[i]))
            continue;

        value = onlineModel.computeValue(this->viewHypotheses[i]);

        demoOwner->context.debug("ActiveSense: H[%d] Value: %f\n", i+1, value);

        if (value < minValue)
        {
            index = i;
            minValue = value;
        }
    }

    demoOwner->context.debug("\nActiveSense: Best View Was H[%d] Value: %f\n\n", index+1, minValue);


    return this->getViewHypothesis(index);
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewContactBased3(grasp::data::Item::Map::iterator contactModelPtr)
{
    golem::CriticalSectionWrapper csw(this->csViewHypotheses);



    if (this->params.regenerateViews) this->generateViews();



    int index = 0;

    data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());
    golem::Controller::State::Seq traj;
    trajectory->createTrajectory(traj);
    grasp::Manipulator::Waypoint::Seq path = demoOwner->convertToManipulatorWayPoints(traj);


    Real maxValue(golem::REAL_MIN);//minValue(golem::REAL_MAX);
    Real value(0.0);
    demoOwner->context.debug("ActiveSense: Next Best View Contact Based V3\n");
    for (int i = 0; i < this->viewHypotheses.size(); i++)
    {
        if (this->viewHypotheses[i]->visited || hasViewed(this->viewHypotheses[i]))
            continue;




        value = onlineModel2.computeValueWithHand(this->viewHypotheses[i], path);
        // TODO
        // check if it has trajectory


        //demoOwner->context.debug("ActiveSense: H[%d] Value: %f\n", i+1, value);

        if (value > maxValue)//if (value < minValue)
        {
            index = i;
            maxValue = value;//minValue = value;
        }
    }

    demoOwner->context.debug("\nActiveSense: Best View V3 Was H[%d] Value: %f\n\n", index+1, maxValue);//minValue);


    return this->getViewHypothesis(index);
}

pacman::HypothesisSensor::Ptr pacman::ActiveSense::selectNextBestViewInfGain()
{
    golem::CriticalSectionWrapper csw(this->csViewHypotheses);

    demoOwner->context.debug("ActiveSense: Next Best View Information Gain\n");



    if (this->params.regenerateViews) this->generateViews();

    /* Getting bounding box */
    demoOwner->context.debug("ActiveSense: Getting bounding box\n");
    golem::BoundingBox::Desc boundingBoxDesc;
    golem::Mat34& frame = boundingBoxDesc.pose;
    const data::Point3D* location = is<const data::Point3D>(this->pointCurvItem->second.get());
    demoOwner->context.debug("ActiveSense: locations acquired %d\n",location->getNumOfPoints());
    golem::Vec3 min(golem::REAL_MAX), max(-golem::REAL_MAX), v[3];
    // retrieving axis
    for (size_t j = 0; j < 3; ++j) {
        v[j].setZero();
        v[j][j] = golem::REAL_ONE;
        ((const golem::Mat33&)frame.R).multiply(v[j], v[j]);
    }
    demoOwner->context.debug("ActiveSense: Finding box limits\n");
    golem::Vec3 point;
    for (size_t i = 0; i < location->getNumOfPoints(); ++i) {
        point = location->getPoint(i);
        for (size_t j = 0; j < 3; ++j) {
            const golem::Real proj = point.dot(v[j]);

            min[j] = std::min(min[j], proj);
            max[j] = std::max(max[j], proj);
        }
    }
    /* End: Getting bounding box */
    demoOwner->context.debug("ActiveSense: box limits min(%lf,%lf,%lf) max(%lf,%lf,%lf)\n", min[0],min[1],min[2],max[0], max[1], max[2]);

    demoOwner->context.debug("ActiveSense: creating collision result\n");
    Collision::Result::Ptr collisionResult(new Collision::Result());
    collisionResult->setToDefault();
    demoOwner->context.debug("ActiveSense: Getting voxels\n");

    this->onlineModel2.workspaceTree->getVoxels(collisionResult->voxels,min[0],min[1],min[2],max[0],max[1],max[2]);
    this->onlineModel2.lastResult = collisionResult;
    this->demoOwner->createRender();
    demoOwner->context.debug("ActiveSense: Retrieved %d voxels\n",collisionResult->voxels.size());
    demoOwner->option("\x0D", "Press <Enter> to continue...");


    int index = 0;
    Real maxValue(golem::REAL_MIN);//minValue(golem::REAL_MAX);
    Real value(0.0);
    demoOwner->context.debug("ActiveSense: Starting selection\n");
    for (int i = 0; i < this->viewHypotheses.size(); i++)
    {
        if (this->viewHypotheses[i]->visited || hasViewed(this->viewHypotheses[i]))
            continue;

        value = onlineModel2.computeValue2(viewHypotheses[i], *collisionResult);
        viewHypotheses[i]->value = value;
        // TODO
        // check if it has trajectory


        //demoOwner->context.debug("ActiveSense: H[%d] Value: %f\n", i+1, value);

        if (value > maxValue)//if (value < minValue)
        {
            index = i;
            maxValue = value;//minValue = value;
        }
    }

    demoOwner->context.debug("\nActiveSense: Best View Information Gain Was H[%d] Value: %f\n\n", index+1, maxValue);//minValue);


    return this->getViewHypothesis(index);
}


bool pacman::ActiveSense::safetyExploration(){

    if(!this->result.trajectories.size()){
        demoOwner->getContext().debug("ActiveSense: There are no trajectories to perform safetyExploration. Aborting.\n");
        return false;
    }

    demoOwner->getContext().debug("ActiveSense: Performing safety exploration. There are %d trajectories!\n",this->result.trajectories.size());

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);
    grasp::data::Item::List scannedImageItems;

    grasp::data::Item::Map::iterator pointCurvPtr;

    data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());


    //golem::Controller::State::Seq completeTrajectory = demoOwner->completeTrajectory2(*trajectory);
    //data::ItemTrajectory::Ptr trajItem = demoOwner->convertToTrajectory(completeTrajectory);

    //data::Item::Map::iterator lastTraj = addItem("TempTrajectory",data::Item::Ptr(trajItem.get()));

    //grasp::Manipulator::Waypoint::Seq path = demoOwner->convertToManipulatorWayPoints(completeTrajectory);//demoOwner->convertToManipulatorWayPoints(controllerTraj);
    Collision::Result collisionResult;
    size_t eval_size = 50;


    pacman::HypothesisSensor::Seq visitedHypotheses;

    //    for (int i = 0; i < this->viewHypotheses.size(); i++)
    //        this->viewHypotheses[i]->visited = false;

    bool stop = false;
    golem::Real prev_entropy = golem::REAL_MAX;
    golem::Real curr_entropy = golem::REAL_MAX;
    golem::Real prev_gain = golem::REAL_MAX;
    data::Item::Ptr trajItem;
    golem::Controller::State::Seq completeTrajectory;
    data::Item::Map::iterator lastTraj = demoOwner->dataCurrentPtr->second->itemMap.end();
    while(!stop){


        CollisionBounds::Ptr collisionBoundsTraj = this->selectCollisionBounds(true, this->result.pointCurvs.back());
        completeTrajectory = demoOwner->completeTrajectory2(*trajectory);
        trajItem = demoOwner->convertToTrajectory(completeTrajectory);
        lastTraj = addItem("TempTrajectory",trajItem);
        grasp::Manipulator::Waypoint::Seq path = demoOwner->convertToManipulatorWayPoints(completeTrajectory);
        collisionBoundsTraj.release();

        golem::Real prob = onlineModel2.computeValue(path, collisionResult, eval_size);
        result.completeTrajectories.push_back(std::make_pair(trajItem,prob));

        curr_entropy = collisionResult.entropy/collisionResult.getEntropyCount();
        double inf_gain = prev_entropy - curr_entropy;
        double entropy = curr_entropy;
        printf("ActiveSense: probability of collision %lf ENTROPY %lf Information Gain %lf\n",prob, curr_entropy, prev_entropy - curr_entropy);
        prev_entropy = curr_entropy;
        demoOwner->createRender();

        //        if((prev_entropy-curr_entropy) >= 0.01/*params.entropyThr*/){
        //            prev_entropy = curr_entropy;

        for(int i = 0; i < viewHypotheses.size(); i++){
            viewHypotheses[i]->value = onlineModel2.computeValue2(viewHypotheses[i], collisionResult);
            demoOwner->getContext().debug("ActiveSense: H[%d] value %lf voxels\n",i,viewHypotheses[i]->value);
        }

        {
            golem::CriticalSectionWrapper lock(this->getCSViewHypotheses());
            std::sort(viewHypotheses.begin(), viewHypotheses.end(), pacman::HypothesisSensor::compareHypothesisSensor);
        }

        golem::Mat34 goal;
        goal.setId();
        CollisionBounds::Ptr collisionBounds = this->selectCollisionBounds(true, this->result.pointCurvs.back());
        bool gotToHypothesis = false;
        pacman::HypothesisSensor::Ptr hypothesis;
        int i = 0;
        while (!gotToHypothesis && i < viewHypotheses.size())
        {
            if(viewHypotheses[i]->visited){
                i++;
                continue;
            }
            hypothesis = viewHypotheses[i++];
            printf("ActiveSense safetyExploration: selected hypothesis %d with value %lf\n",i-1,hypothesis->value);
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

        if(gotToHypothesis){

            pacman::io_adhoc::log_out(this->out, "se_selected_view", this->experiment_id, this->experiment_trial, ActiveSense::S_INFORMATION_GAIN, false, hypothesis->id, -1, hypothesis->value, entropy, inf_gain, prob );

            //Adding hypotheses to the set of visited hypotheses so far (used for not going to this hypothesis again later)
            hypothesis->visited = true;
            visitedHypotheses.push_back(hypothesis);
            prev_gain = hypothesis->value;

            // Stops if the predicted information gain is below some threshold
            if( prev_gain < params.entropyThr )
                stop = true;


            demoOwner->scanPoseActive(scannedImageItems,hypothesis->getLabel(), "Image+ActiveSenseGraspDataImageNoCrop");
            //pointCurvPtr = processItems(scannedImageItems);

            this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
            onlineModel2.insertCloud(scannedImageItems.back());

            demoOwner->createRender();


        }
        //        }
        //        else{
        //            stop = true;
        //        }

        stop = stop? stop : visitedHypotheses.size() >= viewHypotheses.size();



    }

    golem::Real prob_collision = 1.0 - golem::Math::exp(collisionResult.eval);
    return prob_collision < 0.5;

    //    context.debug("Current Distance %lf\n", prob);

}

grasp::data::Trajectory* ActiveSense::getSafestTrajectoryToDate(){

    if(!this->result.trajectories.size())
        return NULL;

    grasp::data::Trajectory* safestToDate;
    safestToDate = NULL;

    Collision::Result collisionResult;
    size_t eval_size = 50;

    golem::Controller::State::Seq controller_traj;
    data::Item::Ptr trajItem;
    golem::Real safest_prob(1.0);
    golem::Real best_landmark = 1.0;

    struct Output {

        golem::Real prob;
        golem::Real landmark;
        grasp::data::Trajectory* traj;

    };
    std::vector< Output > outputs;

    for(auto it = this->result.trajectories.begin(); it != this->result.trajectories.end(); it++){
        grasp::data::Trajectory* trajectory = is<data::Trajectory>(*it);
        trajectory->createTrajectory(controller_traj);
        trajItem = demoOwner->convertToTrajectory(controller_traj);
        grasp::Manipulator::Waypoint::Seq path = demoOwner->convertToManipulatorWayPoints(controller_traj);
        this->demoOwner->createRender();
        golem::Real prob = onlineModel2.computeValue(path, collisionResult, eval_size);

        Output o;
        o.prob = prob;
        o.landmark = collisionResult.landmark;
        o.traj = trajectory;
        outputs.push_back(o);




    }
    
    for(Output& o : outputs){
        if(o.prob >= 0.90 && o.landmark >= 0.90)
            o.prob = 0.0;
    }

    std::sort(outputs.begin(), outputs.end(), [](const Output& o1, const Output& o2){ return o1.prob < o2.prob; });


    if( outputs.front().prob >= 0.95 ){
        demoOwner->context.debug("ActiveSense: no safe trajectory, choosing by best landmark prob is %lf\n",outputs.front().prob);
        std::sort(outputs.begin(), outputs.end(), [](const Output& o1, const Output& o2){ return o1.landmark > o2.landmark; });
    }

    safest_prob = outputs.front().prob;
    best_landmark = outputs.front().landmark;
    safestToDate = outputs.front().traj;

    demoOwner->context.debug("ActiveSense: safest trajectory to date has prob %lf landmark %lf\n",safest_prob, best_landmark);
    return safestToDate;

}


bool pacman::ActiveSense::safetyExploration2(data::Trajectory* trajectory , grasp::data::Item::List& scannedImageItems, std::vector<std::pair<grasp::data::Item::Ptr, float> >& bestTrajectories){

    if(!this->result.trajectories.size()){
        demoOwner->getContext().debug("ActiveSense: There are no trajectories to perform safetyExploration. Aborting.\n");
        return false;
    }


    std::vector<std::pair<grasp::data::Item::Ptr, float> > completeTrajectories;
    std::vector<float> traj_entropies;

    demoOwner->getContext().debug("ActiveSense: Performing safety exploration. There are %d trajectories!\n",this->result.trajectories.size());

    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    // Local scanned items for octree update
    grasp::data::Item::List localScannedImageItems;



    Collision::Result collisionResult;
    size_t eval_size = 50;


    pacman::HypothesisSensor::Seq visitedHypotheses;

    bool stop = false;
    golem::Real prev_entropy = golem::REAL_MAX;
    golem::Real curr_entropy = golem::REAL_MAX;
    golem::Real prev_gain = golem::REAL_MAX;
    data::Item::Ptr trajItem;
    golem::Controller::State::Seq completeTrajectory;
    data::Item::Map::iterator lastTraj;// = demoOwner->dataCurrentPtr->second->itemMap.end();

    while(!stop){


        // Completing trajectory
        CollisionBounds::Ptr collisionBoundsTraj = this->selectCollisionBounds(true, this->result.pointCurvs.back());
        //completeTrajectory = demoOwner->completeTrajectory2(*trajectory);
        completeTrajectory.clear();
        trajectory->createTrajectory(completeTrajectory);
        trajItem = demoOwner->convertToTrajectory(completeTrajectory);
        lastTraj = addItem("TempTrajectory",trajItem);
        grasp::Manipulator::Waypoint::Seq path = demoOwner->convertToManipulatorWayPoints(completeTrajectory);
        collisionBoundsTraj.release();
        this->demoOwner->createRender();
        // Computing probability of collision, entropy and other state metrics
        golem::Real prob = onlineModel2.computeValue(path, collisionResult, eval_size);


        curr_entropy = collisionResult.entropy/collisionResult.getEntropyCount();
        //if( curr_entropy <= prev_entropy ) {
        //    demoOwner->context.debug("ActiveSense: Selected safest trajectory is %d with collision probability %lf with entropy\n",idx, safest_prob, min_entropy);
        completeTrajectories.push_back(std::make_pair(trajItem,prob));

        //}

        double inf_gain = prev_entropy - curr_entropy;
        double entropy = curr_entropy;
        printf("ActiveSense: probability of collision %lf ENTROPY %lf Information Gain %lf\n",prob, curr_entropy, prev_entropy - curr_entropy);
        prev_entropy = curr_entropy;
        traj_entropies.push_back(curr_entropy);
        demoOwner->createRender();


        if( this->params.selectionMethod != S_RANDOM) {
            // Calculating prediction information gain for each view
            for(int i = 0; i < viewHypotheses.size(); i++){
                viewHypotheses[i]->value = onlineModel2.computeValue2(viewHypotheses[i], collisionResult);
                demoOwner->getContext().debug("ActiveSense: H[%d] value %lf voxels\n",viewHypotheses[i]->getId(),viewHypotheses[i]->value);
            }

            // Sorting views according to information gain value in decreasing order
            {
                golem::CriticalSectionWrapper lock(this->getCSViewHypotheses());
                std::sort(viewHypotheses.begin(), viewHypotheses.end(), pacman::HypothesisSensor::compareHypothesisSensor);
            }
        }

        // Selecting the next best, feasible and non-visited view
        golem::Mat34 goal;
        goal.setId();
        CollisionBounds::Ptr collisionBounds = this->selectCollisionBounds(true, this->result.pointCurvs.back());
        bool gotToHypothesis = false;
        pacman::HypothesisSensor::Ptr hypothesis;
        int i = 0;
        while (!gotToHypothesis && i < viewHypotheses.size())
        {
            hypothesis = this->params.selectionMethod != S_RANDOM? viewHypotheses[i++] : selectNextBestViewRandom();
            if(hypothesis->visited){
                continue;
            }

            printf("ActiveSense safetyExploration: selected hypothesis %d with value %lf\n",hypothesis->getId(),hypothesis->value);
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

        // If we got to the hypothesis...
        if(gotToHypothesis){
            this->safetyViews++;

            pacman::io_adhoc::log_out(this->out, "se_selected_view", this->experiment_id, this->experiment_trial, ActiveSense::S_INFORMATION_GAIN, false, hypothesis->id,collisionResult.landmark, hypothesis->value, entropy, inf_gain, prob );
            pacman::io_adhoc::log_out(this->out, collisionResult.landmark, collisionResult.collisionProfile);
            //Adding hypotheses to the set of visited hypotheses so far (used for not going to this hypothesis again later)
            hypothesis->visited = true;
            visitedHypotheses.push_back(hypothesis);
            prev_gain = hypothesis->value;

            // Stops if the predicted information gain for this hypothesis is below some threshold
            if(this->params.selectionMethod != S_RANDOM) {
                if( prev_gain < params.entropyThr )
                stop = true;
            }
            else {

                demoOwner->context.debug("ActiveSense: Safety Exploration View Count is %d.\n",this->safetyViews);
                stop = demoOwner->option("YN", "Wanna stop? (Y/N)?") == 'Y';
            }


            demoOwner->scanPoseActive(localScannedImageItems,hypothesis->getLabel(), "Image+ActiveSenseGraspDataImageNoCrop");

            // Integrate local point curv item (performing a new measurement for that)
            demoOwner->scanPoseActive(scannedImageItems, hypothesis->getLabel());
            this->pointCurvItem = processItems(scannedImageItems);

            // Inserting last local scanned item into model
            this->onlineModel2.setCurrentSensorPose(this->getCameraPose());
            onlineModel2.insertCloud(localScannedImageItems.back());

            // Inserting global integrated point curv
            this->onlineModel2.insertCloud(this->pointCurvItem);

            // Draw current state
            demoOwner->createRender();


        }

        stop = stop? stop : visitedHypotheses.size() >= viewHypotheses.size();



    }


    int idx = 0;
    float safest_prob = 1.0;
    float min_entropy = 99999999;
    int i = 0;
    //    for(auto it = completeTrajectories.begin(); it != completeTrajectories.end(); it++,i++)
    //    {
    //        if( /*it->second <= safest_prob &&*/ traj_entropies[i] <= min_entropy ){
    //            safest_prob = it->second;
    //            min_entropy = traj_entropies[i];
    //            idx = i;
    //        }
    //    }
    idx = completeTrajectories.size()-1;
    safest_prob = completeTrajectories[idx].second;
    min_entropy = traj_entropies[idx];
    bestTrajectories.push_back(completeTrajectories[idx]);
    demoOwner->context.debug("ActiveSense: Selected safest trajectory is %d with collision probability %lf with entropy\n",idx, safest_prob, min_entropy);



    golem::Real prob_collision = 1.0 - golem::Math::exp(collisionResult.eval);
    return prob_collision < 0.5;


}


golem::Mat34 pacman::ActiveSense::getCameraPose() {
    grasp::Camera* camera = this->getOwnerSensor(this->params.sensorId);

    grasp::ConfigMat34 cameraJointMountConfig;
    this->demoOwner->getPose(camera->getConfigJoint()-1, cameraJointMountConfig);

    golem::Mat34 cameraPose = cameraJointMountConfig.w*camera->getCurrentCalibration()->getParameters().pose;


    return cameraPose;

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
    state.cpos.set(config.c.data(), config.c.data() + std::min(config.c.size(), static_cast<size_t>(demoOwner->info.getJoints().size())));

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

pacman::HypothesisSensor::Ptr ActiveSense::generateSphericalView(const golem::Vec3& centroid, const golem::Real& radius, const golem::Real& phi, const golem::Real& theta){

    HypothesisSensor::Config sensorConfig;

    //Show pose lambda
    typedef std::function<void(const std::string&, const golem::Mat34& m)> ShowPoseFunc;
    ShowPoseFunc showPose = [&](const std::string& description, const golem::Mat34& m) {
        demoOwner->context.debug("ActiveSense: %s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
    };


    golem::Vec3 randomVec;
    golem::U8 r, g, b, a;

    //Generate random color
    r = static_cast<U8>(rand.nextUniform<float>(50, 255)), g = static_cast<U8>(rand.nextUniform<float>(50, 255)), b = static_cast<U8>(rand.nextUniform<float>(50, 255)), a = 255;


    //Generate random orientation vector
    //randomVec.x = rand.nextUniform<float>(-1, 1);
    //randomVec.y = rand.nextUniform<float>(-1, 1);
    //randomVec.z = rand.nextUniform<float>(-1, 1);

    const Real cos = Math::cos(theta);
    const Real sin = Math::sqrt(numeric_const<Real>::ONE - cos*cos);

    randomVec.set(sin * Math::cos(phi), sin * Math::sin(phi), cos);
    randomVec.setMagnitude(radius);




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
    r = static_cast<U8>(rand.nextUniform<float>(50, 255)), g = static_cast<U8>(rand.nextUniform<float>(50, 255)), b = static_cast<U8>(rand.nextUniform<float>(50, 255)), a = 255;


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
        const Real theta = rand.nextUniform<Real>(Math::degToRad<Real>(this->params.minTheta), Math::degToRad<Real>(this->params.maxTheta));
        const Real cos = Math::cos(theta);
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

void pacman::ActiveSense::generateSphericalSteppedViews(const golem::Real& nPhi, const golem::Real& nTheta)
{
    this->viewHypotheses.clear();

    golem::Real phiStep = (params.maxPhi - params.minPhi)/nPhi;
    golem::Real thetaStep = (params.maxPhi - params.minPhi)/nTheta;

    golem::Real cphi(0.0), ctheta(0.0);
    for(golem::Real phi = params.minPhi; phi <= params.maxPhi; golem::kahanSum(phi,cphi,phiStep)){
        for(golem::Real theta = params.minTheta; theta <= params.maxTheta; golem::kahanSum(theta,ctheta,thetaStep)){
            //this->viewHypotheses.push_back(this->generateSphericalView(this->params.centroid, this->params.radius, this->params.m));
            //generateSphericalView
        }
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

/************************ Active Sense Value Computation ***************************************/
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
            //if (++k < maxK) debugRenderer.addAxes3D(frame, golem::Vec3(0.01));



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
///////////////////////////////////////////////////////////////////////////////////////////////////

/************************ UTILS Player Items/Cloud Processing and Grasping ***********************/

grasp::data::Item::Map::iterator pacman::ActiveSense::processItems(grasp::data::Item::List& list)
{

    //Used transforms: PointsCurv+PointsCurv
    //PredictorModel+PredictorModel
    //PredictorQuery+PredictorQuery


    //PointCurv+PointCurv Transform
    grasp::data::Item::Map::iterator itemPtr = reduce(list, transformMap[POINT_CURV], ActiveSense::DFT_POINT_CURV_ITEM_LABEL);


    //Adding PointCurv to results
    this->result.pointCurvs.push_back(itemPtr);


    UI::removeCallback(*demoOwner, demoOwner->getCurrentHandler());

    grasp::data::Item::Ptr item = itemPtr->second;

    UI::addCallback(*demoOwner, &item->getHandler());

    return itemPtr;
}

grasp::data::Item::Map::iterator ActiveSense::convertToTrajectory(grasp::data::Item::Map::iterator contactQueryModel)
{
    grasp::Manager::RenderBlock renderBlock(*demoOwner);
    grasp::data::Convert* convert = grasp::is<grasp::data::Convert>(contactQueryModel);
    if (!convert)
        throw Message(Message::LEVEL_ERROR, "Input item does not support Convert interface");

    // find matching handlers
    typedef std::vector<data::Handler*> HandlerSet;
    HandlerSet handlerSet;
    //for (data::Handler::Map::const_iterator i = demoOwner->handlerMap.begin(); i != demoOwner->handlerMap.end(); ++i)
    //	if (std::find(convert->getHandlerTypes().begin(), convert->getHandlerTypes().end(), i->second->getType()) != convert->getHandlerTypes().end())
    //		handlerSet.push_back(i->second.get());


    // pick up handler
    data::Handler::Map::iterator handlerPtr = demoOwner->handlerMap.find(this->params.trajectoryHandler);
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

grasp::data::Item::Map::iterator ActiveSense::computeFeedBackTransform(grasp::data::Item::Map::iterator contactModelItem, grasp::data::Item::Map::iterator pointCurvItem)
{


    grasp::data::Item::List list;
    list.push_back(contactModelItem);
    list.push_back(pointCurvItem);
	grasp::data::Item::Map::iterator contactModelNew;

	try{

    demoOwner->context.debug("ActiveSense: Computing FeedBackQuery!\n");
    //Transforms contactModel and pointCurv into a predictorQuery item
    grasp::data::Item::Map::iterator contactQuery = reduce(list, transformMap[3], "FeedBack_contactQuery");
    this->result.predQueries.push_back(contactQuery);

    //Setting current contactQuery
    this->setcontactQueryItem(contactQuery);

     contactModelNew = computeContactModelFeedBack(contactQuery, pointCurvItem);
	
	}
	catch (const std::exception& m) {
		throw golem::Message("%s", m.what());
	}
    //list.push_back(contactModelItem);
    //this->removeItems(list);


    return contactModelNew;

}

grasp::data::Item::Map::iterator pacman::ActiveSense::computeContactModelFeedBack(grasp::data::Item::Map::iterator contactQuery, grasp::data::Item::Map::iterator pointCurvItem)
{

    demoOwner->context.debug("ActiveSense: Computing Trajectory!\n");


    grasp::data::Item::Map::iterator traj = convertToTrajectory(contactQuery);

    {
        Manager::RenderBlock renderBlock(*demoOwner);
        golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
        // add trajectory item into current data bundle (so can execute grasp again)
        std::string trajectoryName("ActiveSens_Grasp");
		grasp::data::Item::Map& itemMap = to<ActiveSenseDemo::Data>(demoOwner->dataCurrentPtr)->itemMap;
        itemMap.erase(trajectoryName);

        grasp::data::Item::Map::iterator ptr = itemMap.insert(itemMap.end(), grasp::data::Item::Map::value_type(trajectoryName, traj->second));
		ActiveSenseDemo::Data::View::setItem(itemMap, ptr, to<ActiveSenseDemo::Data>(demoOwner->dataCurrentPtr)->getView());
    }


    return computeTransformcontactModel(pointCurvItem, traj);
}

grasp::data::Item::Map::iterator pacman::ActiveSense::computeTransformcontactModel(grasp::data::Item::Map::iterator trajItem, grasp::data::Item::Map::iterator pointCurvItem)
{


    grasp::data::Item::List list;

    list.clear();
    //Feedback
    list.push_back(pointCurvItem);
    list.push_back(trajItem);


    demoOwner->context.debug("ActiveSense: Computing contactModelNew");

    grasp::data::Item::Map::iterator contactModelNew = reduce(list, transformMap[2], "contactModelNew");


    return contactModelNew;
}

grasp::CollisionBounds::Ptr pacman::ActiveSense::selectCollisionBounds(bool draw, grasp::data::Item::Map::const_iterator input) {
    CollisionBounds::Ptr collisionBounds;

    // select collision object
    demoOwner->context.debug("ActiveSense: Select collision object...\n");

    const data::Point3D* location = is<const data::Point3D>(input->second.get());


    if (location) {
        demoOwner->context.debug("ActivSense: Number of Locations %d\n", location->getNumOfPoints());
        // create collision bounds
        collisionBounds.reset(new CollisionBounds(*demoOwner->getPlanner().planner, [=](size_t i, golem::Vec3& p) -> bool {

            if (i < location->getNumOfPoints()) p = location->getPoint(i); return i < location->getNumOfPoints();
        }, draw ? &demoOwner->objectRenderer : nullptr, draw ? &demoOwner->scene.getCS() : nullptr));
        // draw locations
        golem::CriticalSectionWrapper csw(demoOwner->scene.getCS());
        for (size_t i = 0; i < location->getNumOfPoints(); ++i)
            demoOwner->objectRenderer.addPoint(location->getPoint(i), golem::RGBA::BLACK);
    }
    else
        demoOwner->context.debug("ActiveSense: Object collisions unsupported: UNSUPPORTED!!!!!!!!!!!!!!!!\n");


    return collisionBounds;
}

grasp::data::Item::Map::iterator ActiveSense::reduce(grasp::data::Item::List& list, TransformMap::value_type& transformPtr, const std::string& itemLabel) {

    demoOwner->context.debug("ActiveSense: ---[computeFeedBackTransform]: List Size: %d---\n", list.size());
    // transform
    // Modified 29/12/2015
    golem::shared_ptr<grasp::Manager::InputBlock> inputBlock;
    if (!this->allowInput) inputBlock = golem::shared_ptr<grasp::Manager::InputBlock>(new grasp::Manager::InputBlock(*demoOwner));
    data::Item::Map::iterator ptr;
    {
        //grasp::Manager::RenderBlock renderBlock(*demoOwner);

        //UI::addCallback(*demoOwner, transformPtr.first);

        grasp::data::Item::Ptr item;
        try{
            item = transformPtr.second->transform(list);
        }
        catch(const Message m){
            demoOwner->context.debug("%s",m.what());

        }

        {
            golem::CriticalSectionWrapper cswData(demoOwner->scene.getCS());
            ptr = to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.insert(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabel, item));
            grasp::Manager::Data::View::setItem(to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->itemMap, ptr, to<grasp::Manager::Data>(demoOwner->dataCurrentPtr)->getView());

        }

        UI::addCallback(*demoOwner, demoOwner->getCurrentHandler());
        demoOwner->createRender();
    }

    // Modified 29/12/2015
    inputBlock.release();
    return ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

void pacman::ActiveSense::executeTrajectory()
{
    demoOwner->context.debug("ActiveSense: Attempting to execute trajectory...\n");

    if (this->result.trajectories.size() <= 0)
        throw Cancel("ActiveSense: could not execute grasp because there's no trajectory!");

    data::Trajectory* trajectory = is<data::Trajectory>(this->result.trajectories.back());
    //trajectory->getWaypoints();

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

    collisionBounds.release();
}

void pacman::ActiveSense::executeTrajectory2()
{
    demoOwner->context.debug("ActiveSense: Attempting to execute trajectory version2...\n");

    if (this->result.completeTrajectories.size() <= 0)
        throw Cancel("ActiveSense: could not execute grasp because there's no trajectory!");

    int idx = 0;
    float safest_prob = 1.0;
    int i = 0;
    for(auto it = this->result.completeTrajectories.begin(); it != this->result.completeTrajectories.end(); it++,i++)
    {
        if( it->second <= safest_prob ){
            safest_prob = it->second;
            idx = i;
        }
    }

    demoOwner->context.debug("ActiveSense: Selected safest trajectory is %d with collision probability %lf\n",idx, safest_prob);

    data::Item::Ptr traj = this->result.completeTrajectories[idx].first;

    data::Trajectory* trajectory = is<data::Trajectory>(traj.get());
    // play
    Controller::State::Seq seq;
    trajectory->createTrajectory(seq);

    // select collision object
    CollisionBounds::Ptr collisionBounds = this->selectCollisionBounds(true, this->result.pointCurvs.back());
    demoOwner->context.debug("ActiveSense: Performing trajectory! \n");
    // perform
    this->demoOwner->perform2(this->demoOwner->dataCurrentPtr->first,"TmpTraj",seq);



    pLastExecutedWaypoint.reset(new Controller::State(seq.front()));
    // done!
    this->demoOwner->createRender();

    collisionBounds.release();
}

//-------------------------------------------------------------------------------------------------
void pacman::ActiveSenseController::initActiveSense(pacman::ActiveSenseDemo* demoOwner)
{
	this->demoOwner = demoOwner;
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








