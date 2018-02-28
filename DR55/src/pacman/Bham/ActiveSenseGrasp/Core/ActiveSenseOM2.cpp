/** @file ActiveSenseOM2.cpp
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
#include <Grasp/Data/ContactModel/ContactModel.h>

#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSenseOM2.h"
#include "pacman/Bham/ActiveSenseGrasp/Utils/PCLConversions.h"

#include "pacman/Bham/ActiveSenseGrasp/UI/UI.h"

#include "pacman/Bham/ActiveSenseGrasp/IO/IO_Adhoc.h"



namespace pacman {


void ActiveSensOnlineModel2::init(const Collision::Ptr& collision, const grasp::Manipulator::Ptr& manipulator){
    this->collision = collision;
    this->manipulator = manipulator;

}

void ActiveSensOnlineModel2::setCurrentSensorPose(const golem::Mat34& sensorPose) {
    golem::Vec3 p = sensorPose.p;
    golem::Vec3 col1, col2, col3;

    sensorPose.R.getColumn(0,col1);
    sensorPose.R.getColumn(1,col2);
    sensorPose.R.getColumn(2,col3);

    currSensorPose(0,0) = col1.v1; currSensorPose(0,1) = col2.v1; currSensorPose(0,2) = col3.v1; currSensorPose(0,3) = p.v1;
    currSensorPose(1,0) = col1.v2; currSensorPose(1,1) = col2.v2; currSensorPose(1,2) = col3.v2; currSensorPose(1,3) = p.v2;
    currSensorPose(2,0) = col1.v3; currSensorPose(2,1) = col2.v3; currSensorPose(2,2) = col3.v3; currSensorPose(2,3) = p.v3;
    currSensorPose(3,0) = 0;       currSensorPose(3,1) = 0;       currSensorPose(3,2) = 0;       currSensorPose(3,3) = 1;
}



void ActiveSensOnlineModel2::insertContacts(grasp::data::Item::Map::iterator contactModelPtr){

    //Uncomment if you want to consider only the current observed contacts instead of the whole history
    //trainingData.clear();

    // collect data (TODO: Receive a list of ItemContactModel
    const grasp::data::ItemContactModel* contactModel = grasp::is<const grasp::data::ItemContactModel>(contactModelPtr);
    if (contactModel)
    {
        trainingData.push_back(&contactModel->dataMap);
        insertContacts(contactModel->dataMap);
    }
    else
    {
        printf("ActiveSense: This is not an ItemContactModel\n");
    }

}

void ActiveSensOnlineModel2::insertContacts(const grasp::data::ItemContactModel::Data::Map& contactMap){

    //For each graspType's mapping of joint->contacts do...
    for (grasp::data::ItemContactModel::Data::Map::const_iterator j = contactMap.begin(); j != contactMap.end(); ++j) {

        //j->first => GraspType
        //j->second.contacts => Map between joints x contact3D
        //where contact3D is contains (point,orientation,frame,weight)
        //For each joint's sequence of contact3D regardless of the joint associated with it, do...
        for (grasp::Contact3D::Map::const_iterator k = j->second.contacts.begin(); k != j->second.contacts.end(); k++)
        {
            // k->first => jointToString()
            // k->second => contactSequence (Contact3DSeq)
            grasp::Manipulator::Link jointLink;
            jointLink.fromString(k->first);
            updateContacts(k->second, jointLink.getID());
        }

    }

}

/**
    Sets current point cloud
    */
void ActiveSensOnlineModel2::updateContacts(const grasp::Contact3D::Seq& graspContacts, const int& jointId){
    //this->graspContacts.insert(this->graspContacts.end(), graspContacts.begin(), graspContacts.end());
    active_sense::PointCloudNormal::Ptr contact_cloud(new active_sense::PointCloudNormal());
    std::vector<float> contact_weights;

    utils::convert(graspContacts, *contact_cloud, contact_weights);

    float cdf = 0.0f;
    for(float w : contact_weights){
        cdf += w;
    }
    // adding pre-normalised weights
    for(float& w : contact_weights){
        w /= cdf;
    }

    contactTree->insertScan(currSensorPose, contact_cloud, contact_weights,jointId);
    workspaceTree->insertScan(currSensorPose, contact_cloud, contact_weights,jointId);
}

void ActiveSensOnlineModel2::insertCloud(grasp::data::Item::Map::const_iterator itemPtr){

    printf("ActiveSensOnlineModel2: Inserting cloud!\n");
    grasp::data::ItemPointsCurv* image = grasp::is<grasp::data::ItemPointsCurv>(itemPtr);
    grasp::data::ItemImage* image2 = grasp::is<grasp::data::ItemImage>(itemPtr);


    active_sense::PointCloudNormal::Ptr cloud_acs(new active_sense::PointCloudNormal());
    if( image ){

        utils::convert(*image->cloud, *cloud_acs);
        printf("ActiveSensOnlineModel2: Inserting pointcurv!\n");

    }else if( image2 ){

        utils::convert(*image2->cloud, *cloud_acs);
        printf("ActiveSensOnlineModel2: Inserting PointNormal!\n");


    }

    if(cloud_acs->size()){
        //printf("ActiveSensOnlineModel2::insertCloud: Inserting %d POINTS!!!!!\n",cloud_acs->size());
        workspaceTree->insertScan(currSensorPose, cloud_acs);
    }
    else{
        printf("ActiveSensOnlineModel2::insertCloud: No points to insert!!!!!\n");
    }








}

golem::Real ActiveSensOnlineModel2::computeValue(HypothesisSensor::Ptr hypothesis, Collision::Result& result){


    std::vector<bool> clip_mask;
    std::vector<cv::Mat> transformed_points;


    golem::Mat34 eyePose = hypothesis->getFrame();
    golem::Mat34 extMat; extMat.setInverse(eyePose);

    camera_model.loadRotationMatrix(extMat.R);
    camera_model.loadTranslation(extMat.p);


    camera_model.transform(result.voxels,transformed_points,clip_mask, true);

    golem::Real val = golem::REAL_ZERO;
    for(int i = 0; i < clip_mask.size(); i++){

        val += static_cast<int>(clip_mask[i]);

    }

    return val;

}

// information gain
golem::Real ActiveSensOnlineModel2::computeValue2(HypothesisSensor::Ptr hypothesis, Collision::Result& result){


    std::vector<bool> clip_mask;
    std::vector<cv::Mat> transformed_points;


    golem::Mat34 eyePose = hypothesis->getFrame();
    golem::Mat34 extMat; extMat.setInverse(eyePose);

    camera_model.loadRotationMatrix(extMat.R);
    camera_model.loadTranslation(extMat.p);


    camera_model.transform(result.voxels,transformed_points,clip_mask, true);

    golem::Real val = golem::REAL_ZERO;
    int count = 0;
    for(int i = 0; i < clip_mask.size(); i++){

        if(clip_mask[i]) {
            float gain = 0.0f;
            if( result.voxels[i].state == active_sense::Model::Voxel::OCC ){
                gain = result.voxels[i].predict_gain(workspaceTree->params.prob_hit_);
            }
            else if( result.voxels[i].state == active_sense::Model::Voxel::FREE ){
                gain = result.voxels[i].predict_gain(workspaceTree->params.prob_miss_);
            }
            else {
                float opt1 = result.voxels[i].predict_gain(workspaceTree->params.prob_hit_);
                float opt2 = result.voxels[i].predict_gain(workspaceTree->params.prob_miss_);
                gain = 0.5*(opt1+opt2);
            }
            //ignoring nan
            if( gain == gain )
                val += gain;


        }
        count += static_cast<int>(clip_mask[i]);

    }

    //printf("Value %f count %d\n", val, count );

    return count > 0? val/count : 0;

}

//golem::Real ActiveSensOnlineModel2::computeValue(const grasp::Manipulator::Waypoint::Seq& path, int eval_size) {
//    collision->setModel(this->workspaceTree);
//    Collision::Result result;
//    float expected_collision_prob = collision->evaluateProb(path, eval_size, result, true);

//    this->manipulator->getContext().debug("Voxel list size: %u",result.voxels.size() );

//    return expected_collision_prob;
//}

golem::Real ActiveSensOnlineModel2::computeValue(const grasp::Manipulator::Waypoint::Seq& path, Collision::Result& result, int eval_size) {
    collision->setModel(this->workspaceTree);
    result.setToDefault();

    float expected_collision_prob = collision->evaluateProb(path, eval_size, result, true);
    lastResult = result.makeShared();
    //this->manipulator->getContext().debug("Voxel list size: %u",result.voxels.size() );

    return expected_collision_prob;
}


void ActiveSensOnlineModel2::getLinkNormals(const grasp::Manipulator::Waypoint::Seq& path, std::map<int, golem::Vec3>& linkNormals){

    golem::WorkspaceJointCoord jointPoses;
    golem::Real hi = path.back().getDistance();
    grasp::Manipulator::Config config = manipulator->interpolate(path, hi);
    const golem::Mat34 base(config.frame.toMat34());

    manipulator->getJointFrames(config.config, base, jointPoses);
    for (golem::Configspace::Index i = manipulator->getHandInfo().getJoints().begin(); i < manipulator->getHandInfo().getJoints().end(); ++i) {
        const grasp::Manipulator::Link jointLink(grasp::Manipulator::Link::TYPE_JOINT, (golem::U32)*i);

        golem::Mat34 linkFrame = golem::Mat34(jointPoses[i]);

        // Negating the z to get the normal
        golem::Vec3 normal = linkFrame.R * golem::Vec3(golem::REAL_ZERO, golem::REAL_ZERO, -golem::REAL_ONE);
        linkNormals.insert(linkNormals.end(),std::make_pair(jointLink.getID(),normal));
    }


}



golem::Real ActiveSensOnlineModel2::computeValue(HypothesisSensor::Ptr hypothesis){


    // Returns a value that is either equal to the previous value for a given voxel, or smaller.
    // This value represents the viewing angle of the voxel from the given hypothesis sensor
    // So we are going to either reduce the viewing angle or leave it as we've seen before
    active_sense::Model::ReduceFunc valueFunc = [&](const octomap::AngleOcTreeNode& node) {

        golem::Real weight = node.getContactWeight();
        golem::Vec3 contactNormal(node.getNormal().nx_, node.getNormal().ny_, node.getNormal().nz_);
        golem::Vec3 viewDir(0.0, 0.0, 0.0);
        hypothesis->getFrame().R.getColumn(2, viewDir); //viewDir of hypothesis sensor

        contactNormal.normalise();
        viewDir.normalise();



        //golem::Real theta = golem::Math::acos(contactNormal.dot(-viewDir));//golem::REAL_PI - golem::Math::acos(contactNormal.dot(viewDir));

        golem::Real sigma = contactTree->params.angleUpdateFunc(node.getAngle(),
                                                                contactNormal.x, contactNormal.y, contactNormal.z,
                                                                viewDir.x, viewDir.y, viewDir.z);

        float value = weight*sigma;


        return value;
    };

    golem::Real value = contactTree->computeReduce(valueFunc);


    return value;
}

golem::Real ActiveSensOnlineModel2::computeValueWithHand(HypothesisSensor::Ptr hypothesis, const grasp::Manipulator::Waypoint::Seq& path){


    std::map<int, golem::Vec3> linkNormals;
    this->getLinkNormals(path,linkNormals);

    // Returns a value that is either equal to the previous value for a given voxel, or smaller.
    // This value represents the viewing angle of the voxel from the given hypothesis sensor
    // So we are going to either reduce the viewing angle or leave it as we've seen before
    active_sense::Model::ReduceFunc valueFunc = [&](const octomap::AngleOcTreeNode& node) {

        golem::Real weight = node.getContactWeight();
        golem::Vec3 contactNormal(node.getNormal().nx_, node.getNormal().ny_, node.getNormal().nz_);
        golem::Vec3 negContactNormal(-node.getNormal().nx_, -node.getNormal().ny_, -node.getNormal().nz_);
        golem::Vec3 viewDir(0.0, 0.0, 0.0);
        hypothesis->getFrame().R.getColumn(2, viewDir); //viewDir of hypothesis sensor
        int jointId = node.getJointId();

        contactNormal.normalise();
        viewDir.normalise();
        negContactNormal.normalise();



        //golem::Real theta = golem::Math::acos(contactNormal.dot(-viewDir));//golem::REAL_PI - golem::Math::acos(contactNormal.dot(viewDir));

        golem::Real sigma = contactTree->params.angleUpdateFunc(node.getAngle(),
                                                                contactNormal.x, contactNormal.y, contactNormal.z,
                                                                viewDir.x, viewDir.y, viewDir.z);

        float value = 0.0;

        const golem::Vec3& normal = linkNormals[jointId];
        int indicator1 = normal.dot(contactNormal) <= 0;
        int indicator2 = normal.dot(negContactNormal) <= 0;

        //TODO: Jeremy's sigma: sigma = acos(min(0,n'*v));
        //printf("JointId %d\n", jointId);
        value += weight*sigma*(indicator1+indicator2);

        return value;
    };

    golem::Real value = contactTree->computeReduce(valueFunc);


    return value;
}




void ActiveSensOnlineModel2::drawVoxels(golem::DebugRenderer &renderer, const golem::Scene &scene, const active_sense::Model::Voxel::Seq& voxels, bool is_free){

    if(!voxels.size())
        return;

    golem::CriticalSectionWrapper cswData(scene.getCS());

    for(auto voxel : voxels){
        boundDesc.dimensions.set(voxel.size,voxel.size,voxel.size);
        golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));

        golem::BoundingBox::Ptr box = descBox->create();
        golem::Mat34 pose = box->getPose();
        //manipulator->getContext().debug("Drawing voxel at %f %f %f\n",voxel.point.x(),voxel.point.y(),voxel.point.z());
        pose.p = golem::Vec3(voxel.point.x(),voxel.point.y(),voxel.point.z());
        //manipulator->getContext().debug("Voxel Color %f\n",voxel.angle);

        box->setPose(pose);

        golem::RGBA color = utils::heightMapColor(voxel.angle);//(pose.p.z);
        if( !is_free ){
            color._rgba.a = voxel.is_contact? 255 : 50;
        }
        else{
            color = golem::RGBA(222,222,222,30);
        }
        renderer.setColour(color);//golem::RGBA(125, 100, 100, 32)
        renderer.addSolid(*box);
        renderer.setColour(golem::RGBA(0, 0, 0, 100));
        //        renderer.addWire(*box);
//        golem::Vec3 p2;
//        golem::Vec3 normal(voxel.normal.x(),voxel.normal.y(), voxel.normal.z());
//        normal.multiply(0.05,normal);
//        p2.add(pose.p,normal);
//        renderer.addLine(pose.p, p2, golem::RGBA(0, 0, 0, 100));


    }
}

void ActiveSensOnlineModel2::drawWorkpaceTree(golem::DebugRenderer &renderer, const golem::Scene &scene){

    //manipulator->getContext().debug("Generating Data!",this->workspaceTree->getOctree()->getNumLeafNodes());
    //io_adhoc::add_testData(*this->workspaceTree->getOctree());
    //manipulator->getContext().debug("Got Number of Leafs is %d",this->workspaceTree->getOctree()->getNumLeafNodes());

    //manipulator->getContext().debug("ClampingMax %lf ClampingMin %.10lf\n",this->workspaceTree->getOctree()->getClampingThresMax(), this->workspaceTree->getOctree()->getClampingThresMin());

    active_sense::Model::Voxel::Seq voxels_occ, voxels_free;
    this->workspaceTree->getVoxels(voxels_occ, active_sense::Model::Voxel::OCC);
    if( showFreeSpace )
        this->workspaceTree->getVoxels(voxels_free, active_sense::Model::Voxel::FREE, this->workspaceTree->params.tree_max_depth_-1);
    //manipulator->getContext().debug("Got %d Occupied Voxels",voxels.size());
    //    if(!voxels_occ.size()){
    //        manipulator->getContext().debug("WorkpaceTree: Nothing to draw\n");
    //        return;
    //    }
    drawVoxels(renderer, scene, voxels_occ);

    if( showFreeSpace )
        drawVoxels(renderer, scene, voxels_free, true);


}

void ActiveSensOnlineModel2::drawContactTree(golem::DebugRenderer &renderer, const golem::Scene &scene){
    //manipulator->getContext().debug("Generating Data!",this->workspaceTree->getOctree()->getNumLeafNodes());
    //io_adhoc::add_testData(*this->workspaceTree->getOctree());

    active_sense::Model::Voxel::Seq voxels;
    this->contactTree->getVoxels(voxels, active_sense::Model::Voxel::OCC);
    //manipulator->getContext().debug("Got %d Occupied Voxels",voxels.size());

    drawVoxels(renderer, scene, voxels);

}

void ActiveSensOnlineModel2::drawBox(golem::DebugRenderer &renderer,  const golem::Scene &scene,  const golem::Vec3& p, const golem::Bounds::Ptr& box, bool visible){

    golem::Mat34 pose = box->getPose();
    pose.p = p;


    box->setPose(pose);
    if(visible) {
        renderer.setColour(golem::RGBA(125, 255, 100, 200));
    }
    else{
        renderer.setColour(golem::RGBA(125, 222, 222, 32));
    }
    renderer.addSolid(*box);
    renderer.setColour(golem::RGBA(0, 0, 0, 50));
    renderer.addWire(*box);

}

void ActiveSensOnlineModel2::drawTrajectory(golem::DebugRenderer &renderer, const golem::Scene &scene) { //,  const grasp::Manipulator::Waypoint::Seq& path

    if(!lastResult.get())
        return;

    //if(!lastResult->changed)
    //    return;

    //lastResult->changed = false;

    //Collision::Result collisionResult;
    //size_t eval_size = 50;
    //golem::Real prob = computeValue(path,collisionResult, eval_size);
    active_sense::Model::Voxel::Seq& voxels = lastResult->voxels;
    //context.debug("Probability of collision %lf\n", prob);




    //Bounds description
    golem::BoundingBox::Desc boundDesc;
    boundDesc.dimensions.set(0.01, 0.01, 0.01);
    boundDesc.pose.setId();
    //boundDesc.pose.p.z -= 0.025;rm

    {
        golem::CriticalSectionWrapper cswData(scene.getCS());
        for(auto vt = voxels.begin(); vt != voxels.end(); vt++){


            boundDesc.dimensions.set(vt->size,vt->size,vt->size);

            golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));

            drawBox(renderer, scene, golem::Vec3(vt->point.x(),vt->point.y(),vt->point.z()),descBox->create(), vt->is_visible);

        }
    }




}


void ActiveSensOnlineModel2::draw(golem::DebugRenderer &renderer, const golem::Scene& scene){

    if( showWorkpaceTree)
        drawWorkpaceTree(renderer, scene);

    if( showContactTree )
        drawContactTree(renderer, scene);

    if( showLastResult )
        drawTrajectory(renderer, scene);

}



};
