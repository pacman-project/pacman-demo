/** @file ActiveSenseOM2.h
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#pragma once
#ifndef _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM2_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM2_H_


#include <Grasp/Data/ContactModel/ContactModel.h>
#include <Grasp/Contact/Manipulator.h>

#include "pacman/Bham/ActiveSenseGrasp/Core/HypothesisSensor.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/Collision.h"
#include "ActiveSense/Core/Model.h"
#include <list>
#include <functional>

/** PaCMan name space */
namespace pacman {



class ActiveSensOnlineModel2 {

public:

    typedef std::vector<const grasp::data::ItemContactModel::Data::Map*> TrainingData;

    pacman::Collision::Ptr collision;
    grasp::Manipulator::Ptr manipulator;

    grasp::Contact3D::Seq graspContacts;
    std::vector<golem::Real> viewingAngles;

    TrainingData trainingData;
    active_sense::Model::Ptr contactTree;
    active_sense::Model::Ptr workspaceTree;

    Eigen::Matrix4f currSensorPose;

    golem::BoundingBox::Desc boundDesc;

    PinholeCamera camera_model;

    pacman::Collision::Result::Ptr lastResult;

    bool showLastResult, showWorkpaceTree, showFreeSpace, showContactTree;
    float resWorkspacetree, resContactTree;

    ActiveSensOnlineModel2(){

        showContactTree = true;
        showLastResult = false;
        showFreeSpace = showWorkpaceTree = false;
        setToDefault();

    }
    void resetContactTree(){

        if( contactTree.get() && workspaceTree.get()){
            active_sense::Model::MapFunc func = [](oct::AngleOcTreeNode& node){

                if( node.isContact() ){
                    node.setContact(false);
                    node.setContactWeight(0);
                }

            };

            workspaceTree->computeMap(func);
        }



        resContactTree = 0.0005;
        contactTree = active_sense::Model::Ptr(new active_sense::Model(resContactTree));
        // Setting very low clamping
        contactTree->params.thres_min_ = 0.00000001;
        //contactTree->params.thres_max_ = 0.99999999;
        contactTree->params.prob_hit_ = 0.9;
        contactTree->params.prob_miss_ = 0.1;
        contactTree->updateParameters();
    }

    void resetWorkspaceTree() {
        resWorkspacetree = 0.0025;
        workspaceTree = active_sense::Model::Ptr(new active_sense::Model(resWorkspacetree));
        workspaceTree->params.thres_min_ = 0.00000001;
        //workspaceTree->params.thres_max_ = 0.99999999;
        workspaceTree->params.prob_hit_ = 0.999;
        workspaceTree->params.prob_miss_ = 0.001;
        workspaceTree->updateParameters();
    }

    void setToDefault(){


        lastResult.release();
        lastResult = pacman::Collision::Result::Ptr();


        resetContactTree();
        resetWorkspaceTree();

        // aspect ratio = 640/480, near 0.8 m, far 3.5 m (primesense spec)
        //before 0.25
        camera_model.setFrustum(45,1.333333f,0.4,3.5);//before 0.8, 4.5



//        //Bounds description
        boundDesc.dimensions.set(resWorkspacetree, resWorkspacetree, resWorkspacetree);
        boundDesc.pose.setId();
//        golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));
//        box = descBox->create();



    }

    void init(const pacman::Collision::Ptr& collision, const grasp::Manipulator::Ptr& manipulator);

    void setCurrentSensorPose(const golem::Mat34& sensorPose);

    void insertContacts(const grasp::data::ItemContactModel::Data::Map& contactMap);

    void insertCloud(grasp::data::Item::Map::const_iterator itemPtr);


    /**
        Sets current point cloud
    */
    void updateContacts(const grasp::Contact3D::Seq& graspContacts, const int& jointId = -1);

    void insertContacts(grasp::data::Item::Map::iterator contactModelPtr);

    /**
        Computes value for a given hypothesis
    */
    golem::Real computeValue(HypothesisSensor::Ptr hypothesis);
    golem::Real computeValueWithHand(HypothesisSensor::Ptr hypothesis, const grasp::Manipulator::Waypoint::Seq& path);

    //golem::Real computeValue(const grasp::Manipulator::Waypoint::Seq& path, int eval_size = 50);
    golem::Real computeValue(const grasp::Manipulator::Waypoint::Seq& path, Collision::Result& result, int eval_size = 50);

    void getLinkNormals(const grasp::Manipulator::Waypoint::Seq& path, std::map<int, golem::Vec3>& linkFrames);

    // TODO:
    // Option1: Counts how many unknown voxels fall in the frustum of the HypothesisSensor
    // Option2: Performs imaginary update on voxels, computes the resultant imaginary entropy, and returns the information gain accoridng to current entropy level
    golem::Real computeValue(HypothesisSensor::Ptr hypothesis, Collision::Result& result);
    golem::Real computeValue2(HypothesisSensor::Ptr hypothesis, Collision::Result& result);



    void draw(golem::DebugRenderer& renderer, const golem::Scene& scene);
    void drawContactTree(golem::DebugRenderer& renderer, const golem::Scene& scene);
    void drawWorkpaceTree(golem::DebugRenderer& renderer, const golem::Scene& scene);
    void drawVoxels(golem::DebugRenderer& renderer, const golem::Scene& scene, const active_sense::Model::Voxel::Seq& voxels, bool is_free=false);

    void drawBox(golem::DebugRenderer &renderer,  const golem::Scene &scene,  const golem::Vec3& p, const golem::Bounds::Ptr& box, bool visible);
    void drawTrajectory(golem::DebugRenderer &renderer, const golem::Scene &scene); //



};


};

#endif // _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM2_H_
