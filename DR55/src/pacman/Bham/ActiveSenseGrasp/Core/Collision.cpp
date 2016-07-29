/** @file Collision.cpp
 *
 * Grasp collision model
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include "pacman/Bham/ActiveSenseGrasp/Core/Collision.h"
#include <Golem/Tools/XMLData.h>
#include "pacman/Bham/ActiveSenseGrasp/Demo/ActiveSenseGraspDemo.h"

// PCL
#include <flann/flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/highgui/highgui.hpp>

//------------------------------------------------------------------------------

using namespace golem;

namespace pacman {
//------------------------------------------------------------------------------

void Collision::Waypoint::load(const golem::XMLContext* xmlcontext) {
    golem::XMLData("path_dist", pathDist, const_cast<golem::XMLContext*>(xmlcontext), false);
    golem::XMLData("points", points, const_cast<golem::XMLContext*>(xmlcontext), false);
    golem::XMLData("depth_stddev", depthStdDev, const_cast<golem::XMLContext*>(xmlcontext), false);
    golem::XMLData("likelihood", likelihood, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void Collision::Desc::load(const golem::XMLContext* xmlcontext) {
    golem::XMLData(waypoints, waypoints.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "waypoint", false);
    XMLData(flannDesc, const_cast<golem::XMLContext*>(xmlcontext->getContextFirst("kdtree")), false);

    try {
        golem::XMLData(regionCaptureDesc, regionCaptureDesc.max_size(), xmlcontext->getContextFirst("region_capture"), "bounds", false);
        printf("Collision: loaded capture region sucessfully!\n");
    }
    catch (const MsgXMLParserNameNotFound&) {
        printf("Collision: couldn't load capture region!\n");
    }



}


//------------------------------------------------------------------------------

Collision::Collision(const grasp::Manipulator& manipulator, pacman::ActiveSenseDemo* demoOwner, const Desc& desc) : manipulator(manipulator), demoOwner(demoOwner), desc(desc) {
    desc.assertValid(grasp::Assert::Context("Collision()."));

    // joints - hand only
    for (golem::Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i)
        jointBounds[i].create(manipulator.getJointBounds(i));

    // base
    baseBounds.create(manipulator.getBaseBounds());

    for (golem::Bounds::Desc::Seq::const_iterator i = desc.regionCaptureDesc.begin(); i != desc.regionCaptureDesc.end(); ++i)
        regionCapture.push_back((*i)->create());


}

//void Collision::create(golem::Rand& rand, const grasp::data::Point3D& points) {
//    // points
//    this->points.clear();
//    this->points.reserve(points.size());
//    for (size_t i = 0, size = points.size(); i < size; ++i){

//        golem::Vec3 p(points.getPoint(i).x, points.getPoint(i).y, points.getPoint(i).z);
//        this->points.push_back(Feature(p));
//    }
//    //this->points.push_back(Bounds::Vec3((const grasp::data::Point3D::Vec3&)points.getPoint(i)));
//    std::random_shuffle(this->points.begin(), this->points.end(), rand);


//    // NN Search
//    flann::SearchParams search;
//    flann::KDTreeSingleIndexParams index;
//    search.checks = 32;
//    search.max_neighbors = 10;
//    desc.nnSearchDesc.getKDTreeSingleIndex(search, index);

//    typedef grasp::KDTree<golem::Real, Feature::FlannDist, flann::SearchParams> KDTree;
//    // Points must not be empty
//    nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));

//    for (golem::Bounds::Desc::Seq::const_iterator i = desc.regionCaptureDesc.begin(); i != desc.regionCaptureDesc.end(); ++i)
//        regionCapture.push_back((*i)->create());


//}

//// Should pass the octree here!
//void Collision::create(golem::Rand& rand, const active_sense::Model::Ptr& model) {

//    this->model = model;

//    for (golem::Bounds::Desc::Seq::const_iterator i = desc.regionCaptureDesc.begin(); i != desc.regionCaptureDesc.end(); ++i)
//        regionCapture.push_back((*i)->create());


//}

void Collision::calculateMetrics(size_t total, size_t free, size_t unknown, size_t collisions, golem::Real& pfree, golem::Real& pocc, golem::Real& punknown, golem::Real &entropy2){
    pfree =  Real(free)/total;
    pocc = Real(collisions)/total;
    punknown =  Real(unknown)/total;

    Real h1 = pfree*golem::Math::log2(pfree);
    Real h2 = pocc*golem::Math::log2(pocc);
    Real h3 = punknown*golem::Math::log2(punknown);

    h1 = h1==h1? h1 : 0;
    h2 =  h2==h2? h2 : 0;
    h3 =  h3==h3? h3 : 0;

    entropy2 = -(h1+h2+h3);
}

golem::Real Collision::evaluate(const grasp::Manipulator::Config& config, active_sense::Model::Voxel::Seq& voxels, golem::Real& entropy, size_t& collisions, size_t& free, size_t& unknown, size_t& total_eval, bool debug) {
    const golem::Mat34 base(config.frame.toMat34());
    golem::WorkspaceJointCoord joints;
    manipulator.getJointFrames(config.config, base, joints);

    if(!model.get()){
        manipulator.getContext().debug("NO MODEL SET!\n");
        return Real(0.0);
    }

    const size_t size = model->getOctree()->getNumLeafNodes();

    golem::Real eval = REAL_ZERO;
    //size_t collisions = 0, free = 0, unknown = 0;
    size_t total = 0;

    // joints - hand only
    for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
        Bounds& bounds = jointBounds[i];
        if (bounds.empty())
            continue;

        bounds.setPose(Bounds::Mat34(joints[i]));
        eval += bounds.evaluate(*this,this->model, voxels, entropy, collisions, free, unknown, total_eval);
    }

    // base
    if (!baseBounds.empty()) {
        baseBounds.setPose(Bounds::Mat34(base));
        eval += baseBounds.evaluate(*this,this->model, voxels, entropy, collisions, free, unknown, total_eval);
    }


    total = collisions + free + unknown;
    golem::Real pocc, pfree, punknown, entropy2;
    calculateMetrics(total, free, unknown, collisions, pocc, pfree, punknown, entropy2);

    //if (debug)
    //    manipulator.getContext().debug("Collision::evaluate(): points=%u, total=%u, collisions=%u free=%u unknowns=%u pocc=%lf pfree=%lf punknown=%lf eval=%lf likelyhood=%lf entropy=%lf entropy2=%lf \n",
    //                                   size, total, collisions, free, unknown, pocc, pfree, punknown, eval, 1.0-golem::Math::exp(eval), entropy, entropy2);

    return eval;
}

golem::Real Collision::evaluateProb(const grasp::Manipulator::Waypoint::Seq &path, int eval_size, Result& result, bool debug, float alpha){

    if(!model.get()){
        manipulator.getContext().debug("NO MODEL SET!\n");
        return golem::Real(0.0);
    }

    manipulator.getContext().debug("path length %d\n", path.size());
    golem::Real lo = path.front().getDistance();
    golem::Real hi = path.back().getDistance();
    golem::Real step = (hi-lo)/eval_size;

    golem::Real eval = REAL_ZERO;

    //size_t collisions = 0, free = 0, unknown = 0,
    //golem::Real entropy = 0.0;
    result.setToDefault();

    golem::Real c = REAL_ZERO;

    //Using kahanSum for minimising precision issues
    // Evaluates until before, and not at the last step of the trajectory
//    /(hi-step*4)
    bool found_landmark = false;
	golem::Real preveval = golem::REAL_ZERO, storedeval = golem::REAL_ZERO;
    for (golem::Real k = lo; k <= (hi-alpha*step); golem::kahanSum(k,c,step)){
        grasp::Manipulator::Config config = manipulator.interpolate(path, k);

        //demoOwner->getManipulatorAppearance().draw(manipulator, config, demoOwner->getDemoRenderer() );

        preveval=eval;
		try{
		manipulator.getContext().debug("Entering here!!!!\n");
        eval += evaluate(manipulator.interpolate(path, k), result.voxels, result.entropy, result.collisions, result.free, result.unknown, result.total_eval, debug);
		}
		catch (const std::exception& e){
			manipulator.getContext().debug("ERROR!!!: %s\n", e.what());
		}
        golem::Real pcollision = (golem::Real(1.0)-golem::Math::exp(eval));
        manipulator.getContext().debug("ProbCollision: %lf interp: %lf\n",pcollision, k/hi);

        result.collisionProfile.push_back(pcollision);

        if( !found_landmark && pcollision >= 0.95){
            result.landmark = k/hi;
            found_landmark = true;
            storedeval = preveval;

            //break;
        }


        //cv::waitKey(0);
    }
    if( found_landmark && result.landmark >= 0.91){
        eval = storedeval;
    }


    grasp::Manipulator::Config config = manipulator.interpolate(path, hi);
    //demoOwner->getManipulatorAppearance().draw(manipulator, config, demoOwner->getDemoRenderer() );
    manipulator.getContext().debug("ProbCollision: %lf interp: %lf\n",(golem::Real(1.0)-golem::Math::exp(eval)), 1.0);
    //cv::waitKey(0);

    result.eval = eval;
    manipulator.getContext().debug("ProbFree: %lf\n",golem::Math::exp(eval));
    eval = 1.0f - golem::Math::exp(eval);
    const golem::Real likelihood = eval;

    size_t total = result.collisions + result.free + result.unknown;
    golem::Real pocc, pfree, punknown, entropy2;
    calculateMetrics(total, result.free, result.unknown, result.collisions, pocc, pfree, punknown, entropy2);


    //if (debug)
    //    manipulator.getContext().debug("Collision::evaluate():  Expected Collision Prob=%lf entropy=%lf entropy2=%lf \n", likelihood, result.entropy, entropy2);

    return eval;
}

//------------------------------------------------------------------------------

void XMLData(Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create) {
    val.load(xmlcontext);
}

void XMLData(Collision::FlannDesc& val, golem::XMLContext* xmlcontext, bool create) {
    golem::XMLData("neighbours", val.neighbours, xmlcontext, create);
    golem::XMLData("points", val.points, xmlcontext, create);
    printf("READ KD NPOINTS %d\n", val.points);
    golem::XMLData("depth_stddev", val.depthStdDev, xmlcontext, create);
    golem::XMLData("likelihood", val.likelihood, xmlcontext, create);

    try {
        golem::XMLData("radius", val.points, xmlcontext, create);
    }
    catch (const Message&) {}
}


//------------------------------------------------------------------------------
} // namespace pacman
