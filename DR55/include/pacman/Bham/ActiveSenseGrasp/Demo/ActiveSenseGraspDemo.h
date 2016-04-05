/** @file ActiveSenseGraspDemo.h
 *
 * ActiveSenseDemo Grasp
 *
 * @author	Ermano Arruda
 *
 *
 */

#pragma once
#ifndef _ACTIVE_SENSE_GRASP_DEMO_GRASP_H_ // if #pragma once is not supported
#define _ACTIVE_SENSE_GRASP_DEMO_GRASP_H_

#include "pacman/Bham/Demo/BaseDemo.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSense.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/Collision.h"

#include <Grasp/Contact/Manipulator.h>

#include <Grasp/Data/Trajectory/Trajectory.h>

/** Pacman name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Grasp demo. */
class ActiveSenseDemo : public pacman::BaseDemoDR55, public pacman::ActiveSenseController {
public:
    friend class ActiveSense;

    /** Grasp demo description */
    class Desc : public pacman::BaseDemoDR55::Desc {
    public:
        typedef golem::shared_ptr<Desc> Ptr;

        /** Active Sense Attribute */
        ActiveSense::Ptr activeSense;
        pacman::Collision::Desc::Ptr collisionDesc;

        ///////// Manipulator //////////////

        /** Manipulator description */
        grasp::Manipulator::Desc::Ptr manipulatorDesc;
        /** Manipulator Appearance */
        grasp::Manipulator::Appearance manipulatorAppearance;
        /** Manipulator pose distribution standard deviation */
        grasp::RBDist manipulatorPoseStdDev;
        /** Maximum distance between frames in standard deviations */
        golem::Real manipulatorPoseStdDevMax;
        /** Manipulator trajectory item */
        std::string manipulatorItemTrj;

        /** Manipulation trajectory duration */
        golem::SecTmReal manipulatorTrajectoryDuration;
        /** Manipulation trajectory force threshold */
        golem::Twist trajectoryThresholdForce;

        /* Withdraw action hand release fraction */
        golem::Real withdrawReleaseFraction;
        /* Withdraw action lift distance */
        golem::Real withdrawLiftDistance;
        ///////// End Manipulator //////////////

        /** Constructs from description object */
        Desc() {
            Desc::setToDefault();
        }
        /** Sets the parameters to the default values */
        virtual void setToDefault() {
            pacman::BaseDemoDR55::Desc::setToDefault();

            this->activeSense = ActiveSense::Ptr(new ActiveSense());

            collisionDesc.reset(new pacman::Collision::Desc());

            ///////// Manipulator //////////////
            manipulatorDesc.reset(new grasp::Manipulator::Desc);
            manipulatorAppearance.setToDefault();
            manipulatorPoseStdDev.set(golem::Real(0.002), golem::Real(1000.0));
            manipulatorPoseStdDevMax = golem::Real(5.0);
            manipulatorItemTrj.clear();

            manipulatorTrajectoryDuration = golem::SecTmReal(5.0);
            trajectoryThresholdForce.setZero();
            withdrawReleaseFraction = golem::Real(0.5);
            withdrawLiftDistance = golem::Real(0.20);
            ///////// End Manipulator //////////////
        }
        /** Assert that the description is valid. */
        virtual void assertValid(const grasp::Assert::Context& ac) const {
            pacman::BaseDemoDR55::Desc::assertValid(ac);


            this->activeSense->assertValid(ac);
            this->collisionDesc->assertValid(ac);

            ////////// Manipulator ///////////////////////
            grasp::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
            manipulatorDesc->assertValid(grasp::Assert::Context(ac, "manipulatorDesc->"));
            manipulatorAppearance.assertValid(grasp::Assert::Context(ac, "manipulatorAppearance."));
            grasp::Assert::valid(manipulatorPoseStdDev.isValid(), ac, "manipulatorPoseStdDev: invalid");
            grasp::Assert::valid(manipulatorPoseStdDevMax > golem::REAL_EPS, ac, "manipulatorPoseStdDevMax: < eps");
            grasp::Assert::valid(manipulatorItemTrj.length() > 0, ac, "manipulatorItemTrj: invalid");

            grasp::Assert::valid(manipulatorTrajectoryDuration > golem::SEC_TM_REAL_ZERO, ac, "manipulatorTrajectoryDuration: <= 0");
            grasp::Assert::valid(trajectoryThresholdForce.isPositive(), ac, "trajectoryThresholdForce: negative");

            grasp::Assert::valid(withdrawReleaseFraction >= golem::REAL_ZERO, ac, "withdrawReleaseFraction: < 0");
            grasp::Assert::valid(withdrawReleaseFraction <= golem::REAL_ONE,  ac, "withdrawReleaseFraction: > 1");
            grasp::Assert::valid(withdrawLiftDistance >= golem::REAL_ZERO, ac, "withdrawLiftDistance: < 0");

            ///////// End Manipulator //////////////
        }
        /** Load descritpion from xml context. */
        virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

    protected:
        GRASP_CREATE_FROM_OBJECT_DESC1(ActiveSenseDemo, golem::Object::Ptr, golem::Scene&)
    };

    golem::Controller::State lookupState() const;
    golem::Controller::State lookupCommand() const;

    void showRecordingState();

    void setHandConfig(golem::Controller::State::Seq& trajectory, const golem::Controller::State cmdHand);

	void gotoPose3(const grasp::ConfigMat34& pose, const golem::SecTmReal duration = golem::SEC_TM_REAL_ZERO, const bool ignoreHand = false);

	void releaseRightHand(const double openFraction, const golem::SecTmReal duration);
	void closeRightHand(const double closeFraction, const golem::SecTmReal duration);
    void releaseRightHand2(const double openFraction, const golem::SecTmReal duration, const golem::Controller::State partReleaseConfig);

    void executeDropOff();
    virtual void setMenus();

	void graspWithActiveSense(bool stopAtBreakPoint = false);

    virtual bool gotoPoseWS(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
	virtual bool gotoPoseWS2(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
    virtual bool gotoPoseConfig(const grasp::ConfigMat34& config, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
    virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL, const std::string& handler = std::string(), ScanPoseCommand scanPoseCommand = nullptr, grasp::Manager::Data::Ptr dataPtr = grasp::Manager::Data::Ptr());
    void scanPoseActiveSensor(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL, const std::string& handler = std::string(), ScanPoseCommand scanPoseCommand = nullptr, grasp::Manager::Data::Ptr dataPtr = grasp::Manager::Data::Ptr());
    void scanPoseActiveFile(grasp::data::Item::List& scannedImageItems, const std::string& itemLabel, const std::string& handler = std::string());
	
	grasp::ConfigMat34 getPoseFromConfig(const grasp::ConfigMat34& config, int jointIdx);

	/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
    /** This function is useful to draw stuff that changes dinamically **/
    virtual void postprocess(golem::SecTmReal elapsedTime);
    /** This function creates renderers and possibly add objects to them **/

    /** This function is useful for adding objects that don't change dinamically, but instead only after some major processing takes place
     *  So this function can be called to reset the renderers and re-add the semi-static objects on the scene after their state is changed **/
    virtual void createRender();
    void clearRender();
    void clearItemsView();

    virtual void perform2(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory,
                          const bool testTrajectory = false, const bool controlRecording = false);

    // Manipulator::Waypoint::Seq is called a path
    grasp::Manipulator::Waypoint::Seq convertToManipulatorWayPoints(const grasp::Waypoint::Seq& waypoints);
    grasp::Manipulator::Waypoint::Seq convertToManipulatorWayPoints(const grasp::data::ItemTrajectory::Ptr& itemTrajectory);
    grasp::Manipulator::Waypoint::Seq convertToManipulatorWayPoints(const golem::Controller::State::Seq &trajectory);
    golem::Controller::State::Seq completeTrajectory2(grasp::data::Trajectory& trajectory);
    grasp::Manipulator::Waypoint::Seq completeTrajectory(grasp::data::Trajectory& trajectory);
    grasp::data::ItemTrajectory::Ptr convertToTrajectory(const golem::Controller::State::Seq& trajectory);

    ////////////////////////////////////// SANDBOX TRAJECTORY INTERPOLATION ////////////////////////////////////
    void drawBox(const golem::Vec3& p, const golem::BoundingBox::Ptr& box, bool visible = false);
    void sandBoxTrajInterp(const golem::Controller::State::Seq& trajectory);
    // create path
    //density.path = manipulator->create(waypoints, [=](const Manipulator::Config& l, const Manipulator::Config& r) -> Real { return poseCovInv.dot(RBDist(l.frame, r.frame)); });

    golem::DebugRenderer& getDemoRenderer(){
        return this->demoRenderer;
    }

    grasp::Manipulator::Appearance& getManipulatorAppearance(){
        return this->manipulatorAppearance;
    }

protected:

    bool toggleShowCurrentHandler;

    /** Current viewHypothesis */
    golem::I32 currentViewHypothesis;

    /** Currently selected viewHypothesis */
    golem::I32 selectedCamera;

    /** ActiveSenseDemo renderer */
    golem::DebugRenderer demoRenderer;

    pacman::Collision::Ptr collision;

    //////// Manipulator /////////////
    /** Manipulator */
    grasp::Manipulator::Ptr manipulator;
    /** Manipulator Appearance */
    grasp::Manipulator::Appearance manipulatorAppearance;
    /** Manipulator trajectory item */
    std::string manipulatorItemTrj;
    /** Manipulator pose distribution covariance */
    grasp::RBDist poseCov, poseCovInv;
    /** Maximum distance between frames in squared standard deviations */
    golem::Real poseDistanceMax;

    /** Manipulation trajectory duration */
    golem::SecTmReal manipulatorTrajectoryDuration;

    /** Manipulation trajectory force threshold */
    golem::Twist trajectoryThresholdForce;

    /* Withdraw action hand release fraction */
    golem::Real withdrawReleaseFraction;
    /* Withdraw action lift distance */
    golem::Real withdrawLiftDistance;
    //////// End Manipulator /////////////

    /** golem::UIRenderer interface */
    virtual void render() const;

    void create(const Desc& desc);

    ActiveSenseDemo(golem::Scene &scene);
    virtual ~ActiveSenseDemo();




};


//------------------------------------------------------------------------------

} // namespace pacman

#endif // _ACTIVE_SENSE_GRASP_DEMO_GRASP_H_
