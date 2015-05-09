/** @file Demo.h
*
* PaCMan DR 5.2. demo
*
*/

#pragma once
#ifndef _PACMAN_BHAM_DEMO_DEMO_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_DEMO_DEMO_H_

#include <Grasp/App/Player/Player.h>

#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Phys/Scene.h>
#include <Grasp/Core/Camera.h>

#include <Grasp/Grasp/Model.h>
#include <Grasp/Core/RBPose.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Grasp/Query.h>

//------------------------------------------------------------------------------

/** PaCMan name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player, public pacman::ActiveSenseController {
public:
	friend class ActiveSense;

	/** Demo description */

	class Desc : public grasp::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Active Sense Attribute */
		ActiveSense::Ptr activeSense;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();


			this->activeSense = ActiveSense::Ptr(new ActiveSense());
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);
			this->activeSense->assertValid(ac);
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(Demo, golem::Object::Ptr, golem::Scene&)
	};
protected:
	
	/** Current viewHypothesis */
	golem::I32 currentViewHypothesis;

	/** Currently selected viewHypothesis */
	golem::I32 selectedCamera;
	

protected:
	/** golem::UIRenderer interface */
	virtual void render() const;

	virtual bool gotoPoseWS(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
	virtual bool gotoPoseConfig(const grasp::ConfigMat34& config, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
	virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems, ScanPoseCommand scanPoseCommand = nullptr, const std::string itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL);
	/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);

	grasp::Camera* getWristCamera(const bool dontThrow = false) const;
	golem::Mat34 getWristPose() const;
	golem::Controller::State::Seq getTrajectoryFromPose(const golem::Mat34& w, const golem::SecTmReal duration);
	grasp::ConfigMat34 getConfigFromPose(const golem::Mat34& w);
	golem::Controller::State lookupStateArmCommandHand() const;

	void setHandConfig(golem::Controller::State::Seq& trajectory, const golem::Controller::State cmdHand);
	void setHandConfig(golem::Controller::State::Seq& trajectory, const grasp::ConfigMat34& handPose);
	void gotoWristPose(const golem::Mat34& w, const golem::SecTmReal duration = golem::SEC_TM_REAL_ZERO);
	void gotoPose2(const grasp::ConfigMat34& pose, const golem::SecTmReal duration, const bool ignoreHand = false);
	void releaseHand(const double openFraction, const golem::SecTmReal duration);
	void closeHand(const double closeFraction, const golem::SecTmReal duration);

	void nudgeWrist();
	void rotateObjectInHand();

	void executeDropOff();

	void create(const Desc& desc);

	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

//------------------------------------------------------------------------------

#endif // _PACMAN_BHAM_DEMO_DEMO_H_