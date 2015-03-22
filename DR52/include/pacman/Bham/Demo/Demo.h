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

/** PaCMan name space */
namespace pacman {

	class ActiveSense;
//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player {
public:
	friend class ActiveSense;

	/** Player description */
	class Desc : public grasp::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;


		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();

		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);


		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(Demo, golem::Object::Ptr, golem::Scene&)
	};
protected:
	
	golem::I32 currentViewHypothesis;
	golem::I32 selectedCamera;
	ActiveSense activeSense;

	pacman::HypothesisSensor::Ptr dummyObject;

public:
	

protected:
	void create(const Desc& desc);
	void initActiveSense(golem::Scene &scene);
	void gotoPoseWS(const grasp::ConfigMat34& pose);
	void scanPoseActive(ScanPoseCommand scanPoseCommand = nullptr, std::string itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL);
	/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);

	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

#endif // _PACMAN_BHAM_DEMO_DEMO_H_