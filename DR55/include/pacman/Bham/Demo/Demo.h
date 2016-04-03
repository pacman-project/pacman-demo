/** @file Grasp.h
 *
 * Adapated from Demo Grasp by Ermano Arruda
 *
 * @author	Marek Kopicki
 * @author Ermano Arruda
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _PACMAN_DEMO_GRASP_H_ // if #pragma once is not supported
#define _PACMAN_DEMO_GRASP_H_

#include "pacman/Bham/ActiveSenseGrasp/Demo/ActiveSenseGraspDemo.h"

/** Pacman name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Grasp demo. */
class DemoDR55 : public pacman::ActiveSenseDemo {
public:

	//friend class Data;
	//
	/** Grasp demo description */
	class Desc : public pacman::ActiveSenseDemo::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Object passing pose */
		grasp::ConfigMat34 objPassingPose;

		/** Scan passing pose */
		grasp::ConfigMat34 scanPassingPose;

		/** Default pose left and right arm */
		grasp::ConfigMat34 dtfLeftPose;
		grasp::ConfigMat34 dtfRightPose;

		/** Passing camera */
		std::string passingCamera;

		/** Passing image handler */
		std::string passingImageHandler;

		/** Passing point curv handler */
		std::string passingPointCurvHandler;

		/** Passing grasp handler (Contact query) */
		std::string passingGraspHandler;

		/** Passing trajectory handler */
		std::string passingTrajectoryHandler;

		/** Passing grasp model item (Contact model) */
		std::string passingGraspModelItem;

		/** Passing contact query item */
		std::string passingCQueryItem;

		/** Passing grasp trajectory item */
		std::string passingGraspTrajectoryItem;

		/** Passing object item */
		std::string passingObjItem;



		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pacman::ActiveSenseDemo::Desc::setToDefault();

		
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			pacman::ActiveSenseDemo::Desc::assertValid(ac);

		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(DemoDR55, golem::Object::Ptr, golem::Scene&)
	};

protected:
	
	/** Model pose estimation camera */
	grasp::Camera* passingCamera;
	/** Image handler */
	grasp::data::Handler* passingImageHandler;

	/** PointsCurv handler */
	grasp::data::Handler* passingPointCurvHandler;

	/** Contact query (grasp) handler */
	grasp::data::Handler* passingGraspHandler;

	/** Passing trajectory handler */
	grasp::data::Handler* passingTrajectoryHandler;

	/** Passing grasp model item (Contact model) */
	std::string passingGraspModelItem;

	/** Passing object item */
	std::string passingObjItem;

	/** Passing contact query item */
	std::string passingCQueryItem;

	/** Passing grasp trajectory item */
	std::string passingGraspTrajectoryItem;


	/** Object passing pose */
	grasp::ConfigMat34 objPassingPose;

	/** Scan passing pose */
	grasp::ConfigMat34 scanPassingPose;

	/** Default pose left and right arm */
	grasp::ConfigMat34 dtfLeftPose;
	grasp::ConfigMat34 dtfRightPose;

	/** Executes passing demo */
	void executePassing(bool stopAtBreakPoint);
	void executePlacement(bool stopAtBreakPoint);

	void setLeftArmDeault(bool stopAtBreakPoint);
	void setRightArmDeault(bool stopAtBreakPoint);
	void setArmsToDefault(bool stopAtBreakPoint);

	virtual void setMenus();

	void executeCmd(const std::string& command);
	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	DemoDR55(golem::Scene &scene);
	virtual ~DemoDR55();
};


//------------------------------------------------------------------------------

};

#endif // _PACMAN_DEMO_GRASP_H_
