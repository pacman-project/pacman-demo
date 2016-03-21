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
#ifndef _GRASP_DEMO_GRASP_H_ // if #pragma once is not supported
#define _GRASP_DEMO_GRASP_H_

#include <Grasp/App/Player/Player.h>
#include <Grasp/Core/Cloud.h>

/** Pacman name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Grasp demo. */
class BaseDemo : public grasp::Player {
public:
	/** Robot pose */
	class Pose : public grasp::ConfigMat34 {
	public:
		typedef std::vector<Pose> Seq;

		/** Duration */
		golem::SecTmReal dt;
		/** Flags */
		std::string flags;

		/** Configspace type */
		bool configspace;
		/** Workspace position and orientation type */
		bool position, orientation;

		Pose() {
			setToDefault();
		}
        Pose(const grasp::RealSeq& c) : grasp::ConfigMat34(c), configspace(true), position(false), orientation(false), dt(golem::SEC_TM_REAL_ZERO) {
			w.setToDefault();
		}
		Pose(const grasp::RealSeq& c, const golem::Mat34& w) : grasp::ConfigMat34(c, w), configspace(true), position(true), orientation(true), dt(golem::SEC_TM_REAL_ZERO) {
		}

		void setToDefault() {
			grasp::ConfigMat34::setToDefault();
			dt = golem::SEC_TM_REAL_ZERO;
			flags.clear();
			configspace = true;
			position = orientation = false;
		}
		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::ConfigMat34::assertValid(ac);
			grasp::Assert::valid(dt >= golem::SEC_TM_REAL_ZERO, ac, "dt: negative");
		}
	};

	/** Grasp demo description */
	class Desc : public grasp::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Depth camera */
		std::string camera;

		/** Data bundle default name */
		std::string bundle;

		/** Raw image handler */
		std::string imageHandler;
		/** Raw image item */
		std::string imageItem;

		/** Processed image/feature handler */
		std::string processHandler;
		/** Processed image/feature item */
		std::string processItem;

		/** Contact model handler */
		std::string modelHandler;
		/** Contact model item */
		std::string modelItem;

		/** Contact query handler */
		std::string queryHandler;
		/** Contact query item */
		std::string queryItem;

		/** Trajectory handler */
		std::string trjHandler;
		/** Trajectory item */
		std::string trjItem;

		/** Filtering algortihm descriptiton */
		grasp::Cloud::FilterDesc detectFilterDesc;
		/** Filtering algortihm descriptiton */
		golem::U32 detectThreadChunkSize;
		/** Region in global coordinates */
		golem::Bounds::Desc::Seq detectRegionDesc;
		/** Object detection minimum size */
		golem::U32 detectMinSize;
		/** Object detection delta change */
		golem::U32 detectDeltaSize;
		/** Object detection delta depth */
		golem::Real detectDeltaDepth;

		/** Scan poses */
		Pose::Seq poseScanSeq;
		/** Action poses */
		Pose::Seq poseActionSeq;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();

			camera.clear();
			bundle.clear();
			imageHandler.clear();
			imageItem.clear();
			processHandler.clear();
			processItem.clear();
			modelHandler.clear();
			modelItem.clear();
			queryHandler.clear();
			queryItem.clear();
			trjHandler.clear();
			trjItem.clear();

			detectFilterDesc.setToDefault();
			detectThreadChunkSize = 1000;
			detectRegionDesc.clear();
			detectMinSize = 10000;
			detectDeltaSize = 100;
			detectDeltaDepth = golem::Real(0.001);

			poseScanSeq.clear();
			poseActionSeq.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);

			grasp::Assert::valid(camera.length() > 0, ac, "camera: invalid");
			grasp::Assert::valid(bundle.length() > 0, ac, "bundle: invalid");
			grasp::Assert::valid(imageHandler.length() > 0, ac, "imageHandler: invalid");
			grasp::Assert::valid(imageItem.length() > 0, ac, "imageItem: invalid");
			grasp::Assert::valid(processHandler.length() > 0, ac, "processHandler: invalid");
			grasp::Assert::valid(processItem.length() > 0, ac, "processItem: invalid");
			grasp::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			grasp::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			grasp::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
			grasp::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
			grasp::Assert::valid(trjHandler.length() > 0, ac, "trjHandler: invalid");
			grasp::Assert::valid(trjItem.length() > 0, ac, "trjItem: invalid");

			detectFilterDesc.assertValid(grasp::Assert::Context(ac, "detectFilterDesc."));
			grasp::Assert::valid(detectThreadChunkSize > 0, ac, "detectThreadChunkSize: < 1");
			for (golem::Bounds::Desc::Seq::const_iterator i = detectRegionDesc.begin(); i != detectRegionDesc.end(); ++i)
				grasp::Assert::valid((*i)->isValid(), ac, "detectRegionDesc[]: invalid");
			grasp::Assert::valid(detectMinSize > 0, ac, "detectMinSize: zero");
			grasp::Assert::valid(detectDeltaSize > 0, ac, "detectDeltaSize: zero");
			grasp::Assert::valid(detectDeltaDepth > golem::REAL_EPS, ac, "detectDeltaDepth: < eps");

			grasp::Assert::valid(!poseScanSeq.empty(), ac, "poseScanSeq: empty");
			for (Pose::Seq::const_iterator i = poseScanSeq.begin(); i != poseScanSeq.end(); ++i)
				i->assertValid(grasp::Assert::Context(ac, "poseScanSeq[]."));
			grasp::Assert::valid(!poseActionSeq.empty(), ac, "poseActionSeq: empty");
			for (Pose::Seq::const_iterator i = poseActionSeq.begin(); i != poseActionSeq.end(); ++i)
				i->assertValid(grasp::Assert::Context(ac, "poseActionSeq[]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(BaseDemo, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Depth camera */
	grasp::CameraDepth* camera;

	/** Data bundle default name */
	std::string bundle;

	/** Raw image handler */
	grasp::data::Handler* imageHandler;
	/** Raw image item */
	std::string imageItem;

	/** Processed image/feature handler */
	grasp::data::Handler* processHandler;
	/** Processed image/feature item */
	std::string processItem;

	/** Contact model handler */
	grasp::data::Handler* modelHandler;
	/** Contact model item */
	std::string modelItem;

	/** Contact query handler */
	grasp::data::Handler* queryHandler;
	/** Contact query item */
	std::string queryItem;

	/** Trajectory handler */
	grasp::data::Handler* trjHandler;
	/** Trajectory item */
	std::string trjItem;

	/** Filtering algortihm descriptiton */
	grasp::Cloud::FilterDesc detectFilterDesc;
	/** Filtering algortihm descriptiton */
	golem::U32 detectThreadChunkSize;
	/** Region in global coordinates */
	golem::Bounds::Desc::Seq detectRegionDesc;
	/** Object detection minimum size */
	golem::U32 detectMinSize;
	/** Object detection delta change */
	golem::U32 detectDeltaSize;
	/** Object detection delta depth */
	golem::Real detectDeltaDepth;

	/** Scan poses */
	Pose::Seq poseScanSeq;
	/** Action poses */
	Pose::Seq poseActionSeq;

	/** Demo renderer */
	golem::DebugRenderer demoRenderer;

	/** Move to the specified configuration */
	void gotoPose(const Pose& pose);

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	BaseDemo(golem::Scene &scene);
	~BaseDemo();
};

/** Reads/writes object from/to a given XML context */
void XMLData(BaseDemo::Pose& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};

#endif // _GRASP_DEMO_GRASP_H_
