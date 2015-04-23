/** @file Demo.h
*
* PaCMan DR 5.2. demo
*
*/

#pragma once
#ifndef _PACMAN_BHAM_DEMO_DEMO_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_DEMO_DEMO_H_

#include <Grasp/App/Player/Player.h>
#include <Grasp/Core/Camera.h>
#include <Grasp/Grasp/Model.h>
#include <Grasp/Core/RBPose.h>

/** PaCMan name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player {
public:
	/** Data */
	class Data : public grasp::Player::Data {
	public:
		/** Data bundle default name */
		std::string dataName;

		/** Model triangles */
		grasp::Vec3Seq modelVertices;
		/** Model triangles */
		grasp::TriangleSeq modelTriangles;

		/** Data bundle description */
		class Desc : public grasp::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Manager */
		virtual void setOwner(grasp::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		Demo* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	friend class Data;

	/** Demo description */
	class Desc : public grasp::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data bundle default name */
		std::string dataName;

		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;
		/** Model scan pose */
		grasp::ConfigMat34 modelScanPose;
		/** Model colour solid */
		golem::RGBA modelColourSolid;
		/** Model colour wireframe */
		golem::RGBA modelColourWire;

		/** Grasp force sensor */
		std::string graspSensorForce;
		/** Grasp force threshold */
		golem::Twist graspThresholdForce;
		/** Grasp force event - hand close time wait */
		golem::SecTmReal graspEventTimeWait;
		/** Grasp hand close duration */
		golem::SecTmReal graspCloseDuration;
		/** Grasp pose open (pre-grasp) */
		grasp::ConfigMat34 graspPoseOpen;
		/** Grasp pose closed (grasp) */
		grasp::ConfigMat34 graspPoseClosed;

		/** Object capture camera */
		std::string objectCamera;
		/** Object data handler (scan) */
		std::string objectHandlerScan;
		/** Object data handler (processed) */
		std::string objectHandler;
		/** Object data item (scan) */
		std::string objectItemScan;
		/** Object data item (processed) */
		std::string objectItem;
		/** Object scan pose */
		grasp::ConfigMat34::Seq objectScanPoseSeq;
		/** Object manual frame adjustment */
		grasp::RBAdjust objectFrameAdjustment;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);

			dataName.clear();

			modelCamera.clear();
			modelHandler.clear();
			modelItem.clear();
			modelHandlerTrj.clear();
			modelItemTrj.clear();
			modelScanPose.setToDefault();
			modelColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			modelColourWire.set(golem::RGBA::GREEN);

			graspSensorForce.clear();
			graspThresholdForce.setZero();
			graspEventTimeWait = golem::SecTmReal(2.0);
			graspCloseDuration = golem::SecTmReal(2.0);
			graspPoseOpen.setToDefault();
			graspPoseClosed.setToDefault();

			objectCamera.clear();
			objectHandlerScan.clear();
			objectHandler.clear();
			objectItemScan.clear();
			objectItem.clear();
			objectScanPoseSeq.clear();
			objectFrameAdjustment.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);

			grasp::Assert::valid(dataDesc != nullptr && grasp::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			grasp::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");

			grasp::Assert::valid(modelCamera.length() > 0, ac, "modelCamera: invalid");
			grasp::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			grasp::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			grasp::Assert::valid(modelHandlerTrj.length() > 0, ac, "modelHandlerTrj: invalid");
			grasp::Assert::valid(modelItemTrj.length() > 0, ac, "modelItemTrj: invalid");
			modelScanPose.assertValid(grasp::Assert::Context(ac, "modelScanPose."));

			grasp::Assert::valid(graspSensorForce.length() > 0, ac, "graspSensorForce: invalid");
			grasp::Assert::valid(graspThresholdForce.isPositive(), ac, "graspThresholdForce: negative");
			grasp::Assert::valid(graspEventTimeWait > golem::SEC_TM_REAL_ZERO, ac, "graspEventTimeWait: <= 0");
			grasp::Assert::valid(graspCloseDuration > golem::SEC_TM_REAL_ZERO, ac, "graspCloseDuration: <= 0");
			graspPoseOpen.assertValid(grasp::Assert::Context(ac, "graspPoseOpen."));
			graspPoseClosed.assertValid(grasp::Assert::Context(ac, "graspPoseClosed."));

			grasp::Assert::valid(objectCamera.length() > 0, ac, "objectCamera: invalid");
			grasp::Assert::valid(objectHandlerScan.length() > 0, ac, "objectHandlerScan: invalid");
			grasp::Assert::valid(objectHandler.length() > 0, ac, "objectHandler: invalid");
			grasp::Assert::valid(objectItemScan.length() > 0, ac, "objectItemScan: invalid");
			grasp::Assert::valid(objectItem.length() > 0, ac, "objectItem: invalid");
			grasp::Assert::valid(!objectScanPoseSeq.empty(), ac, "objectScanPoseSeq: empty");
			for (grasp::ConfigMat34::Seq::const_iterator i = objectScanPoseSeq.begin(); i != objectScanPoseSeq.end(); ++i)
				i->assertValid(grasp::Assert::Context(ac, "objectScanPoseSeq[]."));
			objectFrameAdjustment.assertValid(grasp::Assert::Context(ac, "objectFrameAdjustment."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(Demo, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Data bundle default name */
	std::string dataName;

	/** Model pose estimation camera */
	grasp::Camera* modelCamera;
	/** Model data handler */
	grasp::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model trajectory handler */
	grasp::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;
	/** Model scan pose */
	grasp::ConfigMat34 modelScanPose;

	/** Model colour solid */
	golem::RGBA modelColourSolid;
	/** Model colour wireframe */
	golem::RGBA modelColourWire;
	/** Model renderer */
	golem::DebugRenderer modelRenderer;

	/** Grasp force sensor */
	grasp::FT* graspSensorForce;
	/** Grasp force threshold */
	golem::Twist graspThresholdForce;
	/** Grasp force event - hand close time wait */
	golem::SecTmReal graspEventTimeWait;
	/** Grasp hand close duration */
	golem::SecTmReal graspCloseDuration;
	/** Grasp pose open (pre-grasp) */
	grasp::ConfigMat34 graspPoseOpen;
	/** Grasp pose closed (grasp) */
	grasp::ConfigMat34 graspPoseClosed;

	/** Object capture camera */
	grasp::Camera* objectCamera;
	/** Object data handler (scan) */
	grasp::data::Handler* objectHandlerScan;
	/** Object data handler (processed) */
	grasp::data::Handler* objectHandler;
	/** Object data item (scan) */
	std::string objectItemScan;
	/** Object data item (processed) */
	std::string objectItem;
	/** Object scan pose */
	grasp::ConfigMat34::Seq objectScanPoseSeq;
	/** Object manual frame adjustment */
	grasp::RBAdjust objectFrameAdjustment;
	/** Object renderer */
	golem::DebugRenderer objectRenderer;

	/** golem::UIRenderer interface */
	virtual void render() const;

	/** Grasp and capture object */
	grasp::data::Item::Map::iterator objectGraspAndCapture();
	/** Process object image and add to data bundle */
	void objectProcess(grasp::data::Item::Map::iterator ptr);

	void create(const Desc& desc);

	grasp::Camera* getWristCamera() const;
	golem::Mat34 getWristPose() const;
	void gotoWristPose(const golem::Mat34& w);
	void Demo::gotoPose2(const grasp::ConfigMat34& pose, const golem::SecTmReal duration);
	void rotateObjectInHand();

	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

#endif // _PACMAN_BHAM_DEMO_DEMO_H_