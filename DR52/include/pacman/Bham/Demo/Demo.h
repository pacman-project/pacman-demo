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
#include <Grasp/Grasp/Model.h>
#include <Grasp/Grasp/Query.h>

//------------------------------------------------------------------------------

/** PaCMan name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player, public pacman::ActiveSenseController {
public:
	friend class ActiveSense;
	/** Model/Query any identifier */
	static const std::string ID_ANY;

	/** Data */
	class Data : public grasp::Player::Data {
	public:
		/** Model training data */
		class Training {
		public:
			/** Collection */
			typedef std::multimap<std::string, Training> Map;

			/** Initialisation */
			Training(const golem::Controller::State& state) : state(state) {}

			/** Robot state */
			golem::Controller::State state;
			/** End-effector frame */
			golem::Mat34 frame;
			/** Contacts */
			grasp::Contact3D::Seq contacts;
			/** Locations */
			grasp::data::Location3D::Point::Seq locations;
		};

		/** Data bundle default name */
		std::string dataName;

		/** Query mode */
		bool queryMode;

		/** Model triangles */
		grasp::Vec3Seq modelVertices;
		/** Model triangles */
		grasp::TriangleSeq modelTriangles;
		/** Model frame */
		golem::Mat34 modelFrame;
		/** Model frame offset */
		golem::Mat34 modelFrameOffset;

		/** Model robot state */
		golem::Controller::State::Ptr modelState;
		
		/** Query triangles */
		grasp::Vec3Seq queryVertices;
		/** Query triangles */
		grasp::TriangleSeq queryTriangles;
		/** Query frame */
		golem::Mat34 queryFrame;

		/** Training data */
		Training::Map training;

		/** Model training data type index */
		golem::U32 indexType;
		/** Model training data item index */
		golem::U32 indexItem;
		/** Contact relation */
		golem::U32 contactRelation;

		/** Data bundle description */
		class Desc : public grasp::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Get training item */
		Training::Map::iterator getTrainingItem();
		/** Set training item */
		void setTrainingItem(Training::Map::const_iterator ptr);

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

		/** Active Sense Attribute */
		ActiveSense::Ptr activeSense;

		/** Data bundle default name */
		std::string dataName;

		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model data item object */
		std::string modelItemObj;

		/** Model scan pose */
		grasp::ConfigMat34 modelScanPose;
		/** Model colour solid */
		golem::RGBA modelColourSolid;
		/** Model colour wireframe */
		golem::RGBA modelColourWire;

		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;

		/** Query pose estimation camera */
		std::string queryCamera;
		/** Query data handler */
		std::string queryHandler;
		/** Query data item */
		std::string queryItem;
		/** Query data item object */
		std::string queryItemObj;

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

		/** Model descriptions */
		grasp::Model::Desc::Map modelDescMap;
		/** Contact appearance */
		grasp::Contact3D::Appearance contactAppearance;


		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();


			this->activeSense = ActiveSense::Ptr(new ActiveSense());

			dataDesc.reset(new Data::Desc);

			dataName.clear();

			modelCamera.clear();
			modelHandler.clear();
			modelItem.clear();
			modelItemObj.clear();

			modelScanPose.setToDefault();
			modelColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			modelColourWire.set(golem::RGBA::GREEN);

			modelHandlerTrj.clear();
			modelItemTrj.clear();

			queryCamera.clear();
			queryHandler.clear();
			queryItem.clear();
			queryItemObj.clear();

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

			modelDescMap.clear();
			contactAppearance.setToDefault();

		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);
			this->activeSense->assertValid(ac);

			grasp::Assert::valid(dataDesc != nullptr && grasp::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			grasp::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");

			grasp::Assert::valid(modelCamera.length() > 0, ac, "modelCamera: invalid");
			grasp::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			grasp::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			grasp::Assert::valid(modelItemObj.length() > 0, ac, "modelItemObj: invalid");

			modelScanPose.assertValid(grasp::Assert::Context(ac, "modelScanPose."));

			grasp::Assert::valid(modelHandlerTrj.length() > 0, ac, "modelHandlerTrj: invalid");
			grasp::Assert::valid(modelItemTrj.length() > 0, ac, "modelItemTrj: invalid");

			grasp::Assert::valid(queryCamera.length() > 0, ac, "queryCamera: invalid");
			grasp::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
			grasp::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
			grasp::Assert::valid(queryItemObj.length() > 0, ac, "queryItemObj: invalid");

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

			grasp::Assert::valid(!modelDescMap.empty(), ac, "modelDescMap: empty");
			for (grasp::Model::Desc::Map::const_iterator i = modelDescMap.begin(); i != modelDescMap.end(); ++i) {
				grasp::Assert::valid(i->second != nullptr, ac, "modelDescMap[]: null");
				i->second->assertValid(grasp::Assert::Context(ac, "modelDescMap[]->"));
			}
			contactAppearance.assertValid(grasp::Assert::Context(ac, "contactAppearance."));
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
	/** Data bundle default name */
	std::string dataName;

	/** Model pose estimation camera */
	grasp::Camera* modelCamera;
	/** Model data handler */
	grasp::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model data item object */
	std::string modelItemObj;

	/** Model scan pose */
	grasp::ConfigMat34 modelScanPose;
	/** Model colour solid */
	golem::RGBA modelColourSolid;
	/** Model colour wireframe */
	golem::RGBA modelColourWire;
	
	/** Model renderer */
	golem::DebugRenderer modelRenderer;

	/** Model trajectory handler */
	grasp::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;

	/** Query pose estimation camera */
	grasp::Camera* queryCamera;
	/** Query data handler */
	grasp::data::Handler* queryHandler;
	/** Query data item */
	std::string queryItem;
	/** Query data item object */
	std::string queryItemObj;

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

	/** Models */
	grasp::Model::Map modelMap;
	/** Contact appearance */
	grasp::Contact3D::Appearance contactAppearance;

	/** golem::UIRenderer interface */
	virtual void render() const;

	/** Pose estimation */
	grasp::data::Item::Map::iterator estimatePose(bool query);
	/** Grasp and capture object */
	grasp::data::Item::Map::iterator objectGraspAndCapture();
	/** Process object image and add to data bundle */
	grasp::data::Item::Map::iterator objectProcess(grasp::data::Item::Map::iterator ptr);
	/** Create trajectory name */
	std::string getTrajectoryName(const std::string& type) const;

	virtual bool gotoPoseWS(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
	virtual bool gotoPoseConfig(const grasp::ConfigMat34& config, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001);
	virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems, ScanPoseCommand scanPoseCommand = nullptr, const std::string itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL);
	/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);

	grasp::Camera* getWristCamera() const;
	golem::Mat34 getWristPose() const;
	void gotoWristPose(const golem::Mat34& w);
	void Demo::gotoPose2(const grasp::ConfigMat34& pose, const golem::SecTmReal duration);
	void rotateObjectInHand();


	void create(const Desc& desc);

	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(pacman::Demo::Data::Training::Map::value_type& value) const;
	template <> void Stream::write(const pacman::Demo::Data::Training::Map::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif // _PACMAN_BHAM_DEMO_DEMO_H_