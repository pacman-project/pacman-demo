/** @file Demo.h
*
* PaCMan DR 5.2. demo
*
*/

#pragma once
#ifndef _PACMAN_BHAM_DEMO_DEMO_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_DEMO_DEMO_H_

#include <Grasp/App/Player/Player.h>

#include <Grasp/Grasp/Model.h>
#include <Grasp/Core/RBPose.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Grasp/Model.h>
#include <Grasp/Grasp/Query.h>
#include <Grasp/Grasp/Manipulator.h>

//------------------------------------------------------------------------------

/** PaCMan name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player {
public:
	/** Model/Query any identifier */
	static const std::string ID_ANY;

	/** Data */
	class Data : public grasp::Player::Data {
	public:
		/** Mode */
		enum Mode {
			/** Model data */
			MODE_MODEL,
			/** Query density */
			MODE_QUERY,
			/** Solution */
			MODE_SOLUTION,

			/** First */
			MODE_FIRST = MODE_MODEL,
			/** Last */
			MODE_LAST = MODE_SOLUTION,
		};

		/** Mode name */
		static const std::string ModeName[MODE_LAST + 1];

		/** Model training data */
		class Training {
		public:
			/** Collection */
			typedef std::multimap<std::string, Training> Map;

			/** Initialisation */
			Training(const golem::Controller::State& state) : state(state) {}

			/** Robot state */
			golem::Controller::State state;
			/** Model frame */
			golem::Mat34 frame;
			/** Contacts */
			grasp::Contact3D::Seq contacts;
			/** Locations */
			grasp::data::Location3D::Point::Seq locations;
		};

		/** Query density */
		class Density : public golem::Sample<golem::Real> {
		public:
			/** Collection of distributions */
			typedef std::vector<Density> Seq;

			/** Type */
			std::string type;
			/** Object density */
			grasp::Query::Pose::Seq object;
			/** Pose density */
			grasp::Query::Pose::Seq pose;
			/** Path */
			grasp::Manipulator::Waypoint::Seq path;
			/** Locations */
			grasp::data::Location3D::Point::Seq locations;
			/** End-effector frame */
			golem::Mat34 frame;
		};

		/** Solution */
		class Solution {
		public:
			/** Collection of solutions */
			typedef std::vector<Solution> Seq;

			/** Type */
			std::string type;
			/** Pose */
			grasp::RBCoord pose;
			/** Path */
			grasp::Manipulator::Waypoint::Seq path;

			/** Likelihood */
			golem::Real likelihood;

			/** Query index */
			golem::U32 queryIndex;
		};

		/** Data bundle default name */
		std::string dataName;

		/** Current Mode */
		Mode mode;

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
		/** Model contact relation */
		grasp::Contact3D::Relation contactRelation;

		/** Collection of distributions */
		Density::Seq densities;
		/** Density index */
		golem::U32 indexDensity;

		/** Solutions */
		Solution::Seq solutions;
		/** Solution index */
		golem::U32 indexSolution;

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

	/** Pose density description */
	class PoseDensity {
	public:
		/** map */
		typedef std::map<std::string, PoseDensity> Map;

		/** Std dev */
		grasp::RBDist stdDev;
		/** Kernels */
		golem::U32 kernels;
		/** Path distance */
		grasp::RBDist pathDist;
		/** Path distance std dev */
		golem::Real pathDistStdDev;

		/** Constructs from description object */
		PoseDensity() {
			PoseDensity::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			stdDev.set(golem::Real(0.01), golem::Real(50.0));
			kernels = 100;
			pathDist.set(golem::Real(0.1), golem::Real(20.0));
			pathDistStdDev = golem::Real(1.0);
		};
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(stdDev.isValid(), ac, "stdDev: invalid");
			grasp::Assert::valid(kernels > 0, ac, "kernels: <= 0");
			grasp::Assert::valid(pathDist.isValid(), ac, "pathDist: invalid");
			grasp::Assert::valid(pathDistStdDev > golem::REAL_EPS, ac, "pathDistStdDev: < eps");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Optimisation description */
	class Optimisation {
	public:
		/** number of runs */
		size_t runs;
		/** number of steps per run */
		size_t steps;

		/** Simulated annealing minimum temperature */
		golem::Real saTemp;
		/** Simulated annealing temperature to local coordinate scaling factor */
		golem::Real saDelta;
		/** Simulated annealing temperature to energy scaling factor */
		golem::Real saEnergy;

		/** Constructs description object */
		Optimisation() {
			Optimisation::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			runs = 1000;
			steps = 1000;

			saTemp = golem::Real(0.1);
			saDelta = golem::Real(1.0);
			saEnergy = golem::Real(0.5);
		}
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(runs > 0, ac, "runs: <= 0");
			grasp::Assert::valid(steps > 0, ac, "steps: <= 0");
			grasp::Assert::valid(saTemp >= golem::REAL_ZERO, ac, "saTemp: < 0");
			grasp::Assert::valid(saDelta > golem::REAL_ZERO, ac, "saDelta: <= 0");
			grasp::Assert::valid(saEnergy > golem::REAL_ZERO, ac, "saEnergy: <= 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Demo description */
	class Desc : public grasp::Player::Desc {
	public:
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

		/** Query trajectory handler */
		std::string queryHandlerTrj;
		/** Query trajectory item */
		std::string queryItemTrj;

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

		/** Query descriptions */
		grasp::Query::Desc::Map queryDescMap;
		/** Pose descriptions */
		PoseDensity::Map poseMap;

		/** Optimisation description */
		Optimisation optimisation;

		/** Manipulator description */
		grasp::Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator Appearance */
		grasp::Manipulator::Appearance manipulatorAppearance;
		/** Manipulator pose distribution standard deviation */
		grasp::RBDist manipulatorPoseStdDev;

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

			queryHandlerTrj.clear();
			queryItemTrj.clear();

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

			queryDescMap.clear();
			poseMap.clear();

			optimisation.setToDefault();

			manipulatorDesc.reset(new grasp::Manipulator::Desc);
			manipulatorAppearance.setToDefault();
			manipulatorPoseStdDev.set(golem::Real(0.002), golem::Real(1000.0));
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);

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

			grasp::Assert::valid(queryHandlerTrj.length() > 0, ac, "queryHandlerTrj: invalid");
			grasp::Assert::valid(queryItemTrj.length() > 0, ac, "queryItemTrj: invalid");

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

			grasp::Assert::valid(!queryDescMap.empty(), ac, "queryDescMap: empty");
			for (grasp::Query::Desc::Map::const_iterator i = queryDescMap.begin(); i != queryDescMap.end(); ++i) {
				grasp::Assert::valid(i->second != nullptr, ac, "queryDescMap[]: null");
				i->second->assertValid(grasp::Assert::Context(ac, "queryDescMap[]->"));
			}
			grasp::Assert::valid(!poseMap.empty(), ac, "poseMap: empty");
			for (PoseDensity::Map::const_iterator i = poseMap.begin(); i != poseMap.end(); ++i) {
				i->second.assertValid(grasp::Assert::Context(ac, "poseMap[]->"));
			}

			optimisation.assertValid(grasp::Assert::Context(ac, "optimisation."));

			grasp::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(grasp::Assert::Context(ac, "manipulatorDesc->"));
			manipulatorAppearance.assertValid(grasp::Assert::Context(ac, "manipulatorAppearance."));
			grasp::Assert::valid(manipulatorPoseStdDev.isValid(), ac, "manipulatorPoseStdDev: invalid");
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

	/** Query trajectory handler */
	grasp::data::Handler* queryHandlerTrj;
	/** Query trajectory item */
	std::string queryItemTrj;

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

	/** Query densities */
	grasp::Query::Map queryMap;
	/** Pose descriptions */
	PoseDensity::Map poseMap;

	/** Optimisation description */
	Optimisation optimisation;

	/** Manipulator */
	grasp::Manipulator::Ptr manipulator;
	/** Manipulator Appearance */
	grasp::Manipulator::Appearance manipulatorAppearance;
	/** Manipulator pose distribution covariance */
	grasp::RBDist poseCov, poseCovInv;

	/** Item selection */
	typedef std::function<void(Data::Training::Map&, Data::Training::Map::iterator&)> ItemSelectFunc;
	typedef std::function<void(ItemSelectFunc)> ItemSelect;
	ItemSelect itemSelect;

	/** golem::UIRenderer interface */
	virtual void render() const;

	/** Pose estimation */
	grasp::data::Item::Map::iterator estimatePose(Data::Mode mode);
	/** Grasp and capture object */
	grasp::data::Item::Map::iterator objectGraspAndCapture();
	/** Process object image and add to data bundle */
	grasp::data::Item::Map::iterator objectProcess(grasp::data::Item::Map::iterator ptr);
	/** Create trajectory name */
	std::string getTrajectoryName(const std::string& prefix, const std::string& type) const;

	/** Create query densities */
	void createQuery(grasp::data::Item::Ptr item, const golem::Mat34& frame);

	/** Generate solutions */
	void generateSolutions();
	/** Sort solutions */
	static void sortSolutions(Data::Solution::Seq& seq);

	/** Select trajectory */
	void selectTrajectory();

	/** Perform trajectory */
	void performTrajectory();

	grasp::Camera* getWristCamera(const bool dontThrow = false) const;
	golem::Mat34 getWristPose() const;
	golem::Controller::State::Seq getTrajectoryFromPose(const golem::Mat34& w);
	grasp::ConfigMat34 getConfigFromPose(const golem::Mat34& w);
	void gotoWristPose(const golem::Mat34& w);
	void gotoPose2(const grasp::ConfigMat34& pose, const golem::SecTmReal duration);

	void nudgeWrist();
	void rotateObjectInHand();

	void create(const Desc& desc);

	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

//------------------------------------------------------------------------------

namespace golem {
	void XMLData(pacman::Demo::PoseDensity::Map::value_type& val, golem::XMLContext* context, bool create = false);

	template <> void Stream::read(pacman::Demo::Data::Training::Map::value_type& value) const;
	template <> void Stream::write(const pacman::Demo::Data::Training::Map::value_type& value);

	template <> void Stream::read(pacman::Demo::Data::Density::Seq::value_type& value) const;
	template <> void Stream::write(const pacman::Demo::Data::Density::Seq::value_type& value);

	template <> void Stream::read(pacman::Demo::Data::Solution::Seq::value_type& value) const;
	template <> void Stream::write(const pacman::Demo::Data::Solution::Seq::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif // _PACMAN_BHAM_DEMO_DEMO_H_
