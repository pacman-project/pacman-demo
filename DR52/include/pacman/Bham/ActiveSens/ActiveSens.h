/** @file ActiveSens.h
 *
 * Bham Active sensing library
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_

#include <Grasp/Core/Library.h>
#include <Grasp/Core/Defs.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Phys/Scene.h>
#include <Grasp/Core/Camera.h>
#include <Grasp/App/Manager/Manager.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>
#include <Grasp/Core/Data.h>
#include <Grasp/Core/UI.h>
#include <Grasp/App/Player/Player.h>


/** PaCMan name space */
namespace pacman {

	class Demo;
	//------------------------------------------------------------------------------
	

	/** Sensor interface */
	class HypothesisSensor {
	public:
		typedef golem::shared_ptr<pacman::HypothesisSensor> Ptr;
		typedef std::map<std::string, Ptr> Map;
		typedef std::vector<Ptr> Seq;

		typedef grasp::ConfigMat34 Config;

		/** Sensor appearance */
		class Appearance {
		public:
			/** Frame size */
			golem::Vec3 frameSize;
			/** Show camera frame */
			bool frameShow;

			/** Shape colour */
			golem::RGBA shapeColour;
			/** Show sensor */
			bool shapeShow;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {

				frameSize.set(golem::Real(0.03), golem::Real(0.03), golem::Real(0.05));
				frameShow = true;
				shapeColour = golem::RGBA(127, 127, 127, 255);
				shapeShow = true;
			}
			/** Assert that the object is valid. */
			void assertValid(const grasp::Assert::Context& ac) const {
				grasp::Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");
			}
			/** Load from xml context */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Sensor description */
		class Desc {
		public:
			typedef golem::shared_ptr<Desc> Ptr;

			/** Sensor appearance */
			Appearance appearance;

			/** View frame pose (Frame attached to the front of this sensor appearance)*/
			golem::Mat34 viewFrame;

			/** Sensor shape */
			golem::Bounds::Desc::Seq shapeDesc;

			/** Constructs description. */
			Desc() {
				setToDefault();
			}
			/** Destroys description. */
			virtual ~Desc() {}

			/** Sets the parameters to the default values. */
			void setToDefault() {

				appearance.setToDefault();
				shapeDesc.clear();

				viewFrame.setId();
				viewFrame.p.z = 0.05;

			}

			/** Assert that the description is valid. */
			virtual void assertValid(const grasp::Assert::Context& ac) const {


				appearance.assertValid(grasp::Assert::Context(ac, "appearance."));
				for (golem::Bounds::Desc::Seq::const_iterator i = shapeDesc.begin(); i != shapeDesc.end(); ++i)
					grasp::Assert::valid((*i)->isValid(), ac, "shapeDesc[]: invalid");
			}

			/** Creates the object from the description. */
			//virtual Sensor::Ptr create(golem::Context &context) const = 0;
			virtual HypothesisSensor::Ptr create(const pacman::HypothesisSensor::Desc& desc)
			{
				return HypothesisSensor::Ptr(new pacman::HypothesisSensor(desc));
			}

			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
		};


		/** Curent sensor frame */
		virtual golem::Mat34 getFrame() const {
			//Insert here your hypothesis of a pose
			return pose*viewFrame;
		}

		/** Pose frame */
		golem::Mat34 getPose() const {
			return pose;
		}

		/** Local frame */
		golem::Mat34 getViewFrame() const {
			return viewFrame;
		}

		/** Sensor appearance */
		const Appearance& getAppearance() const {
			return appearance;
		}

		/** Draw sensor at the current pose */
		virtual void draw(const Appearance& appearance, golem::DebugRenderer& renderer) const;

	
	protected:

		/** Camera appearance */
		Appearance appearance;
		/** Camera shape */
		mutable golem::Bounds::Seq shape;
		/** Camera shape frame */
		grasp::Mat34Seq shapeFrame;

		// Hypothesis of a view frame
		golem::Mat34 viewFrame;

		golem::Mat34 pose;


		/** Creates/initialises the Sensor */
		void create(const pacman::HypothesisSensor::Desc& desc);

		/** Constructs the Sensor */
		HypothesisSensor(golem::Context& context);

		/** Constructs the Sensor */
		HypothesisSensor(const pacman::HypothesisSensor::Desc& desc);

	public:
		/** Constructs the Sensor with default configurations */
		HypothesisSensor(golem::Mat34 pose, golem::RGBA shapeColour = golem::RGBA::CYAN);

		/** Set OpenGL view point to *this sensor's frame view point*/
		void setGLView(golem::Scene& scene);

		/** Set OpenGL view point to the sensor's frame view point */
		static void setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame);
	};

	class ActiveSense {

	public:

		enum EMethod {

			RANDOM,
			CONTACT_BASED

		};


		class Parameters {
		public:

			/** Number of hypothesis samples */
			golem::U32 nsamples;

			/** Radius of the sphere defines the distance from object centroid from which sensor hypothesis are going to be generated */
			golem::Real radius;
			bool centroidAutoset;
			/** Min and Max values for biased hypothesis generation (heuristics for minimizing shade) */
			golem::Real minPhi, maxPhi, minTheta, maxTheta; //Param
			EMethod method;

			/** Initial Predictor Model */
			grasp::data::Item::Map::iterator predModelItem;
			/** Initial PointCurv cloud */
			grasp::data::Item::Map::iterator pointCurv;

			bool hasPointCurv;
		};

		static const std::string DFT_IMAGE_ITEM_LABEL;
		static const std::string DFT_POINT_CURV_ITEM_LABEL;
		static const std::string DFT_DATA_PATH;


		typedef std::pair<std::string, float> ValueTuple;
		typedef golem::shared_ptr<ActiveSense> Ptr;
		typedef std::vector<std::pair<grasp::data::Handler*, grasp::data::Transform*>> TransformMap;
		typedef std::function<grasp::data::Item::Map::iterator(grasp::data::Item::List& list, TransformMap::value_type& transformPtr)> ReduceFunc;

		friend class ActiveSenseController;
		
	public:
		/**
		ActiveSense Constructor, needs an owner
		*/
		ActiveSense(pacman::Demo* demoOwner);

		/**
		ActiveSense Default Constructor 
		Warning: *this needs to be initialised before use
		*/
		ActiveSense() : dataPath(ActiveSense::DFT_DATA_PATH), hasPointCurv(false), demoOwner(nullptr), centroid(0.0, 0.0, 0.0), useManualCentroid(false), minPhi(30.0), maxPhi(150.0), minTheta(45.0), maxTheta(135.0) {}

		/**
		ActiveSense initialiser (Same function as Constructor)
		*/
		void init(pacman::Demo* demoOwner);

		/** Generates a set of nsamples sensor hypotheses (viewHypotheses) with uniformly distributed view directions (viewDir) around a sphere with specified radius centered at centroid in workspace coordinates */
		void generateRandomViews(const golem::Vec3& centroid, const golem::I32& nsamples = 5, const golem::Real& radius = 0.25, bool heightBias = true);
		
		/**
		Generates next random view with uniformly generated view direction positioned at a distance of radius from sphere centroid
		*/
		pacman::HypothesisSensor::Ptr generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius = 0.25, bool heightBias = true);

		/**
		Processes list of itemImages
		Output: final item
		*/
		grasp::data::Item::Map::iterator processItems(grasp::data::Item::List& list);

		/** Gaze ActiveSense main method, approach (i)
		Approach i) Random view selection
		
		Output: final Item with ItemPointCurv representing integrated random scans
		*/
		grasp::data::Item::Map::iterator executeActiveSense(golem::U32 nsamples = 5, const golem::Real& radius = 0.25);

		grasp::data::Item::Map::iterator nextBestViewContactBased(golem::U32 nsamples = 5, const golem::Real& radius = 0.25);

		/** Greedy selection for next best view based on contact point information */
		pacman::HypothesisSensor::Ptr nextBestView();

		/** Gets current view hypotheses a.k.a. sensor hypotheses*/
		pacman::HypothesisSensor::Seq& getViewHypotheses() {
			return this->viewHypotheses;
		}

		/** Gets current view hypotheses a.k.a. sensor hypothesis*/
		pacman::HypothesisSensor::Ptr getViewHypothesis(golem::U32 index) {
			grasp::Assert::valid(index < this->viewHypotheses.size(), "this->viewHypotheses[index]");
			return this->viewHypotheses[index];
		}

		/**
		Gets crictical section lock for viewHypotheses vector member attribute of *this
		*/
		golem::CriticalSection& getCSViewHypotheses()
		{
			return this->csViewHypotheses;
		}

		/**
		Sets random generator seed
		*/
		void setRandSeed(const golem::RandSeed& seed){
			this->rand.setRandSeed(seed);
		}

		/**
		Sets bias limits in degrees. They define minimum and maximum spherical coordinates that are going to be used to generate sensor hypothesis
		*/
		void setBias(const golem::Real& minPhi, const golem::Real& maxPhi, const golem::Real& minTheta, const golem::Real& maxTheta){
			this->minPhi = minPhi;
			this->maxPhi = maxPhi;
			
			this->minTheta = minTheta;
			this->maxTheta = maxTheta;
		}

		void setPredModelItem(grasp::data::Item::Map::iterator predModelItem)
		{
			this->predModelItem = predModelItem;
		}

		void setPointCurv(grasp::data::Item::Map::iterator pointCurv)
		{
			this->pointCurv = pointCurv;
			this->hasPointCurv = true;
		}

		void setCentroidFromItemImage(grasp::data::Item::Map::iterator itemImage)
		{
			centroid = this->computeCentroid(itemImage);
		}
		void setUseManualCentroid(bool useManualCentroid)
		{
			this->useManualCentroid = useManualCentroid;
		}

		/**
		Computes goal frame such that i) camera is aligned with targetFrame
		Input: targetFrame, camera (to be aligned)
		Output: goal frame respecting statement (i)
		*/
		golem::Mat34 computeGoal(golem::Mat34& targetFrame, grasp::CameraDepth* camera);

	//Made public just for debugging (it should be protected)
	public:

		/**
		Adds item and its corresponding label to demoOwner->dataCurrentPtr->itemMap
		*/
		grasp::data::Item::Map::iterator addItem(const std::string& label, grasp::data::Item::Ptr item);

		/**
		Removes list of items from demoOwner->dataCurrentPtr->second->itemMap
		*/
		void removeItems(grasp::data::Item::List& list);

		/**
		Removes data bundle pointed by data from demoOwner->dataMap
		*/
		void removeData(grasp::data::Data::Map::iterator data);

		/**
		Creates data bundle and makes it be the demoOwner->dataCurrentPtr
		Output: Iterator of the newly created data bundle in demoOwner->dataMap
		*/
		grasp::Manager::Data::Map::iterator createData();
		
		/**
		Computes centroid of the point cloud represented by itemImage
		Output: Centroid of the point cloud represented by itemImage 
		(undefined behaviour if it's not a pointcloud)
		*/
		golem::Vec3 computeCentroid(grasp::data::Item::Map::const_iterator itemImage);


		/**
		Gets DemoOwner's OPENNI Camera, and also sets it as its demoOwner->sensorCurrentPtr
		Output: Pointer to Openni Camera Sensor
		*/
		grasp::CameraDepth* getOwnerOPENNICamera();

		/** 
		Executes the following steps: 
		1) Transforms (predModelItem,pointCurvItem) into predQuery;
		2) Converts predQuery into a Trajectory (traj)
		3) feedback step => Transforms (pointCurvItem,traj) into a predModelItem

		Output: predModelItem result of the feedBack transform
		*/
		grasp::data::Item::Map::iterator feedBackTransform(grasp::data::Item::Map::iterator predModelItem, grasp::data::Item::Map::iterator pointCurvItem);
		
		grasp::data::Item::Map::iterator convertToTrajectory(grasp::data::Item::Map::iterator predQueryModel);

		/** Computes the value of a hypothesis sensor */
		ValueTuple computeValue(HypothesisSensor::Ptr hypothesis, grasp::data::Item::Map::iterator);

		virtual void render() const;

	protected:
		golem::CriticalSection csRenderer;
		golem::DebugRenderer debugRenderer;

		grasp::data::Item::Map::iterator predModelItem;
		grasp::data::Item::Map::iterator pointCurv;
		bool hasPointCurv;


		std::string dataPath; //Param

		/**
		TransformMap containing the transformations useful for *this
		*/
		TransformMap transformMap;

		/**
		Owner of *this ActiveSense object (usually the main robot interface)
		*/
		pacman::Demo* demoOwner;

		/**
		Sequence of Sensor's View Hypotheses visited so far (or to be visited)
		Important: Write and Read operations should be done using the lock csViewHypotheses
		*/
		pacman::HypothesisSensor::Seq viewHypotheses;

		/**
		Lock for this->viewHypotheses
		Important: Write and Read operations on this->viewHypotheses should be done using this lock
		*/
		golem::CriticalSection csViewHypotheses;

		/**
		Centroid around random generated view directions are generated
		*/
		golem::Vec3 centroid;

		/**
		Defines whether this->centroid is going to be manually set or computed from a image point cloud item
		*/
		bool useManualCentroid; //Param

		/**
		Random number generator (used for uniform direction generation)
		*/
		golem::Rand rand;

		/**
		Min and Max values for biased hypothesis generation (heuristics for minimizing shade)
		*/
		golem::Real minPhi, maxPhi, minTheta, maxTheta; //Param
		
	

	};
	

	/**
	ActiveSenseController defines a set of basic functions that we need to actively control Boris
	*/
	class ActiveSenseController {

	public:
		typedef std::function<bool()> ScanPoseActiveCommand;

		ActiveSenseController() {}

	protected:

		/** ActiveSense Object, responsible for Gaze Selection */
		ActiveSense activeSense;

		//TODO: Delete this dummy thing
		pacman::HypothesisSensor::Ptr dummyObject;

		/** Initializes ActiveSense */
		virtual void initActiveSense(pacman::Demo* demoOwner);

		/** Sends Boris' left arm to a pose in workspace coordinates */
		virtual void gotoPoseWS(const grasp::ConfigMat34& pose) = 0;

		/** Captures an image generating a point cloud represented by ImageItems. 
			It should add all scanned Image Items to scannedImageItems
		*/
		virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems, 
			ScanPoseActiveCommand scanPoseCommand = nullptr, 
			const std::string itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL) = 0;

		/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
		virtual void postprocess(golem::SecTmReal elapsedTime) = 0;

	};
};

#endif // _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_