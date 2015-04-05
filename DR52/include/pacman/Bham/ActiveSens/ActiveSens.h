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

			return config.w*viewFrame;
		}

		/** Pose frame */
		golem::Mat34 getPose() const {
			return config.w;
		}

		/** Pose frame */
		const HypothesisSensor::Config& getConfig() const {

			return this->config;
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

		/** Hypothesis of a view frame */
		golem::Mat34 viewFrame;

		/** Hypothesis view configuration */
		Config config;




		/** Creates/initialises the Sensor */
		void create(const pacman::HypothesisSensor::Desc& desc);

		/** Constructs the Sensor */
		HypothesisSensor(golem::Context& context);

		/** Constructs the Sensor */
		HypothesisSensor(const pacman::HypothesisSensor::Desc& desc);

	public:
		/** Constructs the Sensor with default configurations */
		HypothesisSensor(HypothesisSensor::Config config, golem::RGBA shapeColour = golem::RGBA::CYAN);

		/** Set OpenGL view point to *this sensor's frame view point*/
		void setGLView(golem::Scene& scene);

		/** Set OpenGL view point to the sensor's frame view point */
		static void setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame);
	};

	class ActiveSense {

	public:

		enum ESelectionMethod {

			S_RANDOM,
			S_CONTACT_BASED,
			S_NONE //Used for validity check

		};

		enum EGenerationMethod {

			G_RANDOM_SPHERE,
			G_FIXED,
			G_NONE //Used for validity check

		};


		class Parameters {
		public:

			typedef golem::shared_ptr<Parameters> Ptr;

			Parameters() {
				this->setToDefault();
			}


			/** Number of hypothesis samples */
			golem::U32 nsamples, nviews;

			/** Radius of the sphere defines the distance from object centroid from which sensor hypothesis are going to be generated */
			golem::Real radius;

			/**
			Centroid around random generated view directions are generated
			*/
			golem::Vec3 centroid;

			/**
			Defines whether this->centroid is going to be manually set or computed from a image point cloud item
			*/
			bool useManualCentroid, useHeightBias, regenerateViews;
			/** Min and Max values for biased hypothesis generation (heuristics for minimizing shade) */
			golem::Real minPhi, maxPhi, minTheta, maxTheta; //Param
			golem::U32 selectionMethod, generationMethod;

			/** Configuration sequence */
			HypothesisSensor::Config::Seq configSeq;

			/** Sensor to be used for active control */
			std::string sensorId;


			void setToDefault() {
				this->radius = 0.35;
				this->nsamples = 5;
				this->nviews = 5;
				this->useManualCentroid = false;
				this->centroid.setZero();
				this->minPhi = 30.0;
				this->maxPhi = 150.0;
				this->minTheta = 45.0;
				this->maxTheta = 135.0;

				this->selectionMethod = ESelectionMethod::S_CONTACT_BASED;
				this->generationMethod = EGenerationMethod::G_RANDOM_SPHERE;

				this->sensorId = "OpenNI+OpenNI";


			}

			/** Assert that the object is valid. */
			void assertValid(const grasp::Assert::Context& ac) const {
				grasp::Assert::valid(this->nsamples > 0, ac, "nsamples: <= 0");
				grasp::Assert::valid(this->nviews > 0, ac, "nviews: <= 0");
				grasp::Assert::valid(this->radius > 0, ac, "radius: <= 0");
				grasp::Assert::valid(this->selectionMethod != ESelectionMethod::S_NONE, ac, "Selection Method: is S_NONE (unknown selection method)");
				grasp::Assert::valid(this->generationMethod != EGenerationMethod::G_NONE, ac, "Generation Method: is G_NONE (unknown generation method)");
			}
			/** Load from xml context */
			void load(const golem::XMLContext* xmlcontext);
		};

		class Result {
		public:

			typedef golem::shared_ptr<Result> Ptr;

			grasp::data::Item::List predQueries;
			grasp::data::Item::List trajectories;
			grasp::data::Item::List pointCurvs;
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
		ActiveSense() : dataPath(ActiveSense::DFT_DATA_PATH), demoOwner(nullptr), hasPointCurv(false), hasPredModel(false), hasTrajectory(false) {}

		/**
		ActiveSense initialiser (Same function as Constructor)
		*/
		void init(pacman::Demo* demoOwner);

		/** Generates a set of nsamples sensor hypotheses (viewHypotheses) with uniformly distributed view directions (viewDir) around a sphere with specified radius centered at centroid in workspace coordinates */
		void generateRandomViews();

		/** Generates views according to current this->params setup */
		void generateViews();


		pacman::HypothesisSensor::Ptr generateViewFrom(const HypothesisSensor::Config& config);

		/** Generate Views given a sequence of predefined configurations */
		void generateViewsFromSeq(const HypothesisSensor::Config::Seq& configSeq);

		/**
		Generates next random view with uniformly generated view direction positioned at a distance of radius from sphere centroid
		*/
		pacman::HypothesisSensor::Ptr generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius = 0.25, bool heightBias = true);

		/**
		Processes list of itemImages
		Output: integrated itemPointCurv
		*/
		grasp::data::Item::Map::iterator processItems(grasp::data::Item::List& list);

		/** Gaze ActiveSense main method, approach (i)
		Approach i) Random view selection

		Output: final Item with ItemPointCurv representing integrated random scans
		*/
		grasp::data::Item::Map::iterator nextBestViewRandom();

		/** Contact based next best view selection */
		grasp::data::Item::Map::iterator nextBestViewContactBased();



		/** Gaze ActiveSense main method, automatically uses method/approach set in this->params
		Approach/Method i) Random view selection
		Approach/Method ii) Contact Based view selection

		Output: final Item with ItemPointCurv representing integrated random scans
		*/
		grasp::data::Item::Map::iterator nextBestView();
		
		/** Greedy selection for next best view based on contact point information */
		pacman::HypothesisSensor::Ptr selectNextBestViewContactBased(grasp::data::Item::Map::iterator predModelPtr);

		/** Selects next best view randomly from the sequence of generated this->viewHypotheses*/
		pacman::HypothesisSensor::Ptr selectNextBestViewRandom();

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
			this->params.minPhi = minPhi;
			this->params.maxPhi = maxPhi;

			this->params.minTheta = minTheta;
			this->params.maxTheta = maxTheta;
		}

		/**
		Sets PredictorModel item
		*/
		void setPredModelItem(grasp::data::Item::Map::iterator predModelItem)
		{
			this->predModelItem = predModelItem;
			this->hasPredModel = true;
		}

		/**
		Sets current online model (pointCurv member attribute)
		*/
		void setPointCurvItem(grasp::data::Item::Map::iterator pointCurv)
		{
			this->pointCurvItem = pointCurv;
			this->hasPointCurv = true;
		}

		/**
		Sets current PredQuery item
		*/
		void setPredQueryItem(grasp::data::Item::Map::iterator predQueryItem)
		{
			this->predQueryItem = predQueryItem;
			this->hasPredQuery = true;
		}

		/**
		Sets current trajectory item
		*/
		void setTrajectoryItem(grasp::data::Item::Map::iterator trajectoryItem)
		{
			this->trajectoryItem = trajectoryItem;
			this->hasTrajectory = true;
		}

		/**
		Shortcut for setting centroid parameter using an ItemImage Point Cloud
		*/
		void setCentroidFromItemImage(grasp::data::Item::Map::iterator itemImage)
		{
			this->params.centroid = this->computeCentroid(itemImage);
		}

		/**
		Shortcut for setting useManualCentroid parameter
		*/
		void setUseManualCentroid(bool useManualCentroid)
		{
			this->params.useManualCentroid = useManualCentroid;
		}

		/**
		Sets parameters for *this
		*/
		void setParameters(const pacman::ActiveSense::Parameters& params)
		{
			this->params.centroid = params.centroid;
			this->params.minPhi = params.minPhi;
			this->params.maxPhi = params.maxPhi;
			this->params.minTheta = params.minTheta;
			this->params.maxTheta = params.maxTheta;

			this->params.nsamples = params.nsamples;



			this->params.selectionMethod = params.selectionMethod;
			this->params.generationMethod = params.generationMethod;
		}


		/**
		Gets Parameters of *this
		*/
		pacman::ActiveSense::Parameters& getParameters(){
			return this->params;
		}

		/**
		Gets Parameters of *this
		*/
		const pacman::ActiveSense::Result& getResult(){
			this->result;
		}
		/**
		Computes goal frame such that i) camera is aligned with targetFrame
		Input: targetFrame, camera (to be aligned)
		Output: goal frame respecting statement (i)
		*/
		golem::Mat34 computeGoal(const golem::Mat34& targetFrame, const grasp::Camera* camera);

		/* Renders stuff */
		virtual void render() const;

		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			this->params.assertValid(ac);
		}

		/** Load from xml context */
		void load(const golem::XMLContext* xmlcontext);

	protected:

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

		/** Gets a sensor identified by its id=library+configFile, e.g. OpenNI+OpenNI, DepthSim+DepthSim*/
		grasp::Camera* ActiveSense::getOwnerSensor(const std::string& sensorId);

		/**
		Executes the following steps:
		1) Transforms (predModelItem,pointCurvItem) into predQuery;
		2) Converts predQuery into a Trajectory (traj)
		3) feedback step => Transforms (pointCurvItem,traj) into a predModelItem

		Output: predModelItem result of the feedBack transform
		*/
		grasp::data::Item::Map::iterator computeFeedBackTransform(grasp::data::Item::Map::iterator predModelItem, grasp::data::Item::Map::iterator pointCurvItem);


		/**
		Returns a Predictor model by following the following steps:
		1) Converts predQuery into a Trajectory (traj)
		2) feedback step => Transforms (pointCurvItem,traj) into a predModelItem
		*/
		grasp::data::Item::Map::iterator computePredModelFeedBack(grasp::data::Item::Map::iterator predQuery, grasp::data::Item::Map::iterator pointCurvItem);

		/**
		Returns a Predictor model by following the following steps:
		1) Transforms trajItem + pointCurvItem into a predictory model by applying appropriate transform
		*/
		grasp::data::Item::Map::iterator computeTransformPredModel(grasp::data::Item::Map::iterator trajItem, grasp::data::Item::Map::iterator pointCurvItem);

		/**
		Converts a predQueryModel into a Trajectory
		*/
		grasp::data::Item::Map::iterator convertToTrajectory(grasp::data::Item::Map::iterator predQueryModel);

		/** Computes the value of a hypothesis sensor given an ItemPointCurv as input */
		ValueTuple computeValue(HypothesisSensor::Ptr hypothesis, grasp::data::Item::Map::iterator input);

		/** Computes computes collision bounds given an input itemImage */
		grasp::CollisionBounds::Ptr selectCollisionBounds(bool draw, grasp::data::Item::Map::const_iterator input);



	protected:
		golem::CriticalSection csRenderer;
		golem::DebugRenderer debugRenderer;

		/**Input items */
		grasp::data::Item::Map::iterator predModelItem;
		grasp::data::Item::Map::iterator predQueryItem;
		grasp::data::Item::Map::iterator pointCurvItem;
		grasp::data::Item::Map::iterator trajectoryItem;

		/**Internal control flags*/
		bool hasPointCurv, hasPredModel, hasPredQuery, hasTrajectory;

		/** Parameters */
		Parameters params;

		/** Final output  */
		Result result;


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
		Random number generator (used for uniform direction generation)
		*/
		golem::Rand rand;





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
		ActiveSense::Ptr activeSense;

		/** Initializes ActiveSense */
		virtual void initActiveSense(pacman::Demo* demoOwner);

		/** Sends Boris' left arm to a pose in workspace coordinates */
		//e.g. good error: lin=0.000000065, ang=0.000002688
		virtual bool gotoPoseWS(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001) = 0;
		virtual bool gotoPoseConfig(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001) = 0;
		/** Captures an image generating a point cloud represented by ImageItems.
		It should add all scanned Image Items to scannedImageItems
		*/
		virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems,
			ScanPoseActiveCommand scanPoseCommand = nullptr,
			const std::string itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL) = 0;

		/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
		virtual void postprocess(golem::SecTmReal elapsedTime) = 0;

		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			this->activeSense->assertValid(ac);
		}

		/** Load from xml context */
		void load(const golem::XMLContext* xmlcontext);

	};
};

#endif // _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_

