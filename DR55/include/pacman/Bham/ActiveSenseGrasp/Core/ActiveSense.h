/** @file ActiveSense.h
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#pragma once
#ifndef _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_


#include "ActiveSense/Core/utils3d.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/HypothesisSensor.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSenseOM.h"
#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSenseOM2.h"



/** PaCMan name space */
namespace pacman {

	class ActiveSenseDemo;
	//------------------------------------------------------------------------------
	

	class ActiveSense {

	public:

        enum TransformType {
            IMAGE,
            POINT_CURV,
            CONTACT_MODEL,
            CONTACT_QUERY,
            IMAGE_NOCROP,
            UNKNOWN
        };

		enum EStoppingCriteria {

			C_NVIEWS,
			C_COVERAGE,
			C_NVIEWS_COVERAGE,
			C_NONE //Used for validity check

		};

        enum ESafetyStoppingCriteria {
            ES_ENTROPY,
            ES_NVIEWS,
            ES_NONE
        };

		enum ECoverageMethod {

			M_AREA_BASED,
			M_VOLUME_BASED,
			M_NONE //Used for validity check

		};

		enum ESelectionMethod {

			S_RANDOM,
			S_CONTACT_BASED,
			S_CONTACT_BASED2,
            S_CONTACT_BASED3,
            S_INFORMATION_GAIN,
			S_SEQUENTIAL,
			S_NONE //Used for validity check

		};

		enum EGenerationMethod {

			G_RANDOM_SPHERE,
			G_FIXED,
			G_NONE //Used for validity check

		};
        typedef std::map<std::string, EStoppingCriteria> StoppingCriteriaMap;
        typedef std::map<std::string, ESafetyStoppingCriteria> SafetyStoppingCriteriaMap;
        typedef std::map<std::string, ECoverageMethod> CoverageMethodMap;
        typedef std::map<std::string, ESelectionMethod> SelectionMethodMap;
        typedef std::map<std::string, EGenerationMethod> GenerationMethodMap;



		class Parameters {
		public:

			typedef golem::shared_ptr<Parameters> Ptr;

			Parameters() {
				this->setToDefault();
			}

            SelectionMethodMap getSelectionMethodMap() const {

                SelectionMethodMap retMap;

				retMap["random"] = ESelectionMethod::S_RANDOM;
				retMap["contact_based"] = ESelectionMethod::S_CONTACT_BASED;
				retMap["contact_based_v2"] = ESelectionMethod::S_CONTACT_BASED2;
                retMap["contact_based_v3"] = ESelectionMethod::S_CONTACT_BASED3;
                retMap["information_gain"] = ESelectionMethod::S_INFORMATION_GAIN;
				retMap["sequential"] = ESelectionMethod::S_SEQUENTIAL;

				return retMap;

			}

            GenerationMethodMap getGenerationMethodMap() const {

                GenerationMethodMap retMap;

				retMap["fixed"] = EGenerationMethod::G_FIXED;
				retMap["random_sphere"] = EGenerationMethod::G_RANDOM_SPHERE;


				return retMap;

			}

            CoverageMethodMap getCoverageMethodMap() const {

                CoverageMethodMap retMap;

				retMap["area_based"] = ECoverageMethod::M_AREA_BASED;
				retMap["volume_based"] = ECoverageMethod::M_VOLUME_BASED;


				return retMap;

			}

            StoppingCriteriaMap getStoppingCriteriaMap() const {
                StoppingCriteriaMap retMap;

				retMap["number_of_views"] = EStoppingCriteria::C_NVIEWS;
				retMap["coverage"] = EStoppingCriteria::C_COVERAGE;
				retMap["number_of_views_and_coverage"] = EStoppingCriteria::C_NVIEWS_COVERAGE;


				return retMap;
			}

            SafetyStoppingCriteriaMap getSafetyStoppingCriteriaMap() const {
                SafetyStoppingCriteriaMap retMap;

                retMap["entropy"] = ESafetyStoppingCriteria::ES_ENTROPY;
                retMap["number_of_views"] = ESafetyStoppingCriteria::ES_NVIEWS;

                return retMap;
            }


			/** Number of hypothesis samples */
            golem::U32 nsamples, nviews, maxnviews, maxsafetyviews;

	

			/** Radius of the sphere defines the distance from object centroid from which sensor hypothesis are going to be generated */
			golem::Real radius;

			/**
			Centroid around random generated view directions are generated
			*/
			golem::Vec3 centroid;

			/**
			Defines whether this->centroid is going to be manually set or computed from a image point cloud item
			*/
			bool useManualCentroid, useHeightBias, regenerateViews, filterPlane;
			/** Min and Max values for biased hypothesis generation (heuristics for minimizing shade) */
			golem::Real minPhi, maxPhi, minTheta, maxTheta; //Param

            golem::U32 selectionMethod, alternativeSelectionMethod, generationMethod, coverageMethod, stoppingCriteria, safetyStoppingCriteria;

			/** Coverage threshold for stopping criteria */
            golem::Real coverageThr, entropyThr;

            bool useSimCam;
			std::string contactHandler, queryHandler, imageHandler, imageHandlerNoCrop, pointCurvHandler;


			/** Configuration sequence */
			HypothesisSensor::Config::Seq configSeq;

			grasp::ConfigMat34 dropOffPose;

			/** Sensor to be used for active control */
			std::string sensorId;

            /** Show sensor hypotheses */
            bool showSensorHypotheses;

			void setToDefault() {
				this->radius = 0.35;
				this->nsamples = 5;
                this->nviews = 2;
                this->maxnviews = 7;
                this->maxsafetyviews = 3;
				this->coverageThr = 0.8;
				this->useManualCentroid = false;
				this->centroid.setZero();
				this->minPhi = 30.0;
				this->maxPhi = 150.0;
				this->minTheta = 45.0;
				this->maxTheta = 135.0;

                this->entropyThr = 100;

				this->selectionMethod = ESelectionMethod::S_CONTACT_BASED;
				this->alternativeSelectionMethod = ESelectionMethod::S_RANDOM;
				this->generationMethod = EGenerationMethod::G_RANDOM_SPHERE;
				this->coverageMethod = ECoverageMethod::M_AREA_BASED;
				this->stoppingCriteria = EStoppingCriteria::C_NVIEWS;
                this->safetyStoppingCriteria = ESafetyStoppingCriteria::ES_ENTROPY;
				this->filterPlane = false;

				this->sensorId = "OpenNI+OpenNI";

				this->contactHandler = "ContactModel+ActiveSenseGraspDataContactModel";
				this->queryHandler = "ContactQuery+ActiveSenseGraspDataContactQuery";
				this->imageHandler = "Image+ActiveSenseGraspDataImage";
				this->imageHandlerNoCrop = "Image+ActiveSenseGraspDataImageNoCrop";
				this->pointCurvHandler = "PointsCurv+ActiveSenseGraspDataPointsCurv";

				configSeq.clear();

				dropOffPose.setToDefault();

                this->showSensorHypotheses = true;
                this->useSimCam = false;
			}

			/** Assert that the object is valid. */
			void assertValid(const grasp::Assert::Context& ac) const {
				grasp::Assert::valid(this->nsamples > 0, ac, "nsamples: <= 0");
				grasp::Assert::valid(this->nviews > 0, ac, "nviews: <= 0");
				grasp::Assert::valid(this->radius > 0, ac, "radius: <= 0");
				grasp::Assert::valid(this->selectionMethod != ESelectionMethod::S_NONE, ac, "Selection Method: is S_NONE (unknown selection method)");
				grasp::Assert::valid(this->alternativeSelectionMethod != ESelectionMethod::S_NONE, ac, "Alternative Selection Method: is S_NONE (unknown selection method)");
				grasp::Assert::valid(this->generationMethod != EGenerationMethod::G_NONE, ac, "Generation Method: is G_NONE (unknown generation method)");
				grasp::Assert::valid(this->coverageMethod != ECoverageMethod::M_NONE, ac, "Coverage Method: is M_NONE (unknown coverage method)");
				grasp::Assert::valid(this->stoppingCriteria != EStoppingCriteria::C_NONE, ac, "Stopping Criteria: is C_NONE (unknown stopping criteria)");
                grasp::Assert::valid(this->stoppingCriteria != ESafetyStoppingCriteria::ES_NONE, ac, "Safety Stopping Criteria: is ES_NONE (unknown safety stopping criteria)");
                grasp::Assert::valid(!configSeq.empty(), ac, "need at least one fixed camera pose in parameters (for initial view)");
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

            std::vector<std::pair<grasp::data::Item::Ptr, float> > completeTrajectories;

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
		ActiveSense(pacman::ActiveSenseDemo* demoOwner);

		/**
		ActiveSense Default Constructor
		Warning: *this needs to be initialised before use
		*/
        ActiveSense() : dataPath(ActiveSense::DFT_DATA_PATH), demoOwner(nullptr), hasPointCurv(false), hascontactModel(false), hasTrajectory(false), allowInput(false), seqIndex(0) {}

		/**
		ActiveSense initialiser (Same function as Constructor)
		*/
		void init(pacman::ActiveSenseDemo* demoOwner);

		/** Generates views according to current this->params setup */
		void generateViews();

		/** Generates a set of nsamples sensor hypotheses (viewHypotheses) with uniformly distributed view directions (viewDir) around a sphere with specified radius centered at centroid in workspace coordinates */
		void generateRandomViews();

		/** Generates view from ConfigMat34 according to the forward kinematic parameters of the robot and selected sensor_id */
		pacman::HypothesisSensor::Ptr generateViewFrom(const HypothesisSensor::Config& config);

		/** Generate Views given a sequence of predefined configurations */
		void generateViewsFromSeq(const HypothesisSensor::Config::Seq& configSeq);

		/**
		Generates next random view with uniformly generated view direction positioned at a distance of radius from sphere centroid
		*/
		pacman::HypothesisSensor::Ptr generateNextRandomView(const golem::Vec3& centroid, const golem::Real& radius = 0.25, bool heightBias = true);
        pacman::HypothesisSensor::Ptr generateSphericalView(const golem::Vec3& centroid, const golem::Real& radius, const golem::Real& phi, const golem::Real& theta);
        void generateSphericalSteppedViews(const golem::Real& nPhi, const golem::Real& nTheta);
		/** Gaze ActiveSense main method, automatically uses selection and generation methods/approaches set in this->params
		Approach/Method i) Random view selection
		Approach/Method ii) Contact Based view selection
		Approach/Method iii) Sequential

        \return final Item with ItemPointCurv representing integrated random scans
		*/
		grasp::data::Item::Map::iterator nextBestView();
        grasp::data::Item::Map::iterator nextBestView2();
        grasp::data::Item::Map::iterator nextBestView3();

        grasp::data::Item::Map::iterator collectData();
        bool checkStop(int numViewsAcquired, int maxNumViews);


		/** Gaze ActiveSense main method, approach (i)
		Approach i) Random view selection

        \return final Item with ItemPointCurv representing integrated random scans
		*/
		
        pacman::HypothesisSensor::Ptr selectNextBestView(grasp::data::Item::Map::iterator contactModelPtr, bool found_contacts = true);
        pacman::HypothesisSensor::Ptr selectNextBestView(int selectionMethod, grasp::data::Item::Map::iterator contactModelPtr);

		/** Greedy selection for next best view based on contact point information */
        pacman::HypothesisSensor::Ptr selectNextBestViewContactBased(grasp::data::Item::Map::iterator contactModelPtr);

		/** Greedy selection for next best view based on contact point information VERSION 2 with online model */
        pacman::HypothesisSensor::Ptr selectNextBestViewContactBased2(grasp::data::Item::Map::iterator contactModelPtr);

        /** Greedy selection for next best view based on contact point information VERSION 3 with online model */
        pacman::HypothesisSensor::Ptr selectNextBestViewContactBased3(grasp::data::Item::Map::iterator contactModelPtr);

        pacman::HypothesisSensor::Ptr selectNextBestViewInfGain();

		/** Selects next best view randomly from the sequence of generated this->viewHypotheses*/
		pacman::HypothesisSensor::Ptr selectNextBestViewRandom();

		/** Selects next best view sequentially*/
		pacman::HypothesisSensor::Ptr selectNextBestViewSequential();

        //Returns if it is safe to execute trajectory
        //
        bool safetyExploration();
        bool safetyExploration2(grasp::data::Trajectory* trajectory, grasp::data::Item::List& scannedImageItems, std::vector<std::pair<grasp::data::Item::Ptr, float> >& bestTrajectories);

        grasp::data::Trajectory* getSafestTrajectoryToDate();

        void resetNextBestViewSequential() { seqIndex = 0; }
		void incrNextBestViewSequential() { seqIndex = (seqIndex + 1) % viewHypotheses.size(); }

		void setAllowInput(bool allowInput)
		{
			this->allowInput = allowInput;
		}

		/** Gets current view hypotheses a.k.a. sensor hypotheses*/
		pacman::HypothesisSensor::Seq& getViewHypotheses() {
			return this->viewHypotheses;
		}

		bool hasViewed(pacman::HypothesisSensor::Ptr hypothesis) {
			for (int i = 0; i < this->visitedHypotheses.size(); i++)
			{	
				//If distance is less than or equal 5 centimeters...
				if (hypothesis->getFrame().p.distance(visitedHypotheses[i]->getFrame().p) <= 0.05)
					return true;
			}
			return false;
		}

		/** Gets current view hypotheses a.k.a. sensor hypothesis*/
		pacman::HypothesisSensor::Ptr getViewHypothesis(golem::U32 index) {
			grasp::Assert::valid(index < this->viewHypotheses.size(), "this->viewHypotheses[index]");
			return this->viewHypotheses[index];
		}

		/**
		Gets crictical section lock for viewHypotheses vector member attribute of *this
        \return golem::CriticalSection
		*/
		golem::CriticalSection& getCSViewHypotheses()
		{
			return this->csViewHypotheses;
		}


		/**
		Sets random generator seed
        \return Nothing
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
        \return Nothing
		*/
        void setContactModelItem(grasp::data::Item::Map::iterator contactModelItem)
		{
            this->contactModelItem = contactModelItem;
            this->hascontactModel = true;
		}

		/**
		Sets current online model (pointCurv member attribute)
        \return Nothing
		*/
		void setPointCurvItem(grasp::data::Item::Map::iterator pointCurv)
		{
			this->pointCurvItem = pointCurv;
			this->hasPointCurv = true;
		}

		/**
        Sets current contactQuery item
        \return Nothing
		*/
        void setcontactQueryItem(grasp::data::Item::Map::iterator contactQueryItem)
		{
            this->contactQueryItem = contactQueryItem;
            this->hascontactQuery = true;
		}

		/**
		Sets current trajectory item
        \return Nothing
		*/
		void setTrajectoryItem(grasp::data::Item::Map::iterator trajectoryItem)
		{
			this->trajectoryItem = trajectoryItem;
			this->hasTrajectory = true;
		}

		/**
		Shortcut for setting centroid parameter using an ItemImage Point Cloud
        \return Nothing
		*/
		void setCentroidFromItemImage(grasp::data::Item::Map::iterator itemImage)
		{
			this->params.centroid = this->computeCentroid(itemImage);
		}

		/**
		Shortcut for setting useManualCentroid parameter
        \return Nothing
		*/
		void setUseManualCentroid(bool useManualCentroid)
		{
			this->params.useManualCentroid = useManualCentroid;
		}

		/**
		Sets parameters for *this
        \return Nothing
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
        \return this->params
		*/
		Parameters& getParameters(){
			return this->params;
		}

		/**
		Gets Parameters of *this
		*/
		const Result& getResult(){
			this->result;
		}
		/**
		Computes goal frame such that i) camera is aligned with targetFrame
		Input: targetFrame, camera (to be aligned)
        \return goal frame respecting statement (i)
		*/
        golem::Mat34 computeGoal(const golem::Mat34& targetFrame, grasp::Camera* camera);
        golem::Mat34 getCameraPose();

		/* Renders stuff */
		virtual void render() const;

		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			this->params.assertValid(ac);
		}

		/** Load from xml context */
		void load(const golem::XMLContext* xmlcontext);

		/** Execute Grasp Trajectory (modification of Play trajectory */
		void executeTrajectory();
        void executeTrajectory2();
        //int findSafestTrajectory();


        ActiveSensOnlineModel2& getOnlineModel2(){
            return this->onlineModel2;
        }


        TransformMap& getTransformMap(){
            return this->transformMap;
        }

    public:
		/** Ad-hoc Tools for cloud processing: Aux Types **/
        typedef grasp::Cloud::PointSeq CloudT;
        typedef grasp::Cloud::Point PointT;


        /************************ Utils Player Commands *************************************************/
		/**
		Adds item and its corresponding label to demoOwner->dataCurrentPtr->itemMap
		*/
		grasp::data::Item::Map::iterator addItem(const std::string& label, grasp::data::Item::Ptr item);
        grasp::data::Item::Map::iterator addItem(const std::string& label, grasp::data::Item::Ptr& item, grasp::Manager::Data::Ptr dataBundle);
		/**
		Removes list of items from demoOwner->dataCurrentPtr->second->itemMap
		*/
		void removeItems(grasp::data::Item::List& list);

        void removeItem(grasp::data::Item::Map::iterator itemPtr);

		/**
		Removes data bundle pointed by data from demoOwner->dataMap
		*/
		void removeData(grasp::data::Data::Map::iterator data);

        void saveData(const grasp::Manager::Data& data);

		/**
		Creates data bundle and makes it be the demoOwner->dataCurrentPtr
        \return Iterator of the newly created data bundle in demoOwner->dataMap
		*/
         grasp::Manager::Data::Ptr createData();
         void addData(grasp::Manager::Data::Ptr data);
        ///////////////////////////////////////////////////////////////////////////////////////////////////

        /************************ Utils Cloud Processing *************************************************/

        /**
        Computes centroid of the point cloud represented by itemImage
        \return Centroid of the point cloud represented by itemImage
        (undefined behaviour if it's not a pointcloud)
        */
        golem::Vec3 computeCentroid(grasp::data::Item::Map::const_iterator itemImage);
        golem::Vec3 computeCentroid(CloudT::Ptr cloudIn);

        /**
         * Computes coverage given a PointCurv point cloud either volume or area based as specified by this->params
         * \return coverage percetage
        **/
        golem::Real computeCoverage(grasp::Cloud::PointCurvSeq::Ptr cloudIn);
        ///////////////////////////////////////////////////////////////////////////////////////////////////

        /************************ UTILS Player Robot Sensors *********************************************/

		/**
		Gets DemoOwner's OPENNI Camera, and also sets it as its demoOwner->sensorCurrentPtr
        \return Pointer to Openni Camera Sensor
		*/
		grasp::CameraDepth* getOwnerOPENNICamera();

		/** Gets a sensor identified by its id=library+configFile, e.g. OpenNI+OpenNI, DepthSim+DepthSim*/
        grasp::Camera* getOwnerSensor(const std::string& sensorId);
        ///////////////////////////////////////////////////////////////////////////////////////////////////

        /************************ UTILS Player Items/Cloud Processing and Grasping ***********************/

		/**
		Processes list of itemImages
        \return integrated itemPointCurv
		*/
		grasp::data::Item::Map::iterator processItems(grasp::data::Item::List& list);

		/**
		Executes the following steps:
        0) Input: assumes we have already a contact model (contactModel), previously generate with pointcloud with normals + grasp trajectory
        1) Transforms (contactModelItem,pointCurvItem) into contactQuery;
        2) Converts contactQuery into a Trajectory (traj)
        3) feedback step => Transforms (pointCurvItem,traj) into a contactModelItem

        \return contactModelItem result of the feedBack transform
		*/
        grasp::data::Item::Map::iterator computeFeedBackTransform(grasp::data::Item::Map::iterator contactModelItem, grasp::data::Item::Map::iterator pointCurvItem);

		/**
        Returns a Contact Model by following the following steps:
        1) Converts Contact Query into a Trajectory (traj)
        2) feedback step => Transforms (pointCurvItem,traj) into a Contact Model
        \return
		*/
        grasp::data::Item::Map::iterator computeContactModelFeedBack(grasp::data::Item::Map::iterator contactQuery, grasp::data::Item::Map::iterator pointCurvItem);

		/**
		Returns a Predictor model by following the following steps:
		1) Transforms trajItem + pointCurvItem into a predictory model by applying appropriate transform
		*/
        grasp::data::Item::Map::iterator computeTransformcontactModel(grasp::data::Item::Map::iterator trajItem, grasp::data::Item::Map::iterator pointCurvItem);

		/**
        Converts a contactQueryModel into a Trajectory
		*/
        grasp::data::Item::Map::iterator convertToTrajectory(grasp::data::Item::Map::iterator contactQueryModel);

        /** Computes computes collision bounds given an input itemImage */
        grasp::CollisionBounds::Ptr selectCollisionBounds(bool draw, grasp::data::Item::Map::const_iterator input);

        grasp::data::Item::Map::iterator reduce(grasp::data::Item::List& list, TransformMap::value_type& transformPtr, const std::string& itemLabel);

        /************************ Active Sense Value Computation ***************************************/

		/** Computes the value of a hypothesis sensor given an ItemPointCurv as input */
		ValueTuple computeValue(HypothesisSensor::Ptr hypothesis, grasp::data::Item::Map::iterator input);
        /////////////////////////////////////////////////////////////////////////////////////////////////



	protected:
		golem::CriticalSection csRenderer;
		golem::DebugRenderer debugRenderer;

		/**Input items */
        grasp::data::Item::Map::iterator contactModelItem;
        grasp::data::Item::Map::iterator contactQueryItem;
		grasp::data::Item::Map::iterator pointCurvItem;
		grasp::data::Item::Map::iterator trajectoryItem;

		pacman::ActiveSensOnlineModel onlineModel;
        pacman::ActiveSensOnlineModel2 onlineModel2;

		/**Internal control flags*/
        bool hasPointCurv, hascontactModel, hascontactQuery, hasTrajectory;

		/** toggle allow input flag */
		bool allowInput;
		
		/** Internal control index for sequential selection*/
		golem::U32 seqIndex;

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
		pacman::ActiveSenseDemo* demoOwner;

		/**
		Sequence of Sensor's View Hypotheses visited so far (or to be visited)
		Important: Write and Read operations should be done using the lock csViewHypotheses
		*/
		pacman::HypothesisSensor::Seq viewHypotheses;

		/**
		Visited view hypotheses
		*/
		pacman::HypothesisSensor::Seq visitedHypotheses;

		/**
		Lock for this->viewHypotheses
		Important: Write and Read operations on this->viewHypotheses should be done using this lock
		*/
		golem::CriticalSection csViewHypotheses;


		/**
		Random number generator (used for uniform direction generation)
		*/
		golem::Rand rand;





	public:
        int experiment_trial, experiment_id;
        std::string experiment_alias;

        int nbvViews, safetyViews;
        FILE* out;

		// @@@ HACK @@@
		golem::Controller::State::Ptr pLastExecutedWaypoint;
		//bool lastExecutedWaypointValid = false;
	};


	/**
	ActiveSenseController defines a set of basic functions that we need to actively control Boris
	*/
	class ActiveSenseController {

	public:
		typedef std::function<bool()> ScanPoseActiveCommand;

		ActiveSenseController() {}

	protected:

		pacman::ActiveSenseDemo* demoOwner;

		/** ActiveSense Object, responsible for Gaze Selection */
		ActiveSense::Ptr activeSense;

		/** Initializes ActiveSense */
		virtual void initActiveSense(pacman::ActiveSenseDemo* demoOwner);

		/** Sends Boris' left arm to a pose in workspace coordinates */
		//e.g. good error: lin=0.000000065, ang=0.000002688
		virtual bool gotoPoseWS(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001) = 0;
		virtual bool gotoPoseConfig(const grasp::ConfigMat34& pose, const golem::Real& linthr = 0.0000001, const golem::Real& angthr = 0.0000001) = 0;
		/** Captures an image generating a point cloud represented by ImageItems.
		It should add all scanned Image Items to scannedImageItems
		*/
		virtual void scanPoseActive(grasp::data::Item::List& scannedImageItems,
            const std::string& itemLabel = ActiveSense::DFT_IMAGE_ITEM_LABEL,
            const std::string& handler = std::string(),
            ScanPoseActiveCommand scanPoseCommand = nullptr,
            grasp::Manager::Data::Ptr dataPtr = grasp::Manager::Data::Ptr()) = 0;

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
