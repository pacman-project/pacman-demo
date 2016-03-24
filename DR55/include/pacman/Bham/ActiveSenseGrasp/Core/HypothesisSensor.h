/** @file HypothesisSensor.h
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#pragma once
#ifndef _PACMAN_BHAM_ACTIVESENS_HYPOTHESIS_SENSOR_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVESENS_HYPOTHESIS_SENSOR_H_

#include <Grasp/App/Player/Player.h>
#include <Golem/UI/Data.h>

#include <opencv2/core/core.hpp>

#include "ActiveSense/Core/Model.h"

#include <sstream>

namespace pacman{



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


        std::string getLabel(){

            std::stringstream ss;
            ss << "ActiveSense_ItemFromHypothesis" << id;
            return ss.str();
        }

        inline int getId(){
            return id;
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
		HypothesisSensor(HypothesisSensor::Config config, golem::RGBA shapeColour = golem::RGBA::CYAN, int plannerIdx = 0);

		/** Set OpenGL view point to *this sensor's frame view point*/
		void setGLView(golem::Scene& scene);

		/** Set OpenGL view point to the sensor's frame view point */
		static void setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame);


		/** visited flag */
		bool visited;

        golem::Real value;

        static int next_id;
        int id;
		int plannerIdx;

        static bool compareHypothesisSensor(HypothesisSensor::Ptr h1, HypothesisSensor::Ptr h2);
	};









    class PinholeCamera
    {
    public:

        //Extrinsic Params
        cv::Mat R;
        cv::Mat T;

        int w, h, x, y;


        //Intrinsic params: Perspective matrix (Projection matrix)
        cv::Mat K;

        //Frustum limits
        float n, f, r, l, b, t; //near, far, right, left bottom, top

        PinholeCamera();

        //Loads the camera model with a default frustum, and the following extrinsic params taken as input
        PinholeCamera(cv::Vec3f rvector, cv::Vec3f translation);

        ~PinholeCamera(void);

        //Load a rotation Matrix from a rotation vector v = (rx, ry, rz) which indicates how much the coordinates will be rotate in each axis.
        void loadRotationMatrix(cv::Vec3f rvector);

        //Load a rotation Matrix directly
        void loadRotationMatrix(cv::Mat& R);
        void loadRotationMatrix(golem::Mat33& R);

        //Load translation given as a vector or as a Mat
        void loadTranslation(cv::Vec3f translation);
        void loadTranslation(cv::Mat& translation);
        void loadTranslation(golem::Vec3& translation);


        //Sets the perspective matrix (Projection Matrix) according to the following Frustum limits
        void setFrustum(float l, float r, float b, float t, float n, float f);

        // Primesense ranges:
        // FOV: 58° H, 45° V, 70° D (Horizontal, Vertical, Diagonal)
        // Distances: Between 0.8m and 3.5m
        void setFrustum(float fov_y, float aspect_ratio, float near, float far);

        void transform(active_sense::Model::Voxel::Seq& model, std::vector<cv::Mat> & transformedPoints, std::vector<bool>& clip_mask, bool negate_mask = false);
        cv::Point3f viewportTransform( const cv::Point3f& point );

        cv::Mat getRT();


    };
};

#endif
