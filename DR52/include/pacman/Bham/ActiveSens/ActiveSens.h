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

/** PaCMan name space */
namespace pacman {

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
		void setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame);
	};

	class ActiveSense {



	public:
		ActiveSense() {}
		ActiveSense(golem::Context& context);

		/** Uniformly generates a set of nsamples sensor hypotheses (viewHypotheses) around a sphere with specified radius centered at centroid in workspace coordinates */
		void generateRandomViews(pacman::HypothesisSensor::Seq& viewHypotheses, const golem::Vec3& centroid, const golem::I32& nsamples = 5, const golem::Real& radius = 0.25);
		
		/** Gets current view hypotheses a.k.a. sensor hypotheses*/
		pacman::HypothesisSensor::Seq& getViewHypotheses() {
			return this->viewHypotheses;
		}

		/** Gets current view hypotheses a.k.a. sensor hypotheses*/
		pacman::HypothesisSensor::Ptr getViewHypothesis(golem::U32 index) {
			grasp::Assert::valid(index < this->viewHypotheses.size(), "this->viewHypotheses[index]");
			return this->viewHypotheses[index];
		}
	protected:

		golem::Rand rand;
		pacman::HypothesisSensor::Seq viewHypotheses;
		golem::I32 currentViewHypothesis;
		golem::I32 selectedViewHypothesis;

		

	};




};

#endif // _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_H_