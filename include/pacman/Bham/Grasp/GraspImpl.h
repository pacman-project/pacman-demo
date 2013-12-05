/** @file GraspImpl.h
 * 
 * Bham Grasp interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_GRASP_GRASP_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_GRASP_GRASP_IMPL_H_

#include <pacman/Bham/Grasp/Grasp.h>
#include <Grasp/ShapePlanner/ShapePlanner.h>

/** PaCMan name space */
namespace pacman {
	/** Birmingham grasp interface implementation */
	class BhamGraspImpl : public BhamGrasp, public grasp::ShapePlanner {
	public:
		/** Construct Birmingham grasp interface */
		class Desc : public grasp::ShapePlanner::Desc {
		protected:
			CREATE_FROM_OBJECT_DESC1(BhamGraspImpl, golem::Object::Ptr, golem::Scene&)
		};

		/**	Adds a single grasp example */
		virtual void add(const std::string& id, const Point3D::Seq& points, const RobotUIBK::Config::Seq& trajectory);

		/** Removes a given grasp example */
		virtual void remove(const std::string& id);

		/** Lists all grasp examples */
		virtual void list(std::vector<std::string>& idSeq) const;

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories);

		/** Process messages */
		virtual void spin();

		/** Point cloud conversion */
		void convert(const ::grasp::Point::Seq& src, Point3D::Seq& dst) const;
		/** Point cloud conversion */
		void convert(const Point3D::Seq& src, ::grasp::Point::Seq& dst) const;
		/** Waypoint conversion */
		void convert(const ::grasp::Manipulator::Config& src, ShunkDexHand::Config& dst) const;
		/** Waypoint conversion */
		void convert(const ShunkDexHand::Config& src, ::grasp::Manipulator::Config& dst) const;
		/** Trajectory conversion */
		void convert(const ::grasp::RobotState::List& src, RobotUIBK::Config::Seq& dst) const;
		/** Trajectory conversion */
		void convert(const RobotUIBK::Config::Seq& src, ::grasp::RobotState::List& dst) const;

		void addLine(const pacman::Vec3& p1, const pacman::Vec3& p2) {
			debugRenderer.addLine(golem::Vec3(p1.x, p1.y, p1.z), golem::Vec3(p2.x, p2.y, p2.z), golem::RGBA::GREEN);
			const golem::Real length = golem::Math::sqrt(golem::Math::sqr(p2.x - p1.x) + golem::Math::sqr(p2.y - p1.y) + golem::Math::sqr(p2.z - p1.z));
			printf("Line: length=%f, p1=(%f, %f, %f), p2=(%f, %f, %f)\n", length, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
		}
		void addFrame(const pacman::Mat34& m) {
			golem::Mat34 frame;
			frame.p.set(&m.p.x);
			frame.R.setRow33(&m.R.m11);
			debugRenderer.addAxes3D(frame, golem::Vec3(0.15));
			printf("Frame: p=(%f, %f, %f), R = {(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		}

	protected:
		BhamGraspImpl(golem::Scene &scene);
		bool create(const grasp::ShapePlanner::Desc& desc);

		/** User interface: menu function */
		virtual void function(grasp::TrialData::Map::iterator& dataPtr, int key);

		golem::DebugRenderer debugRenderer;
		virtual void render() {
			grasp::ShapePlanner::render();
			debugRenderer.render();
		}
	};
};

#endif // _PACMAN_BHAM_GRASP_GRASP_IMPL_H_