/** @file GraspImpl.h
 * 
 * Bham Grasp interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_GRASP_GRASP_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_GRASP_GRASP_IMPL_H_

#include <Grasp/ShapePlanner/ShapePlanner.h>
#include <pacman/Bham/Grasp/Grasp.h>

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

		/**	Loads grasp data */
		virtual void load(const std::string& path);

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories);

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud (version with curvatures) */
		void estimate(const ::grasp::Cloud::PointSeq& points, Trajectory::Seq& trajectories);

		/** Process messages */
		virtual void spin();

		/** Point cloud conversion */
		void convert(const ::grasp::Cloud::PointSeq& src, Point3D::Seq& dst) const;
		/** Point cloud conversion */
		void convert(const Point3D::Seq& src, ::grasp::Cloud::PointSeq& dst) const;
		/** Waypoint conversion */
		void convert(const ::grasp::Manipulator::Config& src, SchunkDexHand::Config& dst) const;
		/** Waypoint conversion */
		void convert(const SchunkDexHand::Config& src, ::grasp::Manipulator::Config& dst) const;
		/** Trajectory conversion */
		void convert(const ::grasp::Robot::State::List& src, RobotUIBK::Config::Seq& dst) const;
		/** Trajectory conversion */
		void convert(const RobotUIBK::Config::Seq& src, ::grasp::Robot::State::List& dst) const;

	protected:
		BhamGraspImpl(golem::Scene &scene);
		bool create(const grasp::ShapePlanner::Desc& desc);

		/** User interface: menu function */
		virtual void function(Data::Map::iterator& dataPtr, int key);
	};
};

#endif // _PACMAN_BHAM_GRASP_GRASP_IMPL_H_