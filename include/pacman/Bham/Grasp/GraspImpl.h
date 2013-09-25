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
		virtual void add(const std::string& id, const Point3D::Seq& points, const ShunkDexHand::Pose::Seq& trajectory);

		/** Removes a given grasp example */
		virtual void remove(const std::string& id);

		/** Lists all grasp examples */
		virtual void list(std::vector<std::string>& idSeq) const;

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories);

		/** Process messages */
		virtual void spin();

	protected:
		BhamGraspImpl(golem::Scene &scene);
		bool create(const grasp::ShapePlanner::Desc& desc);

		/** User interface: menu function */
		virtual void function(grasp::TrialData::Map::iterator& dataPtr, int key);
		/** Point cloud conversion */
		void convert(const Point3D::Seq& src, ::grasp::Point::Seq& dst) const;
		/** Point cloud conversion */
		void convert(const ::grasp::Point::Seq& src, Point3D::Seq& dst) const;
	};
};

#endif // _PACMAN_BHAM_GRASP_GRASP_IMPL_H_