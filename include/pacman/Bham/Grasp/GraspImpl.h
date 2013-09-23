/** @file GraspImpl.h
 * 
 * Bham Grasp interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_GRASP_GRASP_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_GRASP_GRASP_IMPL_H_

#include <pacman/Bham/Grasp/Grasp.h>

/** PaCMan name space */
namespace pacman {
	/** Birmingham grasp interface implementation */
	class BhamGraspImpl : public BhamGrasp {
	public:
		/** Construct from configuration file */
		BhamGraspImpl(const std::string& path);

		/**	Adds a single grasp example */
		virtual void add(const std::string& id, const Point3D::Seq& points, const ShunkDexHand::Pose::Seq& trajectory);

		/** Removes a given grasp example */
		virtual void remove(const std::string& id);

		/** Lists all grasp examples */
		virtual void list(std::vector<std::string>& idSeq) const;

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories);
	};
};

#endif // _PACMAN_BHAM_GRASP_GRASP_IMPL_H_