/** @file Grasp.h
 * 
 * Bham Grasp interface
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_GRASP_GRASP_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_GRASP_GRASP_H_

#include <pacman/PaCMan/Defs.h>
#include <string>

/** PaCMan name space */
namespace pacman {
	/** Birmingham grasp interface */
	class BhamGrasp {
	public:
		/** Creates Birmingham grasp
		 *	@param[in]	path			configuration file
		*/
		static BhamGrasp* create(const std::string& path);
		
		/**	Adds a single grasp example
		 *	@param[in]	id				unique grasp identifier
		 *	@param[in]	points			point cloud particular to a grasp
		 *	@param[in]	trajectory		the grasp approach tajectory consists of at least two waypoints with the last waypoint defining a grip
		*/
		virtual void add(const std::string& id, const Point3D::Seq& points, const ShunkDexHand::Pose& trajectory) = 0;

		/** Removes a given grasp example
		 *	@param[in]	id				grasp identifier to remove
		*/
		virtual void remove(const std::string& id) = 0;

		/** Lists all grasp examples
		 *	@param[out]	id				grasp identifier to remove
		*/
		virtual void list(std::vector<std::string>& idSeq) const = 0;

		/** Process grasp examples
		*/
		virtual void process() = 0;

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud
		 *	@param[in]	points			query point cloud
		 *	@param[out]	trajectories	estimated trajectories, each tajectory consists of at least two waypoints with the last waypoint defining a grip
		 *	@param[out]	weights			associated trajectory weights
		*/
		virtual void estimate(const Point3D::Seq& points, ShunkDexHand::Pose::Seq& trajectories, std::vector<float_t>& weights) = 0;
	};
};

#endif // _PACMAN_BHAM_GRASP_GRASP_H_