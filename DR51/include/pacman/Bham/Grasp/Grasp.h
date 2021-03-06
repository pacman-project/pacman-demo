/** @file Grasp.h
 * 
 * Bham Grasp interface
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_GRASP_GRASP_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_GRASP_GRASP_H_

#include <pacman/PaCMan/Defs.h>
#include <boost/shared_ptr.hpp>
#include <string>

/** PaCMan name space */
namespace pacman {
	/** Birmingham grasp interface */
	class BhamGrasp {
	public:
		/** Pointer */
		typedef boost::shared_ptr<BhamGrasp> Ptr;

		/** Weighted approach trajectory */
		class Trajectory {
		public:
			/** */
			typedef std::vector<Trajectory> Seq;

			/** Trajectory */
			SchunkDexHand::Pose::Seq trajectory;
			/** Likelihood */
			float_t likelihood;

			/** Default constructor sets the default configuration. */
			inline Trajectory() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				trajectory.clear();
				likelihood = float_t(1.);
			}
		};

		/** Creates Birmingham grasp
		 *	@param[in]	path			configuration file
		*/
		static Ptr create(const std::string& path);
		
		/**	Loads grasp data
		 *	@param[in]	path			grasp data file
		*/
		virtual void load(const std::string& path) = 0;

		/** Estimate possible grasps together with their with approach trajectories from a given point cloud
		 *	@param[in]	points			query point cloud
		 *	@param[out]	trajectories	weighted approach trajectories, each tajectory consists of at least two waypoints with the last waypoint defining a grip
		*/
		virtual void estimate(const Point3D::Seq& points, Trajectory::Seq& trajectories) = 0;

		/** Process messages
		*/
		virtual void spin() = 0;

		/** Destruction
		*/
		virtual ~BhamGrasp() {}
	};


	/** Point cloud save */
	void save(const std::string& path, const Point3D::Seq& points);
	/** Point cloud load */
	void load(const std::string& path, Point3D::Seq& points);
	/** Trajectory save */
	void save(const std::string& path, const RobotUIBK::Config::Seq& trajectory);
	/** Trajectory load */
	void load(const std::string& path, RobotUIBK::Config::Seq& trajectory);
};

#endif // _PACMAN_BHAM_GRASP_GRASP_H_