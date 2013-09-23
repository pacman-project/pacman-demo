/** @file PoseEstimation.h
 * 
 * Innsbruck pose estimation interface
 *
 */

#pragma once
#ifndef _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_H_ // if #pragma once is not supported
#define _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_H_

#include <pacman/PaCMan/Defs.h>
#include <string>

/** PaCMan name space */
namespace pacman {
	/** Innsbruck object database interface */
	class UIBKObject {
	public:
		/** Creates Innsbruck object database
		 *	@param[in]	path			configuration file
		*/
		static UIBKObject* create(const std::string& path);

		/**	Adds a single object
		 *	@param[in]	id				unique object identifier
		 *	@param[in]	points			object point cloud
		*/
		virtual void add(const std::string& id, const Point3D::Seq& points) = 0;

		/**	Retrieves a single object
		 *	@param[in]	id				unique object identifier
		 *	@param[out]	points			object point cloud
		*/
		virtual void get(const std::string& id, Point3D::Seq& points) const = 0;

		/** Removes a given object
		 *	@param[in]	id				object to remove
		*/
		virtual void remove(const std::string& id) = 0;

		/** Lists all objects
		 *	@param[out]	id				list of all objects
		*/
		virtual void list(std::vector<std::string>& idSeq) const = 0;
	};

	/** Innsbruck pose estimation interface */
	class UIBKPoseEstimation {
	public:
		/** Object pose */
		class Pose {
		public:
			/** */
			typedef std::vector<Pose> Seq;

			/** 3D pose */
			Mat34 pose;
			/** Object identifier */
			std::string id;
			/** Weight */
			float_t weight;

			/** Default constructor sets the default configuration. */
			inline Pose() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				pose.setToDefault();
				id.clear();
				weight = float_t(1.);
			}
		};

		/** Creates Innsbruck pose estimation
		 *	@param[in]	path			configuration file
		*/
		static UIBKPoseEstimation* create(const std::string& path);
		
		/**	Captures point cloud from Kinect
		 *	@param[out]	points			object point cloud
		*/
		virtual void capture(Point3D::Seq& points) = 0;

		/** Find list of objects with their poses from a given point cloud
		 *	@param[in]	points			query point cloud
		 *	@param[out]	poses			estimated set of object poses
		*/
		virtual void estimate(const Point3D::Seq& points, Pose::Seq& poses) = 0;
	};
};

#endif // _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_H_