/** @file GraspImpl.h
 * 
 * Bham Grasp interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_

#include <pacman/UIBK/PoseEstimation/PoseEstimation.h>

/** PaCMan name space */
namespace pacman {
	/** Innsbruck object database interface implementation */
	class UIBKObjectImpl : public UIBKObject {
	public:
		/** Construct from configuration file */
		UIBKObjectImpl(const std::string& path);

		/**	Adds a single object */
		virtual void add(const std::string& id, const Point3D::Seq& points);

		/**	Retrieves a single object */
		virtual void get(const std::string& id, Point3D::Seq& points) const;

		/** Removes a given object */
		virtual void remove(const std::string& id);

		/** Lists all objects */
		virtual void list(std::vector<std::string>& idSeq) const;
	};

	/** Innsbruck pose estimation interface */
	class UIBKPoseEstimationImpl : public UIBKPoseEstimation {
	public:
		/** Construct from configuration file */
		UIBKPoseEstimationImpl(const std::string& path);

		/**	Captures point cloud from Kinect */
		virtual void capture(Point3D::Seq& points);

		/** Find list of objects with their poses from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Pose::Seq& poses);
	};
};

#endif // _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_