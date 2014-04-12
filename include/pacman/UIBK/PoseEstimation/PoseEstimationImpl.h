/** @file GraspImpl.h
 * 
 * Bham Grasp interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_

#include <pacman/UIBK/PoseEstimation/PoseEstimation.h>
#include <pacman/PaCMan/PCL.h>

#include "/home/pacman/CODE/pacman/poseEstimation/include/I_SegmentedObjects.h"
#include "/home/pacman/CODE/pacman/poseEstimation/include/ParametersPoseEstimation.h"
/** PaCMan name space */
namespace pacman {
        /** Transformation from UIBK pose to pacman pose */
        class  Transformations 
        {
        public:
          void transformUIBKPoseToPose(const Eigen::Vector3f& translation, const Eigen::Matrix3f& rotation,pacman::UIBKPoseEstimation::Pose &pose_);
        };
	/** Innsbruck object database interface implementation */
	class UIBKObjectImpl : public UIBKObject {
        friend class I_SegmentedObjects;
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
                
                /*void setObjects(I_SegmentedObjects *objects_);
                I_SegmentedObjects* getObjects();*/
        private:
          Transformations *transformations;
        //  I_SegmentedObjects *objects;
	};

	/** Innsbruck pose estimation interface */
	class UIBKPoseEstimationImpl : public UIBKPoseEstimation {
        friend class I_SegmentedObjects;
	public:
		/** Construct from configuration file */
		UIBKPoseEstimationImpl(const std::string& path);

		/**	Captures point cloud from Kinect */
		virtual void capture(Point3D::Seq& points);
                virtual void load_cloud(std::string filename,Point3D::Seq& points);
		/** Find list of objects with their poses from a given point cloud */
		virtual void estimate(const Point3D::Seq& points, Pose::Seq& poses);

                void setObjects(I_SegmentedObjects *objects_);
                I_SegmentedObjects* getObjects();
		
        private:
                Transformations *transformations;
                bool recognizedObjects;
                pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points_;
          //      I_SegmentedObjects *objects;
	};
       // void setObjects(I_SegmentedObjects *objects_);
        //void getObjects(I_SegmentedObjects& objects_);
       // I_SegmentedObjects *objects;
};

#endif // _PACMAN_UIBK_POSE_ESTIMATION_POSE_ESTIMATION_IMPL_H_
