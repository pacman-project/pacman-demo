/** @file PCL.h
 * 
 * PaCMan PCL definitions
 *
 */

#pragma once
#ifndef _PACMAN_PACMAN_PCL_H_ // if #pragma once is not supported
#define _PACMAN_PACMAN_PCL_H_

#include <pacman/PaCMan/Defs.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** PaCMan name space */
namespace pacman {
	/** pcl::PointXYZ point conversion */
	template <typename _PCLPointXYX> void convertPCLPointXYZ(const _PCLPointXYX& src, Point3D& dst) {
		dst.position.x = (float_t)src.x;
		dst.position.y = (float_t)src.y;
		dst.position.z = (float_t)src.z;
	}
	template <typename _PCLPointXYX> void convertPCLPointXYZ(const Point3D& src, _PCLPointXYX& dst) {
		dst.x = (float)src.position.x;
		dst.y = (float)src.position.y;
		dst.z = (float)src.position.z;
	}
	
	/** pcl::PointNormal point conversion */
	template <typename _PCLPointNormal> void convertPCLPointNormal(const _PCLPointNormal& src, Point3D& dst) {
		dst.normal.x = (float_t)src.normal_x;
		dst.normal.y = (float_t)src.normal_y;
		dst.normal.z = (float_t)src.normal_z;
	}
	template <typename _PCLPointNormal> void convertPCLPointNormal(const Point3D& src, _PCLPointNormal& dst) {
		dst.normal_x = (float)src.normal.x;
		dst.normal_y = (float)src.normal.y;
		dst.normal_z = (float)src.normal.z;
	}
	
	/** pcl::PointRGBA point conversion */
	template <typename _PCLPointRGBA> void convertPCLPointRGBA(const _PCLPointRGBA& src, Point3D& dst) {
		dst.colour.r = (std::uint8_t)src.r;
		dst.colour.g = (std::uint8_t)src.g;
		dst.colour.b = (std::uint8_t)src.b;
	}
	template <typename _PCLPointRGBA> void convertPCLPointRGBA(const Point3D& src, _PCLPointRGBA& dst) {
		dst.r = (std::uint8_t)src.colour.r;
		dst.g = (std::uint8_t)src.colour.g;
		dst.b = (std::uint8_t)src.colour.b;
	}
	
	/** pcl::PointXYZ point conversion */
	inline void convert(const pcl::PointXYZ& src, Point3D& dst) {
		convertPCLPointXYZ(src, dst);
	}
	inline void convert(const Point3D& src, pcl::PointXYZ& dst) {
		convertPCLPointXYZ(src, dst);
	}
	
	/** pcl::PointNormal point conversion */
	inline void convert(const pcl::PointNormal& src, Point3D& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
	}
	inline void convert(const Point3D& src, pcl::PointNormal& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
	}
	
	/** pcl::PointXYZRGBNormal point conversion */
	inline void convert(const pcl::PointXYZRGBNormal& src, Point3D& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
		convertPCLPointRGBA(src, dst);
	}
	inline void convert(const Point3D& src, pcl::PointXYZRGBNormal& dst) {
		convertPCLPointXYZ(src, dst);
		convertPCLPointNormal(src, dst);
		convertPCLPointRGBA(src, dst);
	}
	
	/** Point cloud conversion */
	template <typename _PCLPoint> void convert(const pcl::PointCloud<_PCLPoint>& src, Point3D::Seq& dst) {
		dst.resize(0);
		dst.reserve(src.size());
		for (auto i: src) {
			Point3D p;
			convert(i, p);
			dst.push_back(p);
		}
	}
	template <typename _PCLPoint> void convert(const Point3D::Seq& src, pcl::PointCloud<_PCLPoint>& dst) {
		dst.resize(0);
		dst.reserve(src.size());
		for (auto i: src) {
			_PCLPoint p;
			convert(i, p);
			dst.push_back(p);
		}
	}
};

#endif // _PACMAN_PACMAN_PCL_H_