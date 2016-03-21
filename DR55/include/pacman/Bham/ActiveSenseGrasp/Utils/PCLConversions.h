/** @file PCL.h
 *
 * PaCMan PCL definitions
 *
 */

#pragma once
#ifndef _PACMAN_PACMAN_PCL_H_ // if #pragma once is not supported
#define _PACMAN_PACMAN_PCL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
#include <Grasp/Data/ContactModel/ContactModel.h>

/** PaCMan name space */
namespace pacman {

namespace utils {



template <typename _PCLPointXYX> void convertPCLPointXYZ(const grasp::Cloud::PointCurv& src, _PCLPointXYX& dst) {
    dst.x = static_cast<float>(src.x);
    dst.y = static_cast<float>(src.y);
    dst.z = static_cast<float>(src.z);
}

template <typename _PCLPointXYX> void convertPCLPointXYZ(const grasp::Cloud::Point& src, _PCLPointXYX& dst) {
    dst.x = static_cast<float>(src.x);
    dst.y = static_cast<float>(src.y);
    dst.z = static_cast<float>(src.z);
}

template <typename _PCLPointXYX> void convertPCLPointXYZ(const grasp::Contact3D& src, _PCLPointXYX& dst) {
    dst.x = static_cast<float>(src.point.x);
    dst.y = static_cast<float>(src.point.y);
    dst.z = static_cast<float>(src.point.z);
}

template <typename _PCLPointNormal> void convertPCLPointNormal(const grasp::Cloud::PointCurv& src, _PCLPointNormal& dst) {
    dst.normal_x = static_cast<float>(src.normal_x);
    dst.normal_y = static_cast<float>(src.normal_y);
    dst.normal_z = static_cast<float>(src.normal_z);
}

template <typename _PCLPointNormal> void convertPCLPointNormal(const grasp::Cloud::Point& src, _PCLPointNormal& dst) {
    dst.normal_x = static_cast<float>(src.normal_x);
    dst.normal_y = static_cast<float>(src.normal_y);
    dst.normal_z = static_cast<float>(src.normal_z);
}

template <typename _PCLPointNormal> void convertPCLPointNormal(const grasp::Contact3D& src, _PCLPointNormal& dst) {
    golem::Mat33 R;
    R.fromQuat(src.orientation);
    golem::Vec3 n(0.0, 0.0, 0.0);
    R.getColumn(2,n);//(3, n); //Surface Normal vector of contact point
    n.normalise();

    dst.normal_x = static_cast<float>(n.x);
    dst.normal_y = static_cast<float>(n.y);
    dst.normal_z = static_cast<float>(n.z);
}

template <typename _PCLPointRGB> void convertPCLPointRGBA(const grasp::Cloud::PointCurv& src, _PCLPointRGB& dst) {
    dst.rgba = src.rgba;
}

template <typename _PCLPointRGB> void convertPCLPointRGBA(const grasp::Cloud::Point& src, _PCLPointRGB& dst) {
    dst.rgba = src.rgba;
}

/** pcl::PointXYZRGBNormal point conversion */
inline void convert(const grasp::Cloud::PointCurv& src, pcl::PointXYZRGBNormal& dst) {
    convertPCLPointXYZ(src, dst);
    convertPCLPointNormal(src, dst);
    convertPCLPointRGBA(src, dst);
}

inline void convert(const grasp::Cloud::Point& src, pcl::PointXYZRGBNormal& dst) {
    convertPCLPointXYZ(src, dst);
    convertPCLPointNormal(src, dst);
    convertPCLPointRGBA(src, dst);
}

inline void convert(const grasp::Contact3D& src, pcl::PointXYZRGBNormal& dst) {
    convertPCLPointXYZ(src, dst);
    convertPCLPointNormal(src, dst);
}

inline void convert(const grasp::Contact3D& src, pcl::PointXYZRGBNormal& dst, float& dst_weight_out) {
    convertPCLPointXYZ(src, dst);
    convertPCLPointNormal(src, dst);
    dst_weight_out = src.weight;
}

template <typename _PCLPoint> void convert(const grasp::Cloud::PointCurvSeq& src, pcl::PointCloud<_PCLPoint>& dst) {
    dst.resize(0);
    dst.reserve(src.size());
    for (auto i: src) {
        _PCLPoint p;
        convert(i, p);
        dst.push_back(p);
    }
}

template <typename _PCLPoint> void convert(const grasp::Cloud::PointSeq& src, pcl::PointCloud<_PCLPoint>& dst) {
    dst.resize(0);
    dst.reserve(src.size());
    for (auto i: src) {
        _PCLPoint p;
        convert(i, p);
        dst.push_back(p);
    }
}

template <typename _PCLPoint> void convert(const grasp::Contact3D::Seq& src, pcl::PointCloud<_PCLPoint>& dst, std::vector<float>& dst_weights) {
    dst.resize(0);
    dst.reserve(src.size());

    dst_weights.resize(0);
    dst_weights.reserve(src.size());

    for (auto i: src) {
        _PCLPoint p;
        float w = 0;
        convert(i, p, w);
        dst.push_back(p);
        dst_weights.push_back(w);
    }
}

template <typename _PCLPoint> void convert(const grasp::Contact3D::Seq& src, pcl::PointCloud<_PCLPoint>& dst) {
    dst.resize(0);
    dst.reserve(src.size());
    for (auto i: src) {
        _PCLPoint p;
        convert(i, p);
    }
}






} // namespace conver
};

#endif // _PACMAN_PACMAN_PCL_H_
