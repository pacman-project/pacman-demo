/** @file ROS.h
 *
 * PaCMan ROS definitions
 *
 */

#pragma once
#ifndef _PACMAN_PACMAN_ROS_H_ // if #pragma once is not supported
#define _PACMAN_PACMAN_ROS_H_

#include <pacman/PaCMan/Defs.h>
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include "definitions/GraspPlanning.h"

//In ROS messages the size of the data structure could be 16-byte (for point data alone) or 32-byte (for point data + any other information);

/** PaCMan name space */
namespace pacman {




 	/** RGBA colour space */
	struct RGBAROS {
		/** Colour elements. */
		union {
			struct {
				std::uint8_t r, g, b, a;
			};
			std::uint32_t uint32;
			std::uint8_t uint8[4];
		};
	};

    struct position{
        float x, y, z;
    };

    struct normals{
        float nx, ny, nz;
    };

 	/** Point */
	struct Point3DROS {
		/** . */
		union {
			struct {
				position pxyz;
				float c;  //dummy data
				RGBAROS rgba;
				normals nxyz;
			};
			std::uint8_t uint8[32];
		};
	};



    template <typename _ROSPointXYZ> void convertROSPointXYZ(const _ROSPointXYZ& src, Point3D& dst) {
		dst.position.x = (float_t)src.pxyz.x;
		dst.position.y = (float_t)src.pxyz.y;
		dst.position.z = (float_t)src.pxyz.z;
	}

    template <typename _ROSPointNormal> void convertROSPointNormal(const _ROSPointNormal& src, Point3D& dst) {
		dst.normal.x = (float_t)src.nxyz.nx;
		dst.normal.y = (float_t)src.nxyz.ny;
		dst.normal.z = (float_t)src.nxyz.nz;
	}

    template <typename _ROSPointRGBA> void convertROSPointRGBA(const _ROSPointRGBA& src, Point3D& dst) {
		dst.colour.r = (std::uint8_t)src.rgba.r;
		dst.colour.g = (std::uint8_t)src.rgba.g;
		dst.colour.b = (std::uint8_t)src.rgba.b;
	}


    inline void convert(const Point3DROS.pxyz& src, Point3D& dst) {
		convertROSPointXYZ(src, dst);
	}

    inline void convert(const Point3DROS.nxyz& src, Point3D& dst) {
		convertROSPointXYZ(src, dst);
		convertROSPointNormal(src, dst);
	}

	inline void convert(const Point3DROS.rgba& src, Point3D& dst) {
		convertROSPointXYZ(src, dst);
		convertROSPointNormal(src, dst);
		convertROSPointRGBA(src, dst);
	}

    void convert(const sensor_msgs::PointCloud2& src, Point3D::Seq& dst) {
		dst.resize(0);
		dst.reserve(src->width*src->height);
		for (int i=0;i < src->width*src->height;i++) {
		    Point3DROS pr;
			Point3D p;
			memcpy(pr.uint8,src->data[i*sizeof(pr.uint8)],sizeof(pr.uint8))
			convert(pr, p);
			dst.push_back(p);
		}
	}








	/** ros::PointCloud2 to Point3D */
	inline void convert(const sensor_msgs::PointCloud2& src, const Point3D::Seq& dst){
        dst.resize(src->width*src->height);
        if(src->fields[0].name !="x"){
            cout << "Wrong field in PointCloud2 structure"
            return 0;
        }
        if(src->fields[4].name !="nx"){
            cout << "Wrong field in PointCloud2 structure"
            return 0;
        }

        uint8_t tab[4];
        for(int i=0;i<src->height;i++){
            for(int j=0;j<src->width;j++){
                // filling x field of the pacman::Point3D from sensor_msgs::PointCloud2

                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[0].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].position.x = *(const float*)tab;

                // filling y of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[1].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].position.y = *(const float*)tab;

                // filling z of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[2].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].position.z = *(const float*)tab;

                // filling rgb of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) dst[src->width*i+j].colour.uint8[k]=src->data[k+src->fields[3].offset+j*src->point_step+i*src->row_step];

                // filling nx field of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[4].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].normal.x = *(const float*)tab;

                // filling ny of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[5].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].normal.y = *(const float*)tab;

                // filling nz of the pacman::Point3D from sensor_msgs::PointCloud2
                for(int k=0;k<4;k++) tab[k]=src->data[k+src->fields[6].offset+j*src->point_step+i*src->row_step];
                dst[src->width*i+j].normal.z = *(const float*)tab;
            }
        }

	}
	inline void convert(const ShunkDexHand::Config& src, const definitions::GraspPlanning::GraspList::joints& dst){
            dst[0] = src->middle[0];
            dst[1] = src->middle[1];
            dst[2] = src->left[0];
            dst[3] = src->left[1];
            dst[4] = src->right[0];
            dst[5] = src->right[1];
            dst[6] = src->rotation;
	}
//	inline void convert(const ShunkDexHand::Pose& src, const ros::pose& dst){
//
//	}

};

#endif // _PACMAN_PACMAN_PCL_H_
