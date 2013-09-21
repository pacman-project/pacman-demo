/** @file Desf.h
 * 
 * PaCMan definitions
 *
 */

#pragma once
#ifndef _PACMAN_PACMAN_DEFS_H_ // if #pragma once is not supported
#define _PACMAN_PACMAN_DEFS_H_

#include <cstdint>
#include <vector>

/** PaCMan name space */
namespace pacman {
	/** Pacman default floating point */
	typedef double float_t;

	/** 3 Element vector class. */
	class Vec3 {
	public:
		/** vector components */
		union {
			struct {
				float_t v1, v2, v3;
			};
			struct {
				float_t x, y, z;
			};
			float_t v[3];
		};
		/** Default constructor sets the default configuration. */
		inline Vec3() {
			setToDefault();
		}
		/** The default configuration. */
		inline void setToDefault() {
			v1 = float_t(0.); v2 = float_t(0.); v3 = float_t(0.);
		}
	};

	/** Matrix representation of SO(3) group of rotations. */
	class Mat33 {
	public:
		/** Matrix elements. */
		union {
			struct {
				float_t m11, m12, m13;
				float_t m21, m22, m23;
				float_t m31, m32, m33;
			};
			float_t m[3][3];
		};
		/** Default constructor sets the default configuration. */
		inline Mat33() {
			setToDefault();
		}
		/** The default configuration. */
		inline void setToDefault() {
			m11 = float_t(1.); m12 = float_t(0.); m13 = float_t(0.);
			m21 = float_t(0.); m22 = float_t(1.); m23 = float_t(0.);
			m31 = float_t(0.); m32 = float_t(0.); m33 = float_t(1.);
		}
	};

	/** Homogeneous representation of SE(3) rigid body transformations. */
	class Mat34 {
	public:
		/** rotation matrix	*/
		Mat33 R;
		/** translation	*/
		Vec3 p;

		/** Default constructor sets the default configuration. */
		inline Mat34() {
			//setToDefault();
		}
		/** The default configuration. */
		inline void setToDefault() {
			R.setToDefault();
			p.setToDefault();
		}
	};

	/** RGBA colour space */
	class RGBA {
	public:
		/** Colour elements. */
		union {
			struct {
				std::uint8_t R, G, B, A;
			};
			std::uint32_t uint32;
			std::uint8_t uint8[4];
		};
		/** Default constructor sets the default configuration. */
		inline RGBA() {
			setToDefault();
		}
		/** The default configuration. */
		inline void setToDefault() {
			uint32 = UINT32_MAX; // white colour
		}
	};

	/** 3D point representation. */
	class Point3D {
	public:
		/** Point cloud */
		typedef std::vector<Point3D> Seq;

		/** Position */
		Vec3 position;
		/** Normal */
		Vec3 normal;
		/** Colour */
		RGBA colour;

		/** Default constructor sets the default configuration. */
		inline Point3D() {
			setToDefault();
		}
		/** The default configuration. */
		inline void setToDefault() {
			position.setToDefault();
			normal.x = float_t(0.); normal.y = float_t(0.); normal.z = float_t(1.); // Z direction
			colour.setToDefault();
		}
	};

	/** Kuka LWR */
	class KukaLWR {
	public:
		/** Number of joints= */
		static const std::uintptr_t JOINTS = 7;

		/** Kuka LWR configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Config elements. */
			union {
				float_t c[JOINTS];
			};
			/** Default constructor sets the default configuration. */
			inline Config() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};
	};

	/** Shunk Dexterous Hand */
	class ShunkDexHand {
	public:
		/** Number of fingers */
		static const std::uintptr_t FINGERS = 3;
		/** Number of joints per finger */
		static const std::uintptr_t FINGER_JOINTS = 2;
		/** Number of joints: middle, left, right, left+right rotation */
		static const std::uintptr_t JOINTS = FINGERS*FINGER_JOINTS + 1;

		/** Shunk Dexterous Hand configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Config elements. */
			union {
				struct {
					float_t middle[FINGER_JOINTS];
					float_t left[FINGER_JOINTS];
					float_t right[FINGER_JOINTS];
					float_t rotation;
				};
				float_t c[JOINTS];
			};
			/** Default constructor sets the default configuration. */
			inline Config() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** Shunk Dexterous Hand configuration + pose */
		class Pose {
			/** Sequence */
			typedef std::vector<Pose> Seq;

			/** Configuration. */
			Config config;
			/** 3D pose */
			Mat34 pose;

			/** Default constructor sets the default configuration. */
			inline Pose() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				config.setToDefault();
				pose.setToDefault();
			}
		};
	};

	/** Innsbruck robot */
	class RobotUIBK {
	public:
		/** Number of joints= */
		static const std::uintptr_t JOINTS = 7;

		/** Innsbruck robot configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Arm configuration. */
			KukaLWR::Config configArm;
			/** Hand configuration. */
			ShunkDexHand::Config configHand;

			/** Default constructor sets the default configuration. */
			inline Config() {
				setToDefault();
			}
			/** The default configuration. */
			inline void setToDefault() {
				configArm.setToDefault();
				configHand.setToDefault();
			}
		};
	};
};

#endif // _PACMAN_PACMAN_DEFS_H_