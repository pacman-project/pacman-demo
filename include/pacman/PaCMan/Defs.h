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
		/** Default constructor sets the default values. */
		inline Vec3() {
			setToDefault();
		}
		/** The default values. */
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
		/** Default constructor sets the default values. */
		inline Mat33() {
			setToDefault();
		}
		/** The default values. */
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

		/** Default constructor sets the default values. */
		inline Mat34() {
			//setToDefault();
		}
		/** The default values. */
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
				std::uint8_t r, g, b, a;
			};
			std::uint32_t uint32;
			std::uint8_t uint8[4];
		};
		/** Default constructor sets the default values. */
		inline RGBA() {
			setToDefault();
		}
		/** The default values. */
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

		/** Default constructor sets the default values. */
		inline Point3D() {
			setToDefault();
		}
		/** The default values. */
		inline void setToDefault() {
			position.setToDefault();
			normal.x = float_t(0.); normal.y = float_t(0.); normal.z = float_t(1.); // Z direction
			colour.setToDefault();
		}
	};

	/** Innsbruck robot state */
	class TimeStamp {
	public:
		/** Time stamp */
		float_t t;

		/** Default constructor sets the default values. */
		inline TimeStamp() {
			setToDefault();
		}
		/** The default values. */
		inline void setToDefault() {
			t = float_t(0.);
		}
	};

	/** State template (position control) */
	template <typename _Config> class State {
	public:
		/** Sequence */
		typedef std::vector<State> Seq;

		/** Position */
		_Config pos;

		/** Default constructor sets the default values. */
		inline State() {
			setToDefault();
		}
		/** The default values. */
		inline void setToDefault() {
			pos.setToDefault();
		}
	};

	/** Command template (position control) */
	template <typename _Config> class Command {
	public:
		/** Sequence */
		typedef std::vector<Command> Seq;

		/** Position */
		_Config pos;
		/** Velocity */
		_Config vel;
		/** Acceleration */
		_Config acc;

		/** Default constructor sets the default values. */
		inline Command() {
			setToDefault();
		}
		/** The default values. */
		inline void setToDefault() {
			pos.setToDefault();
			vel.setToDefault();
			acc.setToDefault();
		}
	};

	/** Kuka LWR */
	class KukaLWR {
	public:
		/** Kuka LWR configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = 1;
			/** Number of joints */
			static const std::uintptr_t JOINTS = 7;

			/** Config elements. */
			union {
				float_t c[JOINTS];
			};
			/** Default constructor sets the default values. */
			inline Config() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** Kuka LWR state */
		typedef pacman::State<Config> State;

		/** Kuka LWR command */
		typedef pacman::Command<Config> Command;
	};

	/** Shunk Dexterous Hand */
	class ShunkDexHand {
	public:
		/** Shunk Dexterous Hand configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = 3;
			/** Number of joints per finger */
			static const std::uintptr_t CHAIN_JOINTS = 2;
			/** Number of joints: middle, left, right, left+right rotation */
			static const std::uintptr_t JOINTS = CHAINS*CHAIN_JOINTS + 1;

			/** Config elements. */
			union {
				struct {
					float_t middle[CHAIN_JOINTS];
					float_t left[CHAIN_JOINTS];
					float_t right[CHAIN_JOINTS];
					float_t rotation;
				};
				float_t c[JOINTS];
			};
			/** Default constructor sets the default values. */
			inline Config() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** Shunk Dexterous Hand configuration + pose */
		class Pose {
		public:
			/** Sequence */
			typedef std::vector<Pose> Seq;

			/** Configuration. */
			Config config;
			/** 3D pose */
			Mat34 pose;

			/** Default constructor sets the default values. */
			inline Pose() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				config.setToDefault();
				pose.setToDefault();
			}
		};

		/** Shunk Dexterous Hand state */
		typedef pacman::State<Config> State;

		/** Shunk Dexterous Hand command */
		typedef pacman::Command<Config> Command;
	};

	/** Innsbruck robot */
	class RobotUIBK {
	public:
		/** Innsbruck robot configuration */
		class Config {
		public:
			/** Sequence */
			typedef std::vector<Config> Seq;

			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = KukaLWR::Config::CHAINS + ShunkDexHand::Config::CHAINS;
			/** Number of joints */
			static const std::uintptr_t JOINTS = KukaLWR::Config::JOINTS + ShunkDexHand::Config::JOINTS;

			/** Arm configuration. */
			KukaLWR::Config arm;
			/** Hand configuration. */
			ShunkDexHand::Config hand;

			/** Default constructor sets the default values. */
			inline Config() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				arm.setToDefault();
				hand.setToDefault();
			}
		};

		/** Innsbruck robot configuration + pose */
		class Pose : public Config {
		public:
			/** Sequence */
			typedef std::vector<Pose> Seq;

			/** 3D pose */
			Mat34 pose;

			/** Default constructor sets the default values. */
			inline Pose() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				arm.setToDefault();
				hand.setToDefault();
				pose.setToDefault();
			}
		};

		/** Innsbruck robot state */
		class State : public TimeStamp {
		public:
			/** Sequence */
			typedef std::vector<State> Seq;

			/** Arm state. */
			KukaLWR::State arm;
			/** Hand state. */
			ShunkDexHand::State hand;

			/** Default constructor sets the default values. */
			inline State() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				TimeStamp::setToDefault();
				arm.setToDefault();
				hand.setToDefault();
			}
		};

		/** Innsbruck robot command */
		class Command : public TimeStamp {
		public:
			/** Sequence */
			typedef std::vector<State> Seq;

			/** Arm state. */
			KukaLWR::Command arm;
			/** Hand state. */
			ShunkDexHand::Command hand;

			/** Default constructor sets the default values. */
			inline Command() {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				TimeStamp::setToDefault();
				arm.setToDefault();
				hand.setToDefault();
			}
		};
	};

	/** Robot type */
	enum RobotType {
		/** Innsbruck robot */
		ROBOT_UIBK,
	};
};

#endif // _PACMAN_PACMAN_DEFS_H_