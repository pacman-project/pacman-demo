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

	/** Robot */
	class Robot {
	public:
		/** Robot types */
		enum Type {
			/** Kuka LWR */
			KUKA_LWR = 1,
			/** Schunk Dexterous Hand */
			SCHUNK_DEX_HAND,
			/** KIT Head */
			KIT_HEAD,

			/** Innsbruck robot */
			ROBOT_UIBK = 101,
			/** Innsbruck robot Eddie */
			ROBOT_EDDIE,
		};

		/** Robot data */
		class Data {
		public:
			/** Robot type */
			inline Type getType() const {
				return type;
			}
			/** Data size */
			inline std::uintptr_t getSize() const {
				return size;
			}
		
			/** Data array access */
			inline Data& operator () (std::uintptr_t idx) {
				return *(Data*)((char*)this + idx*size);
			}
			/** Data array access */
			inline const Data& operator () (std::uintptr_t idx) const {
				return *(const Data*)((const char*)this + idx*size);
			}

		protected:
			/** Set robot type and data size. */
			Data(Type type, std::uintptr_t dataSize) : type(type), size(dataSize) {}

		private:
			/** Robot type */
			Type type;
			/** Data size */
			std::uintptr_t size;
		};
		
		/** Robot state base */
		class State : public Data {
		protected:
			/** Set robot type and data size. */
			State(Type type, std::uintptr_t size) : Data(type, size) {}
		};

		/** Robot command base */
		class Command : public Data {
		protected:
			/** Set robot type and data size. */
			Command(Type type, std::uintptr_t size) : Data(type, size) {}
		};

		/** Robot state data template (position control) */
		template <typename _Config> class StateData : public State  {
		public:
			/** Type */
			static const Type TYPE = _Config::TYPE;

			/** Position */
			_Config pos;

			/** Default constructor sets the default values. */
			inline StateData(Type type = TYPE, std::uintptr_t size = sizeof(StateData<_Config>)) : State(type, size) {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				pos.setToDefault();
			}
		};
		/** Robot command data template (position control) */
		template <typename _Config> class CommandData : public Command {
		public:
			/** Type */
			static const Type TYPE = _Config::TYPE;

			/** Position */
			_Config pos;
			/** Velocity */
			_Config vel;
			/** Acceleration */
			_Config acc;

			/** Default constructor sets the default values. */
			inline CommandData(Type type = TYPE, std::uintptr_t size = sizeof(CommandData<_Config>)) : Command(type, size) {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				pos.setToDefault();
				vel.setToDefault();
				acc.setToDefault();
			}
		};
		/** Time ordered sequence */
		template <typename _RobotData> class TimeStamp : public _RobotData  {
		public:
			/** Sequence */
			typedef std::vector<TimeStamp> Seq;

			/** Type */
			static const Type TYPE = _RobotData::TYPE;

			/** Time stamp */
			float_t t;

			/** Default constructor sets the default values. */
			inline TimeStamp() : _RobotData(TYPE, sizeof(TimeStamp<_RobotData>)) {
				setToDefault();
			}
			/** The default values. */
			inline void setToDefault() {
				_RobotData::setToDefault();
				t = float_t(0.);
			}
		};
	};

	/** Kuka LWR */
	class KukaLWR {
	public:
		/** Kuka LWR configuration */
		class Config {
		public:
			/** Type */
			static const Robot::Type TYPE = Robot::Type::KUKA_LWR;
			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = 1;
			/** Number of joints */
			static const std::uintptr_t JOINTS = 7;

			/** Config elements. */
			union {
				float_t c[JOINTS];
			};

			/** The default values. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** Kuka LWR state */
		typedef pacman::Robot::StateData<Config> StateData;
		typedef pacman::Robot::TimeStamp<StateData> State;

		/** Kuka LWR command */
		typedef pacman::Robot::CommandData<Config> CommandData;
		typedef pacman::Robot::TimeStamp<CommandData> Command;
	};

	/** Schunk Dexterous Hand */
	class SchunkDexHand {
	public:
		/** Schunk Dexterous Hand configuration */
		class Config {
		public:
			/** Type */
			static const Robot::Type TYPE = Robot::Type::SCHUNK_DEX_HAND;
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

			/** The default values. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** Schunk Dexterous Hand configuration + pose */
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

		/** Schunk Dexterous Hand state */
		typedef pacman::Robot::StateData<Config> StateData;
		typedef pacman::Robot::TimeStamp<StateData> State;

		/** Schunk Dexterous Hand command */
		typedef pacman::Robot::CommandData<Config> CommandData;
		typedef pacman::Robot::TimeStamp<CommandData> Command;
	};

	/** KIT head */
	class KITHead {
	public:
		/** KIT head configuration */
		class Config {
		public:
			/** Type */
			static const Robot::Type TYPE = Robot::Type::KIT_HEAD;
			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = 3;
			/** Number of joints of the neck */
			static const std::uintptr_t JOINTS_NECK = 5;
			/** Number of joints of the left eye */
			static const std::uintptr_t JOINTS_LEFT_EYE = 1;
			/** Number of joints of the right eye */
			static const std::uintptr_t JOINTS_RIGHT_EYE = 1;
			/** Number of joints */
			static const std::uintptr_t JOINTS = JOINTS_NECK + JOINTS_LEFT_EYE + JOINTS_RIGHT_EYE;

			/** Config elements. */
			union {
				struct {
					float_t neck[JOINTS_NECK];
					float_t eyeLeft;
					float_t eyeRight;
				};
				float_t c[JOINTS];
			};

			/** The default values. */
			inline void setToDefault() {
				std::fill(c, c + JOINTS, float_t(0.));
			}
		};

		/** KIT head state */
		typedef pacman::Robot::StateData<Config> StateData;
		typedef pacman::Robot::TimeStamp<StateData> State;

		/** KIT head command */
		typedef pacman::Robot::CommandData<Config> CommandData;
		typedef pacman::Robot::TimeStamp<CommandData> Command;
	};

	/** Innsbruck robot */
	class RobotUIBK {
	public:
		/** Innsbruck robot data */
		template <typename _Base, typename _ArmData, typename _HandData> class Data : public _Base {
		public:
			/** Sequence */
			typedef std::vector<Data> Seq;

			/** Type */
			static const Robot::Type TYPE = Robot::Type::ROBOT_UIBK;
			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = KukaLWR::Config::CHAINS + SchunkDexHand::Config::CHAINS;
			/** Number of joints */
			static const std::uintptr_t JOINTS = KukaLWR::Config::JOINTS + SchunkDexHand::Config::JOINTS;

			/** Arm. */
			_ArmData arm;
			/** Hand. */
			_HandData hand;

			/** Default constructor sets data type and size. */
			inline Data(Robot::Type type = TYPE, std::uintptr_t size = sizeof(Data<_Base, _ArmData, _HandData>)) : _Base(type, size) {}
			/** The default values. */
			inline void setToDefault() {
				arm.setToDefault();
				hand.setToDefault();
			}
		};

		/** Innsbruck robot configuration */
		typedef Data<Robot::Data, KukaLWR::Config, SchunkDexHand::Config> Config;

		/** Innsbruck robot state */
		typedef Data<Robot::State, KukaLWR::StateData, SchunkDexHand::StateData> StateData;
		typedef pacman::Robot::TimeStamp<StateData> State;

		/** Innsbruck robot command */
		typedef Data<Robot::Command, KukaLWR::CommandData, SchunkDexHand::CommandData> CommandData;
		typedef pacman::Robot::TimeStamp<CommandData> Command;
	};

	/** Innsbruck robot Eddie */
	class RobotEddie {
	public:
		/**  Innsbruck robot Eddie data */
		template <typename _Base, typename _ArmData, typename _HandData, typename _HeadData> class Data : public _Base {
		public:
			/** Sequence */
			typedef std::vector<Data> Seq;

			/** Type */
			static const Robot::Type TYPE = Robot::Type::ROBOT_EDDIE;
			/** Number of kinematic chains */
			static const std::uintptr_t CHAINS = 2*KukaLWR::Config::CHAINS + 2*SchunkDexHand::Config::CHAINS + KITHead::Config::CHAINS;
			/** Number of joints */
			static const std::uintptr_t JOINTS = 2*KukaLWR::Config::JOINTS + 2*SchunkDexHand::Config::JOINTS + KITHead::Config::JOINTS;

			/** Left arm. */
			_ArmData armLeft;
			/** Left hand. */
			_HandData handLeft;
			/** Right arm. */
			_ArmData armRight;
			/** Right hand. */
			_HandData handRight;
			/** Head. */
			_HeadData head;

			/** Default constructor sets data type and size. */
			inline Data(Robot::Type type = TYPE, std::uintptr_t size = sizeof(Data<_Base, _ArmData, _HandData, _HeadData>)) : _Base(type, size) {}

			/** The default values. */
			inline void setToDefault() {
				armLeft.setToDefault();
				handLeft.setToDefault();
				armRight.setToDefault();
				handRight.setToDefault();
				head.setToDefault();
			}
		};

		/** Innsbruck robot Eddie configuration */
		typedef Data<Robot::Data, KukaLWR::Config, SchunkDexHand::Config, KITHead::StateData> Config;

		/**  Innsbruck robot Eddie state */
		typedef Data<Robot::State, KukaLWR::StateData, SchunkDexHand::StateData, KITHead::StateData> StateData;
		typedef pacman::Robot::TimeStamp<StateData> State;

		/**  Innsbruck robot Eddie command */
		typedef Data<Robot::Command, KukaLWR::CommandData, SchunkDexHand::CommandData, KITHead::CommandData> CommandData;
		typedef pacman::Robot::TimeStamp<CommandData> Command;
	};
};

#endif // _PACMAN_PACMAN_DEFS_H_