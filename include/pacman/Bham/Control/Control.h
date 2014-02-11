/** @file Control.h
 * 
 * Bham Control interface
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_CONTROL_CONTROL_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_CONTROL_CONTROL_H_

#include <pacman/PaCMan/Defs.h>
#include <boost/shared_ptr.hpp>
#include <string>

/** PaCMan name space */
namespace pacman {
	/** Birmingham control interface */
	class BhamControl {
	public:
		/** Pointer */
		typedef boost::shared_ptr<BhamControl> Ptr;

		/** Creates Birmingham control interface
		 *	@param[in]	path			configuration file
		 *  @return						interface pointer
		*/
		static Ptr create(const std::string& path);
		
		/** Current time counted from an arbitrary moment
		 *	@return						current time [sec]
		*/
		virtual float_t time() const = 0;

		/** Controller cycle duration
		 *	@return						cycle duration [sec]
		*/
		virtual float_t cycleDuration() const = 0;

		/**	State of a robot at time t
		 *	@param[in]	t				query time [sec]
		 *	@param[out]	state			robot state
		*/
		virtual void lookupState(float_t t, RobotState& state) const = 0;

		/**	Command sent to a robot at time t
		 *	@param[in]	t				query time [sec]
		 *	@param[out]	command			robot command
		*/
		virtual void lookupCommand(float_t t, RobotCommand& command) const = 0;

		/**	Robot control
		 *	@param[in]	command			command sequence begin
		 *	@param[in]	size			number of commands in the sequence
		*/
		virtual void send(const RobotCommand* command, std::uintptr_t size = 1) = 0;

		/**	Wait for begin of the robot control cycle
		 *	@param[in]	timewait		maximum time wait [sec]
		 *	@return						true if success, false otherwise
		*/
		virtual bool waitForCycleBegin(double timewait = std::numeric_limits<float_t>::max()) = 0;

		/**	Wait for end of the robot trajectory execution
		 *	@param[in]	timewait		maximum time wait [sec]
		 *	@return						true if success, false otherwise
		*/
		virtual bool waitForTrajectoryEnd(double timewait = std::numeric_limits<float_t>::max()) = 0;

		
		/** Kuka LWR config conversion */
		template <typename _Real> static void configToPacman(const _Real* c, KukaLWR::Config& config) {
			for (std::uintptr_t i = 0; i < KukaLWR::Config::JOINTS; ++i)
				config.c[i] = (pacman::float_t)c[i];
		}
		/** Kuka LWR config conversion */
		template <typename _Real> static void configToGolem(const KukaLWR::Config& config, _Real* c) {
			for (std::uintptr_t i = 0; i < KukaLWR::Config::JOINTS; ++i)
				c[i] = (_Real)config.c[i];
		}
		
		/** ShunkDexHand config conversion */
		template <typename _Real> static void configToPacman(const _Real* c, ShunkDexHand::Config& config) {
			config.middle[0] = (pacman::float_t)c[0];
			config.middle[1] = (pacman::float_t)c[1];
			config.left[0] = (pacman::float_t)c[3];
			config.left[1] = (pacman::float_t)c[4];
			config.right[0] = (pacman::float_t)c[6];
			config.right[1] = (pacman::float_t)c[7];
			config.rotation = -(pacman::float_t)c[2];// reverse rotation
		}
		/** ShunkDexHand config conversion */
		template <typename _Real> static void configToGolem(const ShunkDexHand::Config& config, _Real* c) {
			c[0] = (_Real)config.middle[0];
			c[1] = (_Real)config.middle[1];
			c[3] = (_Real)config.left[0];
			c[4] = (_Real)config.left[1];
			c[6] = (_Real)config.right[0];
			c[7] = (_Real)config.right[1];
			c[2] = -(_Real)config.rotation;// reverse rotation
			c[5] = (_Real)config.rotation;// reverse rotation
		}
		

		/** Destruction
		*/
		virtual ~BhamControl() {}
	};
};

#endif // _PACMAN_BHAM_CONTROL_CONTROL_H_