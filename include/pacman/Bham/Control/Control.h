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
		 *	@param[in]	type			robot type
		 *	@param[in]	path			configuration file with robot type specs
		 *  @return						interface pointer
		*/
		static Ptr create(RobotType type, const std::string& path);
		
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
		 *	@param[out]	state			pointer to the state
		*/
		virtual void lookupState(float_t t, void* state) const = 0;

		/**	Command sent to a robot at time t
		 *	@param[in]	t				query time [sec]
		 *	@param[out]	command			pointer to the command
		*/
		virtual void lookupCommand(float_t t, void* command) const = 0;

		/**	Robot control
		 *	@param[in]	command			command sequence begin
		 *	@param[in]	size			number of commands in the sequence
		*/
		virtual void send(const void* command, size_t size = 1) = 0;

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

		/** Destruction
		*/
		virtual ~BhamControl() {}
	};
};

#endif // _PACMAN_BHAM_CONTROL_CONTROL_H_