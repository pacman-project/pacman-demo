/** @file Robot.h
 * 
 * Example interface
 *
 */

#pragma once
#ifndef _PACMAN_ROBOT_ROBOT_H_ // if #pragma once is not supported
#define _PACMAN_ROBOT_ROBOT_H_

#include <string>
#include <vector>

/** Always use possibly unique outer name space */
namespace pacman {
	/** Robot interface */
	class Robot {
	public:
		/** Robot's joint configuration */
		typedef std::vector<double> Configuration;

		/** Name of the robot */
		virtual const std::string& getName() const = 0; // do not return heavy-weight object by value (in particular if they use heap allocation), if they are constant (robot's name does not change).

		/** Returns the current robot configuration */
		virtual void getConfiguration(Configuration& configuration) const = 0; // wherever possible use const member functions, if only a call does not modify the object for which it is called. 

		/** Moves robot to the specified configuration */
		virtual void moveConfiguration(const Configuration& configuration) = 0;
		
		/** Virtual descrutor cannot be abstract! */
		virtual ~Robot() {}
	};
};

#endif // _PACMAN_ROBOT_ROBOT_H_