/** @file RobotKuka.h
 * 
 * Example implementation - Kuka Robot.
 *
 */

#pragma once
#ifndef _PACMAN_ROBOTKUKA_ROBOTKUKA_H_ // if #pragma once is not supported
#define _PACMAN_ROBOTKUKA_ROBOTKUKA_H_

#include <pacman/Robot/Robot.h>

/** Always use possibly unique outer name space */
namespace pacman {
	/** The simplest robot factory can only create a single robot of a particular kind (here Kuka Robot). */
	Robot* createRobotKuka(const std::string& cfgfile);
};

#endif // _PACMAN_ROBOTKUKA_ROBOTKUKA_H_

