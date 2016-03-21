
/** @file IO_Adhoc.h
 *
 * PaCMan IO_Adhoc definitions
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_ACTIVE_SENSE_IO_ADHOC_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVE_SENSE_IO_ADHOC_H_

#include "pacman/Bham/ActiveSenseGrasp/Core/ActiveSenseOM2.h"


namespace pacman {

namespace io_adhoc {

void add_testData(oct::AngleOcTree& tree);


void log_out(FILE* out, double landmark, const std::vector<double>& collisionProfile);

void log_out(FILE* out, const std::string& annotation, int experiment_id, int trial, int selection_method = -1, bool has_contacts = false,
             int view_id = -1, double landmark = -1, double view_value = -1, double entropy = -1, double inf_gain = -1, double collision_prob = -1);




}


}


#endif // _PACMAN_BHAM_ACTIVE_SENSE_IO_ADHOC_H_
