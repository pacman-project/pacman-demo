/** @file Grasp.cpp
 *
 * Adapated from Demo Grasp by Ermano Arruda
 *
 * @author	Marek Kopicki
 * @author Ermano Arruda
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */


#include "pacman/Bham/Demo/Demo.h"

using namespace golem;
using namespace grasp;

namespace pacman {


//-----------------------------------------------------------------------------

void DemoDR55::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	ActiveSenseDemo::Desc::load(context, xmlcontext);


}

//------------------------------------------------------------------------------

DemoDR55::DemoDR55(Scene &scene) : ActiveSenseDemo(scene) {
}

DemoDR55::~DemoDR55() {
}

void DemoDR55::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::DemoDR55::Desc."));

	// create object
	ActiveSenseDemo::create(desc); // throws
}

//------------------------------------------------------------------------------

void DemoDR55::render() const {
	ActiveSenseDemo::render();
}



}// namespace pacman

//------------------------------------------------------------------------------

 int main(int argc, char *argv[]) {
 	return pacman::DemoDR55::Desc().main(argc, argv);
 }
