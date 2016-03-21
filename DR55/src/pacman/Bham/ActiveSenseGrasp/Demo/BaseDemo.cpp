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


#include "pacman/Bham/ActiveSenseGrasp/Demo/BaseDemo.h"

using namespace golem;
using namespace grasp;

namespace pacman {


//-----------------------------------------------------------------------------

void BaseDemo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	BaseDemoDR55::Desc::load(context, xmlcontext);


}

//------------------------------------------------------------------------------

BaseDemo::BaseDemo(Scene &scene) : BaseDemoDR55(scene) {
}

BaseDemo::~BaseDemo() {
}

void BaseDemo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("pacman::BaseDemo::Desc."));

	// create object
	BaseDemoDR55::create(desc); // throws
}

//------------------------------------------------------------------------------

void BaseDemo::render() const {
	BaseDemoDR55::render();
}



}// namespace pacman

//------------------------------------------------------------------------------

// int main(int argc, char *argv[]) {
// 	return BaseDemo::Desc().main(argc, argv);
// }
