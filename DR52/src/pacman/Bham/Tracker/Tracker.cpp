#include <pacman/Bham/Tracker/Tracker.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

void Tracker::Desc::load(golem::Context &context, const golem::XMLContext* xmlcontext)
{
	golem::XMLData("nstep", nstep, xmlcontext->getContextFirst("rotate_in_hand"), false);
}

void Tracker::create(const Desc& desc)
{
	desc.assertValid(Assert::Context("Tracker::Desc."));

	nstep = desc.nstep;
}

