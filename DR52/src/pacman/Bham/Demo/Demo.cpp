#include <pacman/Bham/Demo/Demo.h>

using namespace pacman;
using namespace golem;
using namespace grasp;

//-----------------------------------------------------------------------------

void pacman::Demo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

}

//------------------------------------------------------------------------------

pacman::Demo::Demo(Scene &scene) : Player(scene) {
}

pacman::Demo::~Demo() {
}

void pacman::Demo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Demo::Desc."));

	// create object
	Player::create(desc); // throws


	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F5", "  P                                       menu PaCMan\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (D)emo...";
	}));
	menuCmdMap.insert(std::make_pair("PD", [=]() {
		// TODO
	}));
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return pacman::Demo::Desc().main(argc, argv);
}
