#include <pacman/Bham/Control/Control.h>
#include <exception>

using namespace pacman;

int main(int argc, char *argv[]) {
	try {
		if (argc < 3) {
			printf("ControlTest <robot_type> <configuration_file>\n");
			return 1;
		}

		// Create controller
		BhamControl::Ptr grasp = BhamControl::create((RobotType)std::strtoul(argv[1], nullptr, 10), argv[2]);
		
		// TODO
	}
	catch (const std::exception& ex) {
		printf("ControlTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}