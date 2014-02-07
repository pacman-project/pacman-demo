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
		BhamControl::Ptr controller = BhamControl::create((RobotType)std::strtoul(argv[1], nullptr, 10), argv[2]);
		
		// Read current state
		pacman::RobotUIBK::State begin;
		controller->lookupState(controller->time(), &begin);

		// Prepare trajectory
		pacman::RobotUIBK::Command::Seq commands(5);
		// Initial pose
		commands[0].pos = begin.pos;
		// Pose #1
		commands[1].t = commands[0].t + pacman::float_t(5.0);
		commands[1].pos = begin.pos;
		commands[1].pos.arm.c[5] += pacman::float_t(0.5);
		// Pose #2
		commands[2].t = commands[1].t + pacman::float_t(5.0);
		commands[2].pos = begin.pos;
		commands[2].pos.arm.c[5] -= pacman::float_t(0.5);
		// Pose #3
		commands[3].t = commands[2].t + pacman::float_t(5.0);
		commands[3].pos = begin.pos;
		commands[3].pos.arm.c[6] += pacman::float_t(0.5);
		// Pose #4
		commands[4].t = commands[3].t + pacman::float_t(5.0);
		commands[4].pos = begin.pos;
		commands[4].pos.arm.c[6] -= pacman::float_t(0.5);

		// send trajectory
		controller->send(commands.data(), commands.size());

		// wait for completion
		controller->waitForTrajectoryEnd();
	}
	catch (const std::exception& ex) {
		printf("ControlTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}