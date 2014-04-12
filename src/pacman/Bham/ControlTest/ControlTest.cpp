#include <pacman/Bham/Control/Control.h>
#include <exception>

using namespace pacman;

int main(int argc, char *argv[]) {
	try {
		if (argc < 2) {
			printf("ControlTest <configuration_file>\n");
			return 1;
		}

		// Create controller
		BhamControl::Ptr controller = BhamControl::create(argv[1]);

		//// RobotUIBK
		//// Read current state
		//pacman::RobotUIBK::State begin;
		//controller->lookupState(controller->time(), begin);
		//// Prepare trajectory
		//pacman::RobotUIBK::Command::Seq commands(5);
		//// Initial pose
		//commands[0].arm.pos = begin.arm.pos;
		//// Pose #1
		//commands[1].t = commands[0].t + pacman::float_t(5.0);
		//commands[1].arm.pos = begin.arm.pos;
		//commands[1].arm.pos.c[5] += pacman::float_t(0.5);
		//// Pose #2
		//commands[2].t = commands[1].t + pacman::float_t(5.0);
		//commands[2].arm.pos = begin.arm.pos;
		//commands[2].arm.pos.c[5] -= pacman::float_t(0.5);
		//// Pose #3
		//commands[3].t = commands[2].t + pacman::float_t(5.0);
		//commands[3].arm.pos = begin.arm.pos;
		//commands[3].arm.pos.c[6] += pacman::float_t(0.5);
		//// Pose #4
		//commands[4].t = commands[3].t + pacman::float_t(5.0);
		//commands[4].arm.pos = begin.arm.pos;
		//commands[4].arm.pos.c[6] -= pacman::float_t(0.5);

		// RobotEddie
		// Read current state
		pacman::RobotEddie::State begin;
		controller->lookupState(controller->time(), begin);
		// Prepare trajectory
		pacman::RobotEddie::Command::Seq commands(5);
		// Initial pose
		commands[0].armRight.pos = begin.armRight.pos;
		// Pose #1
		commands[1].t = commands[0].t + pacman::float_t(5.0);
		commands[1].armRight.pos = begin.armRight.pos;
		commands[1].armRight.pos.c[5] += pacman::float_t(0.5);
		// Pose #2
		commands[2].t = commands[1].t + pacman::float_t(5.0);
		commands[2].armRight.pos = begin.armRight.pos;
		commands[2].armRight.pos.c[5] -= pacman::float_t(0.5);
		// Pose #3
		commands[3].t = commands[2].t + pacman::float_t(5.0);
		commands[3].armRight.pos = begin.armRight.pos;
		commands[3].armRight.pos.c[6] += pacman::float_t(0.5);
		// Pose #4
		commands[4].t = commands[3].t + pacman::float_t(5.0);
		commands[4].armRight.pos = begin.armRight.pos;
		commands[4].armRight.pos.c[6] -= pacman::float_t(0.5);

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