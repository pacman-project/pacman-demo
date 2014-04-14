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

		// RobotUIBK
		// Read current state
		pacman::RobotUIBK::State begin;
		controller->lookupState(controller->time(), begin);
		// Initial configuration command
		pacman::RobotUIBK::Command init;
		init.arm.pos = begin.arm.pos;
		init.hand.pos = begin.hand.pos;
		// Prepare trajectory
		pacman::RobotUIBK::Command::Seq commands(6);
		// Pose #0
		commands[0] = init;
		commands[0].t = controller->time() + pacman::float_t(1.0);
		// Pose #1
		commands[1] = init;
		commands[1].arm.pos.c[5] += pacman::float_t(0.5);
		commands[1].t = commands[0].t + pacman::float_t(5.0);
		// Pose #2
		commands[2] = init;
		commands[2].arm.pos.c[5] -= pacman::float_t(0.5);
		commands[2].t = commands[1].t + pacman::float_t(5.0);
		// Pose #3
		commands[3] = init;
		commands[3].arm.pos.c[6] += pacman::float_t(0.5);
		commands[3].t = commands[2].t + pacman::float_t(5.0);
		// Pose #4
		commands[4] = init;
		commands[4].arm.pos.c[6] -= pacman::float_t(0.5);
		commands[4].t = commands[3].t + pacman::float_t(5.0);
		// Pose #5
		commands[5] = init;
		commands[5].t = commands[4].t + pacman::float_t(5.0);

		//// RobotEddie
		//// Read current state
		//pacman::RobotEddie::State begin;
		//controller->lookupState(controller->time(), begin);
		//// Initial configuration command
		//pacman::RobotEddie::Command init;
		//init.armLeft.pos = begin.armLeft.pos;
		//init.handLeft.pos = begin.handLeft.pos;
		//init.armRight.pos = begin.armRight.pos;
		//init.handRight.pos = begin.handRight.pos;
		//init.head.pos = begin.head.pos;
		//// Prepare trajectory
		//pacman::RobotEddie::Command::Seq commands(6);
		//// Pose #0
		//commands[0] = init;
		//commands[0].t = controller->time() + pacman::float_t(1.0);
		//// Pose #1
		//commands[1] = init;
		//commands[1].armRight.pos.c[5] += pacman::float_t(0.5);
		//commands[1].t = commands[0].t + pacman::float_t(5.0);
		//// Pose #2
		//commands[2] = init;
		//commands[2].armRight.pos.c[5] -= pacman::float_t(0.5);
		//commands[2].t = commands[1].t + pacman::float_t(5.0);
		//// Pose #3
		//commands[3] = init;
		//commands[3].armRight.pos.c[6] += pacman::float_t(0.5);
		//commands[3].t = commands[2].t + pacman::float_t(5.0);
		//// Pose #4
		//commands[4] = init;
		//commands[4].armRight.pos.c[6] -= pacman::float_t(0.5);
		//commands[4].t = commands[3].t + pacman::float_t(5.0);
		//// Pose #5
		//commands[5] = init;
		//commands[5].t = commands[4].t + pacman::float_t(5.0);

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