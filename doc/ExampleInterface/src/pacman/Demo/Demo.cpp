#include <pacman/RobotKuka/RobotKuka.h>
#include <exception>
#include <iostream>

int main() {
	// always catch exceptions
	try {
		using namespace pacman;

		Robot* robot = createRobotKuka("RobotKuka.cfg");

		// print robot name
		std::cout << "Robot name: " << robot->getName() << std::endl;

		// print robot configuration
		Robot::Configuration c;
		robot->getConfiguration(c);
		std::cout << "Robot configuration: (";
		for (Robot::Configuration::const_iterator i = c.begin(); i != c.end(); ++i)
			std::cout << *i << ", ";
		std::cout << ")" << std::endl;

		// generate error
		c.resize(1);
		robot->moveConfiguration(c);
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}
	return 0;
}