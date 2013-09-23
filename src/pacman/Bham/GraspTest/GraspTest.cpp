#include <pacman/Bham/Grasp/Grasp.h>
#include <exception>

using namespace pacman;


int main(int argc, char *argv[]) {
	try {
		if (argc < 2) {
			printf("GraspTest <configuration_file>\n");
			return 1;
		}

		BhamGrasp* grasp = BhamGrasp::create(argv[1]);

		// TODO
	}
	catch (const std::exception& ex) {
		printf("GraspTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}