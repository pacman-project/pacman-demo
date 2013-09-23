#include <pacman/UIBK/PoseEstimation/PoseEstimationImpl.h>
#include <exception>

using namespace pacman;


int main(int argc, char *argv[]) {
	try {
		if (argc < 2) {
			printf("GraspTest <configuration_file>\n");
			return 1;
		}

		UIBKPoseEstimation* pose = UIBKPoseEstimation::create(argv[1]);
		UIBKObject* object = UIBKObject::create(argv[1]);

		// TODO
	}
	catch (const std::exception& ex) {
		printf("PoseEstimationTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}