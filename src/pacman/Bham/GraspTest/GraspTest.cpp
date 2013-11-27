#include <pacman/Bham/Grasp/Grasp.h>
#include <exception>

using namespace pacman;

int main(int argc, char *argv[]) {
	try {
		if (argc < 2) {
			printf("GraspTest <configuration_file>\n");
			return 1;
		}

		// create grasp
		BhamGrasp* grasp = BhamGrasp::create(argv[1]);
		
		// load and add initial training data
		Point3D::Seq points;
		load("pacman_mug1.pcd", points);
		RobotUIBK::Config::Seq trajectory;
		load("pacman_mug1.trj", trajectory);
		grasp->add("pacman_mug1", points, trajectory);
		
		// run service here and comment out the 3 lines of code below

		// find grasp on an object
		BhamGrasp::Trajectory::Seq trajectories;
		grasp->estimate(points, trajectories);

		// pass control to the application
		grasp->spin();
	}
	catch (const std::exception& ex) {
		printf("GraspTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}