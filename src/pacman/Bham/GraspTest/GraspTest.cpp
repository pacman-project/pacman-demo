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
		BhamGrasp::Ptr grasp = BhamGrasp::create(argv[1]);
		
		// load and add initial training data
		Point3D::Seq trainingPoints;
		load("pacman_container_2.pcd", trainingPoints);
		//load("pacman_kettle.pcd", trainingPoints);
		RobotUIBK::Config::Seq trainingTrajectory;
		load("pacman_container_2.trj", trainingTrajectory);
		//load("pacman_kettle.trj", trainingTrajectory);
		grasp->add("pacman_container_2", trainingPoints, trainingTrajectory);
		
		// run service here and comment out all the code lines below in this block

		// find grasp on an object
		Point3D::Seq graspPoints;
		load("pacman_container_1.pcd", graspPoints);
		//load("pacman_kettle.pcd", graspPoints);
		BhamGrasp::Trajectory::Seq graspTrajectories;
		grasp->estimate(graspPoints, graspTrajectories);

		// pass control to the application
		grasp->spin();
	}
	catch (const std::exception& ex) {
		printf("GraspTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}