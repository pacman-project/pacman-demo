#include <pacman/Bham/Grasp/Grasp.h>
#include <exception>

#include <pacman/Bham/Grasp/GraspImpl.h> // Fast method

using namespace pacman;

int main(int argc, char *argv[]) {
	try {
		if (argc < 2) {
			printf("GraspTest <configuration_file>\n");
			return 1;
		}

		// create grasp
		BhamGrasp::Ptr grasp = BhamGrasp::create(argv[1]);
		
		// load grasp data
		grasp->load("pacman.graspclass");
		
		// run service here and comment out all the code lines below in this block

		// Slow method: compute curvatures, then compute grasp trajectories
		//Point3D::Seq graspPoints;
		//load("pacman_container_1.pcd", graspPoints);
		//BhamGrasp::Trajectory::Seq graspTrajectories;
		//grasp->estimate(graspPoints, graspTrajectories);

		// Fast method: load pre-computed point cloud with curvatures, then compute grasp trajectories
		grasp::Cloud::PointSeq graspPointsWithCurvature;
		// uncomment to convert
		//Point3D::Seq graspPoints;
		//load("pacman_container_1.pcd", graspPoints);
		//static_cast<BhamGraspImpl*>(grasp.get())->convert(graspPoints, graspPointsWithCurvature);
		//pcl::PCDWriter().writeBinaryCompressed("pacman_container_1_curv.pcd", graspPointsWithCurvature);
		pcl::PCDReader().read("pacman_container_1_curv.pcd", graspPointsWithCurvature);
		BhamGrasp::Trajectory::Seq graspTrajectories;
		static_cast<BhamGraspImpl*>(grasp.get())->estimate(graspPointsWithCurvature, graspTrajectories);

		// pass control to the application
		grasp->spin();
	}
	catch (const std::exception& ex) {
		printf("GraspTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}