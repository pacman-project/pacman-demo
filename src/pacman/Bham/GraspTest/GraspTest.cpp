#include <pacman/Bham/Grasp/Grasp.h>
#include <pacman/Bham/Grasp/GraspImpl.h>
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

		// Debug
		BhamGraspImpl* graspImpl = (BhamGraspImpl*)grasp;
		// Coordinate system origin (0, 0, 0)
		graspImpl->addFrame(Mat34());
		// Object cloud centroid
		Mat34 centroid;
		for (auto i: points) {
			centroid.p.x += i.position.x; centroid.p.y += i.position.y; centroid.p.z += i.position.z;
		}
		centroid.p.x /= points.size(); centroid.p.y /= points.size(); centroid.p.z /= points.size();
		graspImpl->addFrame(centroid);
		// Hand frame
		Mat34 frame = trajectories.front().trajectory.back().pose;
		graspImpl->addFrame(frame);
		// Centroid - hand frame line
		graspImpl->addLine(centroid.p, frame.p);

		// pass control to the application
		grasp->spin();
	}
	catch (const std::exception& ex) {
		printf("GraspTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}