#include <pacman/UIBK/PoseEstimation/PoseEstimationImpl.h>
#include <exception>

using namespace pacman;

int main(int argc, char *argv[]) 
{
	try {
		if (argc < 3) 
                {
			printf("PoseEstimationTest <configuration_file>\n");
			return 1;
		}

                UIBKPoseEstimation* pose = UIBKPoseEstimation::create(argv[1]);
		        UIBKObject* object = UIBKObject::create(argv[2]);
                Point3D::Seq points_;

                pose->capture(points_);

                UIBKPoseEstimation::Pose::Seq poses_;

                pose->estimate(points_,poses_);
	}
	catch (const std::exception& ex) 
        {
		printf("PoseEstimationTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}
