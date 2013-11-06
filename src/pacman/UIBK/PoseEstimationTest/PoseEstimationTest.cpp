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

				printf("====== OBJECTS ======\n");
				std::vector<std::string> objects_;
				object->list(objects_);
				for (auto i: objects_)
					printf("%s\n", i.c_str());

				printf("====== POSES ======\n");
				Point3D::Seq outpoints_;
				for (auto i: poses_) {
					object->get(i.id, outpoints_);
					printf("%s: weight=%f, pose=(%f, %f, %f), size=%d\n", i.id.c_str(), i.weight, i.pose.p.x, i.pose.p.y, i.pose.p.z, outpoints_.size());
				}
	}
	catch (const std::exception& ex) 
        {
		printf("PoseEstimationTest exception: %s\n", ex.what());
		return 1;
	}
	return 0;
}
