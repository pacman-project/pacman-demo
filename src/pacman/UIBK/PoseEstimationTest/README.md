General prerequisites
---------------------

Download poseEstimation package from https://github.com/pacman-project/poseEstimation.git in the same directory as the pacman folder.
(e.g. pacman_project/pacman
      pacman_project/poseEstimation) as the relative paths refer to this configuration.

In the poseEstimation/data folder put the TRAINED-LOCAL-MODELS and PLY-MODELS from https://github.com/mirelapopa/dataFiles.

Running the Pose Estimation module
---------------------------------
./PaCManUIBKPoseEstimationTest ../pacman/resources/UIBK/PoseEstimation.txt ../poseEstimation/data/recognizedObjects

 - first argument indicates the location of the configuration file 
 - second argument indicates the folder in which the detected objects from a scene captured with Kinect will be saved

