PERCEPTION
==========

Build
-----

You need to set the var `POSE_ESTIMATION_EXT` in the CMakeLists.txt to point where the poseEstimation library was built, and check that the build folder is called `build` in there. And you might need to change the paths in `src/pose_estimation_uibk.cpp` for configuration and database location (ToDo: improve this to avoid compiling everytime, e.g. passing the path as a parameter in the launch file).

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.

ToDo: use the pacman interface for this node, such that the path is fixed w.r.t. the PACMAN_ROOT.

Use
---


