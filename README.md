# PaCMan

## Overview

PaCMan demo.

## Dependencies

* Boost >= 1.46 (PCL)
* FLANN >= 1.7.1 (PCL)
* Eigen >= 3.0 (PCL)
* VTK >= 5.6 (PCL)
* OpenNI 1.5.4 (PCL)
* PCL 1.7
* ROS Hydro (uibk, unipi)
* OpenCV >= 2.4.6 (bham)
* Expat (bham)
* Freeglut (bham)
* NVIDIA PhysX 2.8 (bham)

All of them can be installed from the synaptic/apt-get standard repos.

### Installation of ROS Hydro

### General prerequisites

* `sudo apt-get install cmake build-essential`

* `sudo apt-get install git`

* `sudo apt-get install cmake-gui`

* Boost installation: `sudo apt-get install libboost-date-time1.46.1 libboost-date-time1.46-dev libboost-filesystem1.46.1 libboost-filesystem1.46-dev libboost-iostreams1.46.1 libboost-iostreams1.46-dev libboost-mpi1.46.1 libboost-mpi1.46-dev libboost-serialization1.46.1 libboost-serialization1.46-dev libboost-system1.46.1 libboost-system1.46-dev libboost-thread1.46.1 libboost-thread1.46-dev`

### PCL

* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'`

* `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`

* `sudo apt-get update`

* `sudo apt-get install libflann1 libflann-dev libeigen3-dev libvtk5.8 libvtk5.8-qt4 libvtk5-dev libvtk5-qt4-dev libopenni-dev`

### [Installation PCL 1.7 (from sources)](http://pointclouds.org/downloads/source.html)

* `git clone https://github.com/PointCloudLibrary/pcl pcl-trunk`

* `cd pcl-trunk && mkdir BUILD && cd BUILD`

* `cmake-gui ..`

* Press **Configure** and then choose Unix makefiles target.

* Set **Grouped** to group options.

* In **BUILD** group set options: **BUILD_apps** 

* Press **Configure** and then choose **BUILD_app_3d_rec_framework**.

* Press **Configure** and then **Generate**.

* Build and install PCL: `sudo make install`

* In some architectures, PCL and dependant libs such as Grasp, might present compilation problems regarding the lack of low level instructions. If so, edit PCLROOT/cmake/pcl_find_sse.cmake with your available hardware options `-march=CPU-TYPE (for Ubuntu 12.04, gcc-4.6.3) [here](http://gcc.gnu.org/onlinedocs/gcc-4.6.3/gcc/Submodel-Options.html#Submodel-Options)


## PaCMan libraries


### Getting

Suggested folder tree (althought not strictly necessary)

* `mkdir PACMAN_ROOT`

* `cd PACMAN_ROOT`

* `git clone https://github.com/pacman-project/pacman.git`

* `git clone https://github.com/pacman-project/poseEstimation.git`

* `git clone https://github.com/pacman-project/Golem.git`

* `git clone https://github.com/pacman-project/Grasp.git`

And build the partner libraries following their instructions. It is advised to work locally, instead of installing.

### Build

* cd `/PATH/TO/PACMAN_ROOT/pacman`

* `mkdir build`

* `cd build`

* `cmake-gui ..`

* Set the GRASP_{INCLUDE,BINARIES,LIBRARY}, GOLEM_{INCLUDE,BINARIES,LIBRARY,RESOURCES} and UIBK_POSE_ESTIMATION_EXTERNALLIB paths to point where partner libraries are.

* `make`

NOTE: By default, ROS packages are not built, since it is less likely that people has ROS installed. To enable this, check the `BUILD_ROS_PKGS` option. In such case, you need to follow the next subsection instructions before building. 

### Configuration the [ROS](http://wiki.ros.org/groovy/Installation/Ubuntu#groovy.2BAC8-Installation.2BAC8-DebEnvironment.Environment_setup)[/catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) environment:

Before building the pacman software, ensure you have the ROS environment lodaded. For the current terminal session, type:

* `source /opt/ros/hydro/setup.bash`

To make ROS environment available for all sessions type:

* `echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc`

After the pacman software is built, you need to load also the created catkin workspace in order to use the pacman ROS packages. For For the current terminal session, type:

* `source /PATH/TO/PACMAN_ROOT/catkin/devel/setup.bash`

Again, To make the catkin environment available for all sessions type:

* `echo "source /PATH/TO/PACMAN_ROOT/catkin/devel/setup.bash" >> ~/.bashrc`

### Changing Golem/Grasp configuration files

All required files should be in the /PATH/TO/PACMAN_ROOT/bin folder. If you need to modify any of these, please, do it here, and type

* `cd /PATH/TO/PACMAN_ROOT/pacman/build`

*`make`






