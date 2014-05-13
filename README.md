# PaCMan

## Overview

PaCMan demo.

## Dependencies

1.- General tools:

* `sudo apt-get install cmake build-essential`
* `sudo apt-get install git`
* `sudo apt-get install cmake-gui`

2.- External libs that can be installed in synaptic/apt-get:

* Boost >= 1.48 (PCL/bham)
* FLANN >= 1.7.1 (PCL)
* Eigen >= 3.0 (PCL)
* VTK >= 5.6 (PCL)
* OpenNI 1.5.4 (PCL)
* OpenCV >= 2.4.6 (bham)
* Expat (bham)
* Freeglut (bham)

Alternatively, dependencies for PCL can be installed following the instructions given at [Install PCL 1.7 from sources](http://pointclouds.org/downloads/source.html).

3.- For the Golem Library, you will need the all PhysX libs in the compressed folder which correspond to your architecture. You can download it from here.

* [NVIDIA PhysX 2.8](https://www.dropbox.com/sh/2o9e4sgt6xp0e5c/FhYfhRLmvt) (bham) 

4.- ROS system and components can be installed following the instructions from their [website](http://wiki.ros.org/hydro/Installation/Ubuntu), we summarize it here for our settings:

* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'`
* `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`
* `sudo apt-get update`
* `sudo apt-get install ros-hydro-desktop-full`
* `sudo apt-get install ros-hydro-moveit-full ros-hydro-octomap ros-hydro-octomap-msgs ros-hydro-openni-launch ros-hydro-openni2-launch` (if you find you need to install additional packages, please complete this list)

5.- PCL library should be installed using our forked repo which is already configured for our settings:

* `git clone https://github.com/pacman-project/pcl.git pcl-trunk-Feb-11-2014`
* `cd pcl-trunk-Feb-11-2014 && mkdir build && cd build`
* `cmake ..`
* `make`
* `sudo make install`

NOTE: In some architectures, PCL and dependant libs such as Grasp and PaCManGrasp, might present compilation problems regarding the lack of low level instructions which are CPU-dependant. If so, edit the file `pcl-trunk-Feb-11-2014/cmake/pcl_find_sse.cmake` at line 18 with your available hardware option `-march=CPU-TYPE` (for Ubuntu 12.04, gcc-4.6.3 options are found here [here](http://gcc.gnu.org/onlinedocs/gcc-4.6.3/gcc/Submodel-Options.html#Submodel-Options) ). This flag is required to compile anything taht depends on PCL.

## PaCMan libraries

### Suggested folder tree

Althought not strictly necessary, the suggested folder tree can be created as:

* `mkdir PACMAN_ROOT`
* `cd PACMAN_ROOT`
* `git clone https://github.com/pacman-project/poseEstimation.git`
* `git clone https://github.com/pacman-project/Golem.git`
* `git clone https://github.com/pacman-project/Grasp.git`
* `git clone https://github.com/pacman-project/pacman.git`

You need to build the external libraries before building the pacman libraries, because you need to point to where the libraries were build. Follow the instructions in each of the repos.

It is advised not to install the libraries in the system when testing.

### Build

* cd `/PATH/TO/PACMAN_ROOT/pacman`
* `mkdir build`
* `cd build`
* `cmake-gui ..`

* Set the GRASP_{INCLUDE,BINARIES,LIBRARY} and GOLEM_{INCLUDE,BINARIES,LIBRARY,RESOURCES}. Set the UIBK_POSE_ESTIMATION_EXTERNALLIB to point to the root folder of the pose estimation library. Note that, the build directory of the library must be named `build` (case sensitive).

* `make`

NOTE: In case of linker problems (Grasp + PCL) the PCL library should be recompiled using revision: 3cd3608931257c238729f595032b2ffebd9b4698; Author: Jochen Sprickerhof <github@jochen.sprickerhof.de>; Date: 2013-10-28 20:13:08; Message: Merge pull request #340 from juagargi/static_lib_fix; Fix bug to allow static library creation


NOTE: By default, ROS packages are not built, since it is less likely that people has ROS installed. To enable this, check the `BUILD_ROS_PKGS` option and recall the current support is for ROS/hydro. In such case, you need to follow the next subsection instructions before building.

### Configuration of the [ROS](http://wiki.ros.org/groovy/Installation/Ubuntu#groovy.2BAC8-Installation.2BAC8-DebEnvironment.Environment_setup)[/catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) environment:

Before building the pacman software, ensure you have the ROS environment lodaded. For the current terminal session, type:

* `source /opt/ros/hydro/setup.bash`

To make ROS environment available for all sessions type:

* `echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc`

After the pacman software is built, you need to load also the created catkin workspace in order to use the pacman ROS packages. For For the current terminal session, type:

* `source /PATH/TO/PACMAN_ROOT/catkin/devel/setup.bash`

Again, To make the catkin environment available for all sessions type:

* `echo "source /PATH/TO/PACMAN_ROOT/catkin/devel/setup.bash" >> ~/.bashrc`

### Changing configuration files for Golem/Grasp/PoseEstimation

All configuration files required for the ros packages to run should be in the /PATH/TO/PACMAN_ROOT/bin folder. If you need to modify any of these, please, do it here, and then make the pacman project again

* `cd /PATH/TO/PACMAN_ROOT/pacman/build`
* `make`

which automatically update the files in the catkin workspace.

### Configuring poseEstimation module:

Before using poseEstimation node in ROS, OpenNI has to be properly installed from ROS repositories. In order to do so the default installation of PCL 1.7 from repositories has to be completely removed from the system. Then run: 

* `sudo apt-get install ros-hydro-openni-launch` this will install, appropriate for ROS, OpenNI drivers and the whole PCL 1.7 from ROS-hydro repositories

## Running PACMAN Software

This section describes how to start and use the different robots used in the project (Eddie, Boris, ...)


### Running Eddie

**Before** starting any software application, first of all make sure that the **2 Kuka LWR**, the **2 Schunk hands** and the **KIT Head** are powered up and that no emergency stop is active.
The KIT Head should be placed in the "Zero position", before starting the software


#### Kuka robot controllers (KRC)

* Configure the tool number if required `Configure-->Set tool/base` and select the right tool number (Number 1 for the Right arm and Number 2 for the Left Arm, the tool name should be "sdh2")
* Press OK to confirm
* Select the scripts `golemEddieR` and `golemEddieL` for each robot in the `KRC:\R1\Program\golem` repository
* Run the scripts for the 2 arms until you reach the line `wait for ($FriQuality==#PERFECT)`


#### Control PC

Open a new terminal
* `cd /home/master/Projects/HRController/bin`


To run the demonstration software
* `./GolemDemoTrajectory GolemDemoTrajectoryRobotEddie.xml`

Wait for all the controllers to be initialized e.g, the KIT Head will start moving.
When the graphical interface appears you should be able to run the configured trajectories.
* Select the window with the 3D view
* Press the `SPACE Key` to see the available trajectories
* Type the trajectory number that you would like to execute: 1(Low velocity), 2(Low velocity) or 3(High velocity)
* Press the `ENTER Key` to start the trajectory


To run the server
* `./GolemDeviceCtrlPhysServer`


#### Ros control PC
`To do ...`


### Running Boris
`To do ...`



#### TRICKS

It is possible to run parts of the real robot and simulate the others without having to recompile the code.
They are *.xml configuration files in the `/home/master/Projects/HRController/bin` folder which can be modified to specify the desired configuration.

The configuration file for the server is `GolemDeviceCtrlPhysServer.xml` and the one for the demonstration software is `GolemDemoTrajectoryRobotEddie.xml`. In the `GolemDeviceCtrlPhysServer.xml` you will find a line with                     
`<controller library_path="GolemDeviceMultiCtrl" config_path="GolemDeviceRobotEddie" ... >`

`GolemDeviceMultiCtrl(.so)` - is the library used                                                                      
`GolemDeviceRobotEddie(.xml)` - is the configuration file for the robot


<dl><dt>To simulate the entire robot</dt></dl>
* Change the **config_path** variable to `GolemDeviceRobotEddieSim`. You can open the `GolemDeviceRobotEddieSim.xml` file to check the configuration
* Save the file

<dl><dt>To simulate parts of the entire robot</dt></dl>
* Change the **config_path** variable to `GolemDeviceRobotEddie`. 
* Open the file `GolemDeviceRobotEddie.xml` file.


In the beginning of the file you will see 5 lines with the prototype                                   
`<controller library_path="MyRobotDynamicLibrary" config_path="MyRobotXmlConfigurationFile" ... >`

* Change the **library_path** to `MyRobotDynamicLibrarySim` to use the simulator instead
* Save the file

eg. To configure the Left Kuka LWR  and the Left Schunk gripper for simulation update the configuration file with       
`<controller library_path="GolemDeviceKukaLWRSim" config_path="GolemDeviceKukaLWREddieL" ... >`                        
`<controller library_path="GolemDeviceSchunkDexHandSim" config_path="GolemDeviceSchunkDexHandEddieL" ... >`

<dl><dt>To change the temperature limits for the Schunk gripper</dt></dl>
* Open the `GolemDeviceSchunkDexHandEddieL.xml` and `GolemDeviceSchunkDexHandEddieR.xml`
* In the field `<temp> ... <\temp>` change the values.                                                      
	`on` - Maximum temperature allowed                                                             
	`off` - Minimum Hysterisis temperature to restart the gripper after a temperature error
* Save the files


#### WARNING: 
**they are still minor problems during the initialization of the Schunk grippers and the KIT Head (Control PC section).**

If you see the error:

* Schunk gripper

	>Waiting for Schunk device to be ready (RS232)...
	
	>Segmentation fault (core dumped)

	==> Execute again the application until the connection is accepted by the Schunk device

* KIT Head

	The Head should be in the "Zero position" **BEFORE** running the demontsration software, especially the Neck roll, Neck pitch and Neck yaw axis, otherwise some axis will not be initialized properly.

