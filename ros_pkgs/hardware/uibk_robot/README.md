UIBK Robot
==========

The UIBK robot is composed of two Kuka LWR mounted on a torso, with one of the arm equipped with the Schunk Dexterous Hand. The torso is fixed. 

## Moving the model

At this point, you should have already your ROS installation running and cofigured. 

*Open a terminal and type:

`roslaunch uibk_robot display.launch`

*Open another terminal and type:

`rqt`

*In rqt menu, add an RViz pluggin. In the global options select any of the available frames being pusblished by the robot, typically, "/world_link". The same for grid reference frame.

Remove any other Display Type and add add only a "RobotModel", and optionally, a "TF" to visualize the frames, you can desactivate it without removing it.

*In rqt menu, add a Shell pluggin, this is a terminal where you will publish the desired joint state of the robot with a command like

`rostopic pub --once /joint_states sensor_msgs/JointState "name: ['leftGripper_thumb_2_joint', 'rightArm_arm_1_joint', 'rightGripper_finger_23_joint', 'rightGripper_knuckle_joint', 'rightArm_arm_5_joint', 'leftGripper_finger_12_joint', 'leftGripper_finger_21_joint', 'rightArm_arm_2_joint', 'leftGripper_finger_22_joint', 'rightGripper_finger_21_joint', 'rightGripper_finger_13_joint', 'leftArm_arm_2_joint', 'leftArm_arm_6_joint', 'rightGripper_thumb_3_joint', 'rightArm_arm_3_joint', 'leftArm_arm_1_joint', 'rightArm_arm_6_joint', 'rightArm_arm_4_joint', 'rightGripper_thumb_2_joint', 'leftArm_arm_0_joint', 'leftGripper_thumb_3_joint', 'leftArm_arm_5_joint', 'leftGripper_knuckle_joint', 'rightGripper_finger_12_joint', 'leftArm_arm_4_joint', 'leftGripper_finger_13_joint', 'rightArm_arm_0_joint', 'rightGripper_finger_22_joint', 'leftArm_arm_3_joint', 'leftGripper_finger_23_joint'] 
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`

I'm sorry, you have to manually set the values in RAD to see any motion. You can also use the GUI wchich should be activated as well.