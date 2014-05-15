## THIS FILE DEFINES THE SCHUNK DEXTREOUS HAND STATE/COMMAND

geometry_msgs/PoseStamped wrist_pose

# order convention for the joints w.r.t. pacman Defs rotation[0], left[1,2], right[3,4], thumb[5,6] ;
# "left/right_sdh_knuckle_joint";
# "left/right_sdh_finger_12_joint";
# "left/right_sdh_finger_13_joint";
# "left/right_sdh_finger_22_joint";
# "left/right_sdh_finger_23_joint";
# "left/right_sdh_thumb_2_joint";
# "left/right_sdh_thumb_3_joint";
float32[] joints
float32[] velocity
float32[] acceleration
float32[] effort
float32[] stiffness