// File Path

/ros_ws/src/baxter_common/rethink_ee_description/urdf/electric_gripper/fingers/standard_narrow_update.xacro

/ros_ws/src/baxter_common/baxter_description/urdf/left_end_effector_hook.urdf.xacro

/ros_ws/src/baxter_common/baxter_description/urdf/right_end_effector_hook.urdf.xacro


// Update Gripper

./baxter/sh

rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/left_end_effector_hook.urdf.xacro -l left_hand -j left_gripper_base

(Successful if no output. And open a new terminal tab)

./baxter.sh

rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/right_end_effector_hook.urdf.xacro -l right_hand -j right_gripper_base
