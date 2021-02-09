# fingers controller

 Copyright (c) 2021 Iason Batzianoulis
 
 License:    GNU GPL v3

## propostional controller:

A ros-node that listens to the contact status on the fingertips from bioTac sensors and the high-level command comming from an EMG decoder for computing the torques on the fingertips for holding an object and/or apply a rotation to it. No control-loop is implemented on the generation of torques making this controller an open-loop controller. The main assumption of the contoller is that the object to grasp/rotate is a cuboid and the index and middle fingers approach the object from the opposite object's side/face than the thumb.

The initial and final poses of the fingertips, the number of fingertips equipped with the bioTac sensors, the maximum magnitude of the rotational torque and the applied force on each side of the object are defined in the file prop_rot_params.yaml, which loaded from the launcher and read from the node's initialization. 

The ros-node has three subscribers to:
1. listen to the pose of the fingertips (ros-message: geometry_msgs::PoseArray)
2. listen to the contact status from the bioTac sensors (ros-message: biotac_interface::biotacMsg)
3. listen to the high-level commands from an EMG-decoder (ros-message: emg_interface::emg_msg)

The ros-node has two publishers to:
1. publish the desired pose of the fingertips
2. publish the desired torques for the fingertips

To lanch the proportional controller, type
```bash
$ roslaunch fingers_controller prop_rot.launch
```

To lanch a ros-node to send custom commands to the controller, type:
```bash
$ rosrun fingers_controllerest_prop
```