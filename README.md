# ros_hros5

This is a work-in-progress version of a functioning ROS stack for the HR-OS5 robot. It's been composed of several open source projects and quite a bit of my time. I hope you find this useful. 

Current hardware configuration:

  - HR-OS5 Endoskeleton
  - Arbotix-Pro Controller
  - NUC i5 8GB/32GB
  - PrimeSense
  - Intel R200
  - Sony DS4 Controller

### Version
0.0.1

### System Configuration

 - Ubuntu 14.04
 - ROS Indigo

### Dependencies

This I do not have compiled yet. Many packages were installed.

* TODO

#### Important:
hros5_hardware_interface.cpp, lines 86-87 specify the motion binary and walk gait files are located in /robotis/Data/
These are quite important to have in place. Ideally these should be loaded from $(find hros5_framework)/Data/

### Launching

##### No Sensors

To launch without extra sensors and mapping (ie. Teleoperation only)
```
roslaunch hros5_bringup basic_no_mapping.launch
```

##### PrimeSense
To launch with a PrimeSense or ASUS Xtion (defined to be mounted on top of the head tilt servo):

```
roslaunch hros5_bringup primesense_local_debug.launch
```

##### Intel R200
To launch with an Intel R200 (defined to be mounted on the front of the head tilt servo) I'm currenly performing a little work around to allow RTAB-Map to work at the R200's optimal resolution.

First launch your R200 camera. 
```
roslaunch realsense realsense_r200_launch.launch
```

I should note my camera is configured to launch with the following modifications:
```
  <arg name="cHeight" default="480" />
  <arg name="cWidth" default="640" />
  <arg name="dHeight" default="468" />
  <arg name="dWidth" default="628" />
```
Second the depth frame resizer:
```
rosrun resize_image resize_image -i /camera/depth/image_raw -o /camera/depth/image_resized --aw 640 --ah 480
```

Finally, the ROS stack:
```
roslaunch hros5_bringup r200_local_debug.launch
```

### Todos

 - Many, search TODO in the source. 
 - Gazebo
 - MoveIt


