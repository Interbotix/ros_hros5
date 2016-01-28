# ros_hros5

This is a work-in-progress ROS package for the HR-OS5 humanoid research platform. It is a collaborate effort composed of several open source projects.

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
The Interbotix HROS5-Framework is required in /src/hros5_misc/hros5_framework

````
git clone https://github.com/Interbotix/HROS5-Framework ./src/hros5_misc/hros5_framework
cd src/hros5_misc/hros5_framework/Linux/build/
make
````

##### ROS packages:
````
sudo apt-get install ros-indigo-joy
sudo apt-get install python-rosinstall
sudo apt-get install ros-indigo-robot-localization
sudo apt-get install ros-indigo-rtabmap-ros
sudo apt-get install ros-indigo-rtabmap
sudo apt-get install ros-indigo-joint-trajectory-controller
sudo apt-get install ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-joint-state-publisher ros-indigo-controller-manager ros-indigo-imu-sensor-controller ros-indigo-position-controllers 
````

##### Optional (configuration dependant)
````
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-pocketsphinx
https://github.com/PercATI/RealSense_ROS
````

##### Other requirements
Grant local user real-time access to the Arbotix Pro:
````
echo "ulimit -r 31" >> ~/.bashrc
sudo su
echo "<username> hard rtprio 31" >> /etc/security/limits.conf

sudo usermod -a -G dialout <username>
````

### Robot Models

Two robot models are now available. The default configuration is the endoskeleton only and the alternate configuration contains a mix of the v2 and v3 3D printable armor/shells as well as a custom head design for the PrimeSense sensor. Expect the alternate configuration to change in time as development progresses. 

To use the alternate visual and collision models specify `hros5_visuals_collisions_armored.xacro` in your launch files.
```
<!-- xacro for visuals/collisions -->
<arg name="meshes_xacro" default="$(find hros5_description)/urdf/hros5_visuals_collisions_armored.xacro"/>
```

### Launching

##### No Sensors
To launch without extra sensors and mapping (ie. Teleoperation only)
```
roslaunch hros5_bringup basic_no_mapping.launch
```

##### MoveIt!
To launch with MoveIt compatibiity bring up the hros5_controller with trajectory controllers then launch the MoveIt Planner.
**NOTE: There are known (joint positioning) issues with using the DarwinOP Action Pages and Walk Gait in conjunction with the MoveIt Planner.
```
roslaunch hros5_bringup basic_no_mapping_traj.launch
roslaunch hros5_moveit_config moveit_planner.launch
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

I should note the R200 is configured to launch with the following modifications:
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

### Contributing Repositories

 - https://github.com/costashatz/HROS5
 - https://github.com/ROBOTIS-OP/robotis_op_ros_control
 - https://github.com/Interbotix/HROS5-Framework
