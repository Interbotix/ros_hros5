# ros_hros5

This is a work-in-progress version of a functioning ROS stack for the HR-OS5 robot. It's been composed of several open source projects and a resonable amount of my time. I hope you find this useful.

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
git clone https://github.com/Interbotix/HROS5-Framework ./src/ros_misc/hros5_framework
cd src/ros_misc/hros5_framework/Linux/build/
make
````

ROS packages:
````
sudo apt-get install ros-indigo-joy
sudo apt-get install python-rosinstall
sudo apt-get install ros-indigo-robot-localization
sudo apt-get install ros-indigo-rtabmap-ros
sudo apt-get install ros-indigo-rtabmap
sudo apt-get install ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-joint-state-publisher ros-indigo-controller-manager ros-indigo-imu-sensor-controller ros-indigo-position-controllers 
````

##### Other requirements

Grant local user real-time access to the Arbotix Pro:
````
echo "ulimit -r 31" >> ~/.bashrc
sudo su
echo "<username> hard rtprio 31" >> /etc/security/limits.conf

sudo usermod -a -G dialout <username>
````
##### Optional (configuration dependant)

````
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-pocketsphinx
https://github.com/PercATI/RealSense_ROS
````

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

### Contributing Repositories

 - https://github.com/costashatz/HROS5
 - https://github.com/ROBOTIS-OP/robotis_op_ros_control
 - https://github.com/Interbotix/HROS5-Framework
