#hros5

##ROS Stack for the HR-OS5 Humanoid Research Platform

###WORK IN PROGRESS

Version
----

0.0.1 (Beta)

Requirements
-----------

hros5 requires several packages to be installed in order to work properly:

* [ROS] - ROS >= **Hydro**
* [ROS Control] - **Version >=0.6.0**
* [ROS MoveIt!] - Used for motion planning

Basic Usage
--------------

###MoveIt Demo
```sh
roslaunch hros5_moveit_config demo.launch
```

*This will open up **rviz with the Navigation plugin configured** to give to a fake HR-OS5 trajectories to execute.

![Alt text](https://bitbucket.org/costashatz/hros5/raw/master/MoveIt!Example.png)

###Plan with MoveIt

```sh
roslaunch hros5_moveit_config moveit_planner.launch
```

*This requires that you have connected to the Robot/Simulation* and will open up **rviz with the Navigation plugin configured** to give HR-OS5 trajectories to execute.

![Alt text](https://bitbucket.org/costashatz/hros5/raw/master/MoveIt!Example2.png)

###Gazebo Simulation
```sh
roslaunch hros5_gazebo hros5_gazebo.launch
```

![Alt text](https://bitbucket.org/costashatz/hros5/raw/master/GazeboExample.png)

This will **launch Gazebo Simulator** and **trajectory controllers** to simulate the real robot. You can use MoveIt to give trajectories to HR-OS5 to execute.

![Alt text](https://bitbucket.org/costashatz/hros5/raw/master/GazeboExample2.png)

License
----

BSD


Copyright (c) 2014-2015, **Konstantinos Chatzilygeroudis**

[ros]: http://www.ros.org
[ros control]: http://wiki.ros.org/ros_control
[ros moveit!]: http://moveit.ros.org/
