cmake_minimum_required(VERSION 2.8.3)
project(hros5_gazebo)

## Find catkin
find_package(catkin REQUIRED COMPONENTS gazebo_ros rospy)

catkin_python_setup()

##Needed for ros packages
catkin_package()

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})