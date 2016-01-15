#!/usr/bin/env python

import rospy
from hros5_gazebo.hros5 import HROS5


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating HR-OS5 Client")
    hros5=HROS5()
    rospy.sleep(1)
 
    rospy.loginfo("HR-OS5 Walker Demo Starting")


    hros5.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)
    hros5.set_walk_velocity(1,0,0)
    rospy.sleep(3)
    hros5.set_walk_velocity(0,1,0)
    rospy.sleep(3)
    hros5.set_walk_velocity(0,-1,0)
    rospy.sleep(3)
    hros5.set_walk_velocity(-1,0,0)
    rospy.sleep(3)
    hros5.set_walk_velocity(1,1,0)
    rospy.sleep(5)
    hros5.set_walk_velocity(0,0,0)
    
    rospy.loginfo("HR-OS5 Walker Demo Finished")