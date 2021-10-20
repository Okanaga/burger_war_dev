#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 2 up

import rospy
import random

from geometry_msgs.msg import Twist

import tf
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

PI = math.pi
deg_15 = PI/12
deg_20 = PI/9
deg_45 = PI/2
deg_65 = PI/36*13
deg_100=PI/9*5
deg_120 = PI/3*2
deg_150 =PI/6*5
deg_160 = PI/9*8
deg_165 = deg_150+deg_15

class NaviBot():
    def __init__(self):
        print(self)
        # velocity publisher
        self.speed = 0.1
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #self.target_id_sub = rospy.Subscriber('war_state', String, self.get_war_state)
        #print("alksdjlfkajlgkjdald",self.target_id_sub)



    def setGoal(self,x,y,yaw):#kawaranai
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def strategy(self):
        r = rospy.Rate(10) # change speed 5fps

        while not rospy.is_shutdown():
            self.setGoal(-0.9,0,0)
            self.setGoal(-0.9,0,deg_65)
            self.setGoal(-0.9,0,-deg_65)

            self.setGoal(0,0.8,-deg_20)
            self.setGoal(0,0.8,-deg_45)
            self.setGoal(0,0.85,-deg_160)

            self.setGoal(0.85,0.25,-deg_100)
            self.setGoal(0.85,0.25,-deg_150)
            self.setGoal(0.85,0.25,deg_120)

            self.setGoal(0,-0.8,deg_15)
            self.setGoal(0,-0.8,deg_45)
            self.setGoal(0,-0.8,deg_165)
            """"
            self.setGoal(-0.9,0,0)
            self.setGoal(-0.9,0,deg_60+deg_5)
            self.setGoal(-0.9,0,-(deg_5+deg_60))

            self.setGoal(0,0.8,-deg_10*2)
            self.setGoal(0,0.8,-deg_45)
            self.setGoal(0,0.85,-(PI-deg_10*2))

            self.setGoal(0.85,0.25,-deg_10*10)
            self.setGoal(0.85,0.25,-deg_120-deg_10*3)
            self.setGoal(0.85,0.25,deg_120)

            self.setGoal(0,-0.8,deg_10+deg_5)
            self.setGoal(0,-0.8,deg_45)
            self.setGoal(0,-0.8,PI-(deg_10+deg_5))
            """

        
        """
        self.setGoal(-0.5,0,3.1415/2)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415)
        
        self.setGoal(-0.5,0,-3.1415/2)
        
        self.setGoal(0,-0.5,0)
        self.setGoal(0,-0.5,3.1415)
            """



if __name__ == '__main__':
    rospy.init_node('test')
    bot = NaviBot()
    bot.strategy()
