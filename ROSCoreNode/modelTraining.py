#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, Pose2D
from std_msgs.msg import Float32MultiArray
from math import pow,atan2,sqrt
import cv2
import numpy as np
import sys
import tty
import termios

class turtlebot():

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.error_publisher = rospy.Publisher('error', Pose2D, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('Encoder', Pose2D, self.callback)
        self.pose_subscriber = rospy.Subscriber('trash', Float32MultiArray, self.callback1)
        self.pose = Pose2D()
        self.error = Pose2D()
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.goal_pose = Pose2D()
            
    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)  # Read a single character
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        #rospy.loginfo(f"Robot Position: x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta}°")

    def callback1(self, data):
        self.trash = data
        
    def move2goal(self):
        self.goal_pose.x = float(input("Set your x goal:"))
        self.goal_pose.y = float(input("Set your y goal:"))
        distance_tolerance = float(input("Set your tolerance:"))

        while sqrt(pow((self.goal_pose.x - self.pose.x), 2) + pow((self.goal_pose.y - self.pose.y), 2)) >= distance_tolerance:            
            inputt = self.getch()
                
            if inputt == 'w':
                self.vel_msg.linear.x+=0.1
                self.vel_msg.angular.z=0
                
                #motor1_speed = ((motor1_speed + motor2_speed)/2)+5
                #motor2_speed = motor1_speed
            elif inputt == 's':
                self.vel_msg.linear.x-=0.1
                self.vel_msg.angular.z=0
                #motor1_speed = ((motor1_speed + motor2_speed)/2)-5
                #motor2_speed = motor1_speed
            elif inputt == 'a':
                self.vel_msg.angular.z-=0.1
                #motor1_speed+=5
            elif inputt == 'd':
                self.vel_msg.angular.z+=0.1
                #motor2_speed+=5
            else:
                self.vel_msg.linear.x=0
                self.vel_msg.angular.z=0
                
            #rospy.loginfo(f"vel_msg.linear.x= {self.vel_msg.linear.x}, vel_msg.angular.z ={self.vel_msg.angular.z}")
            
            
            #self.error_publisher.publish(self.error)
            self.velocity_publisher.publish(self.vel_msg)
            
            self.rate.sleep()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        #Stopping our robot after the movement is over
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z =0
        self.velocity_publisher.publish(self.vel_msg)
        

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()

    except rospy.ROSInterruptException: pass