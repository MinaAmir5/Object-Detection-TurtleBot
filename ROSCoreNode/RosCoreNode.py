import rospy
from geometry_msgs.msg import Twist, Point, Pose2D
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from math import atan2, sqrt, pow
from ultralytics import YOLO
import torch
import joblib

# Initialize the ROS node and publisher globally
rospy.init_node('publisher_node', anonymous=True)
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.Subscriber('Encoder', Pose2D, lambda data: positionCallback(data))
rospy.Subscriber('trash', Float32MultiArray, lambda data: cameraCallback(data))
rate = rospy.Rate(10)

vel_msg = Twist()
trash = Float32MultiArray()
current_pose = Pose2D()

def cameraCallback(data):
    global trash
    trash = data
    #rospy.loginfo(f"closest distance: {trash.data[0]}, closest angle: {trash.data[1]}")
    
def positionCallback(data):
    global current_pose
    current_pose = data
    current_pose.x = round(current_pose.x, 4)
    current_pose.y = round(current_pose.y, 4)
    #rospy.loginfo(f"Robot Position: x={self.current_pose.x}, y={self.current_pose.y}, theta={self.current_pose.theta}°")

if __name__ == '__main__':
    try:
        goToGoal_Model = joblib.load("/home/mina/Music/Neural_Network_model2.pkl")
        obstAvoid_Model = joblib.load("/home/mina/Music/Neural_Network_model1.pkl")
        
        rospy.loginfo("Node started")
        trash.data = [1000, 0, 50]
        rospy.sleep(1)
        
        while True:
            rospy.loginfo(f"{trash.data[2]}")
            if(trash.data[0] != 1000 and trash.data[2] == 39):
                rospy.loginfo("Goal found")
                absolute_X = current_pose.x
                absolute_Y = current_pose.y
                absolute_Theta = current_pose.theta
                trash_Temp = trash.data
                while True:
                    if (trash.data[0] < 40):
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        velocity_publisher.publish(vel_msg)
                        break
                    relative_X = (absolute_X - current_pose.x*100) + trash_Temp[0]*np.cos(np.radians(trash_Temp[1]))
                    relative_Y = (absolute_Y - current_pose.y*100) + trash_Temp[0]*np.sin(np.radians(trash_Temp[1]))
                    relative_Distance = sqrt(pow(relative_X, 2) + pow(relative_Y, 2))
                    relative_Theta = (absolute_Theta - current_pose.theta*(180/np.pi)) + trash_Temp[1]
                    rospy.loginfo(f"Relative distance: {relative_Distance}")
                    rospy.loginfo(f"Robot Position: x={current_pose.x}, y={current_pose.y}, theta={current_pose.theta}°")

                    outputs = goToGoal_Model.predict([[relative_Distance ,relative_Theta]])
                    vel_msg.linear.x = outputs[0][0]
                    vel_msg.angular.z = outputs[0][1]
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(0.02)

                    bot.move_to_goal()
                    rospy.sleep(0.01)
            elif (trash.data[0] != 1000 and trash.data[2] == 41):
                rospy.loginfo("Obstacle found")
                while True:
                    results = obstAvoid_Model.predict([[trash.data[0], trash.data[1]]])
                    vel_msg.linear.x = results[0][0]
                    vel_msg.angular.z = results[0][1]
                    rospy.loginfo(f"Linear: {vel_msg.linear.x}, Angular: {vel_msg.angular.z}")
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(0.01)
                    
                    if (trash.data[1] >= 60):
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        velocity_publisher.publish(vel_msg)
                        trash.data = [1000, 0]
                        rospy.sleep(1)
                        break
            else:
                vel_msg.linear.x = 0.6
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.01)
                #rospy.loginfo("No objects found")
                
    except rospy.ROSInterruptException:
        pass
