#!/usr/bin/env python3  
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OdomSubscriber():

    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  
        self.odom_data = Odometry()
    
    def odom_callback(self, msg):
        self.odom_data = msg
    
    def get_position(self):
        odom_position = {"x" :self.odom_data.pose.pose.position.x, 
                         "y" :self.odom_data.pose.pose.position.y, 
                         "z" :self.odom_data.pose.pose.position.z}
        return (odom_position)
        
    def get_orientation(self,orientation_choice='quaternion'):
        odom_orientation_quaternion= { "x" :self.odom_data.pose.pose.orientation.x ,
                                    "y" :self.odom_data.pose.pose.orientation.y,
                                    "z" :   self.odom_data.pose.pose.orientation.z, 
                                    "w" :self.odom_data.pose.pose.orientation.w,}
        x  = self.odom_data.pose.pose.orientation.x
        y  = self.odom_data.pose.pose.orientation.y
        z  = self.odom_data.pose.pose.orientation.z
        w  = self.odom_data.pose.pose.orientation.w

        orientation_list =  [x,y,z,w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        odom_orientation_euler = {'roll':roll,'pitch':pitch,'yaw':yaw}
        
        if orientation_choice.lower() == 'euler':
            return odom_orientation_euler
        else :
            return odom_orientation_quaternion
        