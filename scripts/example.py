#!/usr/bin/env python3
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
import CameraProcessing
import math
import PID as PID
from utils import go_straight, turn_lef_right, go_back, set_velocities

safty_range = 0.6

class Example(object):

    def __init__(self):
        rospy.loginfo("[Example] loaging")
        rospy.on_shutdown(self.shutdown)

        self.gui = os.getenv('GUI')=='true' or os.getenv('GUI')=='True'
        self.camera_subscriber = rospy.Subscriber("/head/camera1/image_raw", Image, self.camera_cb)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.range_front_subscriber = rospy.Subscriber("/range/front", Range, self.range_front_callback)
        self.sonar_left_subscriber = rospy.Subscriber('/range/left', Range, self.range_left_callback)
        self.sonar_rear_subscriber = rospy.Subscriber('/range/rear', Range, self.range_rear_callback)
        self.sonar_right_subscriber = rospy.Subscriber('/range/right', Range, self.range_right_callback)

        self.odometry = Odometry()
        self.command = Twist()

        self.sonar_data = [0, 0, 0, 0]

        self.curent_image = None
        self.error_x = None
        self.error_y = None
        self.isBlue = None
        self.isRed = None
        self.bridge = CvBridge()
        self.obj_detection = None
        self.control = None
        self.kp = 0.1234
        self.pid_controller = PID.PID(self.kp, 0.001, 0.15)
        self.center_img = None
        self.rate = rospy.Rate(100) # 100 Hz
        # for wall information
        self.is_obstacle_left = False
        self.is_obstacle_right = False
        self.is_obstacle_front = False

        rospy.loginfo("[Example] loaded")

    def shutdown(self):
        # stop robots here
        self.cmd_vel.publish(Twist())

    def camera_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.current_image = frame
        except CvBridgeError as e:
            rospy.logerr(e)
        # Obj detect and calculate error 
        self.obj_detection = CameraProcessing.color_detector(self.current_image)
        center_blue = self.obj_detection[1]
        center_red = self.obj_detection[2] 
        
        self.center_img = [frame.shape[1]/2, frame.shape[0]/2]
        print(f"center img : {self.center_img}")
        
        # print(f"Center of image is {self.center_img}")

        if len(center_blue) > 0:
            self.isBlue = True
            x_blue = center_blue[0]
            y_blue = center_blue[1]
            # print(f"Blue: {self.isBlue}")
            self.error_x = center_blue[0] - self.center_img[0]
            self.error_y = center_blue[1] - self.center_img[1]
        else:
            self.isBlue = False

        if len(center_red) > 0:
            self.isRed = True
            # print(f"Red: {self.isRed}")
            x_red = center_red[0]
            y_red = center_red[1]
            self.error_x = center_red[0] - self.center_img[0]
            self.error_y = center_red[1] - self.center_img[1]
        else:
            self.isRed = False

        #display from onbourd camera
        if self.gui != False:
            cv2.imshow("output", frame)
            cv2.waitKey(1)

    def range_front_callback(self, msg):
        self.sonar_data[0] = msg.range
        if msg.range > 0.7:
            self.is_obstacle_front = False
        else:
            self.is_obstacle_front = True
    def range_left_callback(self,msg):
        self.sonar_data[1] = msg.range
        if msg.range > 0.6:
            self.is_obstacle_left = False
        else:
            self.is_obstacle_left = True 
    def range_rear_callback(self, msg):
        self.sonar_data[2] = msg.range
    def range_right_callback(self, msg):
        self.sonar_data[3] = msg.range
        if msg.range > 0.6:
            self.is_obstacle_right = False
        else:
            self.is_obstacle_right = True 

    def odom_callback(self, msg: Odometry):
        self.odometry = msg
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = msg.pose.pose.position.z
    def bound_control_signal(value):
        if abs(value) > 0.5:
            value = value/(abs(value)*2)
        return value
    

    def spin(self):

        t0 = rospy.get_time()
        # get infor params from images
        while not rospy.is_shutdown():

            t = rospy.get_time() - t0
    
            # khong phat hien thi chay thang
            if self.isBlue == False and self.isRed == False:
                if self.is_obstacle_front == False:
                    self.command = go_straight(0.45)
                    self.cmd_vel.publish(self.command)
                else:
                    if  self.is_obstacle_left == False and self.is_obstacle_right==True: # turn left
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, 0.34)
                    if self.is_obstacle_left == True and self.is_obstacle_right == False: # turn right
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, -0.34)
                    if self.is_obstacle_left == False and self.is_obstacle_right == False: # can turn or right => turn left
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, 0.34)
                    if self.is_obstacle_left == True and self.is_obstacle_right==True: # canot turn left or right
                        self.command = set_velocities(-0.08, 0, 0, 0, 0, 0.23)
                    self.cmd_vel.publish(self.command)
            # when detect blue
            elif self.isBlue == True:
                if abs(self.error_x) > 5:
                    self.control = self.pid_controller.update(self.error_x, t)
                    self.command = set_velocities(0.32, 0, 0, 0, 0, -self.control)
                    self.cmd_vel.publish(self.command)
                if self.is_obstacle_front == False:
                    self.command = go_straight(0.45)
                    self.cmd_vel.publish(self.command)
                else:
                    if  self.is_obstacle_left == False and self.is_obstacle_right==True: # turn left
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, 0.34)
                    if self.is_obstacle_left == True and self.is_obstacle_right == False: # turn right
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, -0.34)
                    if self.is_obstacle_left == False and self.is_obstacle_right == False: # can turn or right => turn left
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, 0.34)
                    if self.is_obstacle_left == True and self.is_obstacle_right==True: # canot turn left or right
                        self.command = set_velocities(-0.08, 0, 0, 0, 0, 0.23)
                    self.cmd_vel.publish(self.command)


            # elif self.isRed == True:
            #     if self.error_x > 100:
            #         if self.sonar_data[0] > 0.7:
            #             self.command = go_straight(0.45)
            #         else:
            #             if self.sonar_data[1] > 0.7:
            #                 self.command = set_velocities(-0.1, 0, 0, 0, 0, 0.25)
            #             elif self.sonar_data[3] > 0.7:
            #                 # xoay phai
            #                 self.command = set_velocities(0, 0, 0, 0, 0, -0.25)
            #             else:
            #                 # lui lai
            #                 self.command = set_velocities(-0.5, 0, 0, 0, 0, 0.25)
            #                 self.cmd_vel.publish(self.command)
            self.rate.sleep()

def main(args=None):
    rospy.init_node("example_node")

    exp = Example()
    exp.spin()


if __name__ == "__main__":
    main()



