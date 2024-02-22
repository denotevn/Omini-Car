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
        self.kp = 0.77 # 0.1234
        self.pid_controller = PID.PID(self.kp, 0.001, 0.005)
        self.center_img = None
        self.center_yellow = None # obstacle wall
        self.isYellow = None # detect wall
        self.rate = rospy.Rate(100) # 100 Hz
        # for wall information
        self.is_obstacle_left = False
        self.is_obstacle_right = False
        self.is_obstacle_front = False
        self.angle_error = None
        self.speed_x = 0.5
        self.last_dir_command = None
        self.obstacle_yellow_is_left = None
        self.obstacle_yellow_is_right = None
        self.is_red_left = None
        self.is_red_right = None
        self.speed_angle_cons = 0.47

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
        self.center_yellow  = self.obj_detection[3]
        
        self.center_img = [frame.shape[1]/2, frame.shape[0]/2]
        # blue
        if len(center_blue) > 0:
            self.isBlue = True
            # print(f"Blue: {self.isBlue}")
            self.error_x = center_blue[0] - self.center_img[0]
            self.error_y = center_blue[1] - self.center_img[1]
            self.angle_error = (self.error_x*1.57)/(720)
        else:
            self.isBlue = False

        # yellow
        if len(self.center_yellow) != 0:
            self.isYellow = True
            # print(f"Yellow wall detected: x:{self.center_yellow[1]}, y: {self.center_yellow[0]}")
            if self.center_yellow[0] < self.center_img[0]:
                self.obstacle_yellow_is_right = False
                self.obstacle_yellow_is_left = True
            else:
                self.obstacle_yellow_is_right = True
                self.obstacle_yellow_is_left = False
        else:
            self.isYellow = False
        # red
        if len(center_red) > 0:
            self.isRed = True
            if center_red[0]  < self.center_img[0]:
                self.is_red_left = True
                self.is_red_right = False
            else:
                self.is_red_right = True
                self.is_red_left = False
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


    def spin(self):

        t0 = rospy.get_time()
        # get infor params from images
        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            '''
                if thay tuong:
                    if vang_ben_trai:
                        if trai_co_vat_can va phai_co_vat_can:
                            quay xe
                        elif phai_co_vat_can:
                            quay trai
                        else:
                            quay phai
                    else:
                        if trai_co_vat_can va phai_co_vat_can:
                            quay xe
                        elif trai_co_vat_can:
                            quay phai
                        else:
                            quay trai
                elif thay_red:
                    if red_ben_trai:
                        quay phai
                    else:
                        quay trai
                elif thay blue:
                    di toi blue        
            '''
            # thay tuong
            if self.is_obstacle_front:
                if self.obstacle_yellow_is_left == True:
                    if self.is_obstacle_left ==True and self.is_obstacle_right == True:
                        self.command = set_velocities(-self.speed_x, 0, 0, 0, 0, 0) # can dieu chinh
                        # self.cmd_vel.publish(self.command) #
                    elif self.is_obstacle_right == True: # TODO: turn left
                        # print("co vat can ben phai")
                        self.command = set_velocities(-0.15, 0, 0, 0, 0, self.speed_angle_cons)
                        # self.cmd_vel.publish(self.command) #
                    else: # turn right
                        # print("Truong hop con lai")
                        self.command = set_velocities(-0.15, 0, 0, 0, 0, -self.speed_angle_cons)
                    self.cmd_vel.publish(self.command) #
                    rospy.sleep(0.06)
                else:
                    if self.is_obstacle_left and self.is_obstacle_right:
                        self.command = set_velocities(-self.speed_x, 0, 0, 0, 0, 0) # can dieu chinh
                    elif self.is_obstacle_left: # turn right
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, -self.speed_angle_cons)
                    else:
                        self.command = set_velocities(-0.1, 0, 0, 0, 0, self.speed_angle_cons)
                    self.cmd_vel.publish(self.command) #
            elif self.isRed == True:
                if  self.isYellow == True:
                    if self.obstacle_yellow_is_left:
                        self.command = set_velocities(-0.15, 0, 0, 0, 0, -0.37)
                    else:
                        self.command = set_velocities(-0.15, 0, 0, 0, 0, 0.37)
                    self.cmd_vel.publish(self.command)
                elif self.is_red_left == True: # turn right
                    self.command = set_velocities(-0.15, 0, 0, 0, 0, -self.speed_angle_cons)
                else: # turn left
                    self.command = set_velocities(-0.2, 0, 0, 0, 0, self.speed_angle_cons)
                self.cmd_vel.publish(self.command)
            elif self.isBlue == True:
                self.control = self.pid_controller.update(self.angle_error, t)
                self.command = set_velocities(self.speed_x, 0, 0, 0, 0, -self.control)
                self.cmd_vel.publish(self.command)
                rospy.sleep(0.1)
                if self.isYellow == True and self.isBlue == True:
                    if self.obstacle_yellow_is_left:
                        self.command = set_velocities(0.2, 0, 0, 0, 0, self.speed_angle_cons)
                    else:
                        self.command = set_velocities(0.2, 0, 0, 0, 0, -self.speed_angle_cons)
                    self.cmd_vel.publish(self.command)
                   

            self.rate.sleep()
            '''
                if thay tuong:
                    if vang_ben_trai:
                        if trai_co_vat_can va phai_co_vat_can:
                            quay xe
                        elif phai_co_vat_can:
                            quay trai
                        else:
                            quay phai
                    else:
                        if trai_co_vat_can va phai_co_vat_can:
                            quay xe
                        elif trai_co_vat_can:
                            quay phai
                        else:
                            quay trai
                elif thay_red:
                    if red_ben_trai:
                        quay phai
                    else:
                        quay trai
                elif thay blue:
                    di toi blue        
            '''
 

def main(args=None):
    rospy.init_node("example_node")

    exp = Example()
    exp.spin()


if __name__ == "__main__":
    main()
