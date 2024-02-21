import rospy
from geometry_msgs.msg import Twist
import math

class PID:
    '''
    Control robot using PID algorithm
    '''
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
        self.command = 0.0

    def update(self, error, dt):

        if (dt == 0):
            dt = 0.001

        self.integral += error*dt

        if abs(self.integral > 1):
            self.integral = self.integral/abs(self.integral)

        self.derivative = (error - self.prev_error)/dt

        self.prev_error = error

        output_control_x = self.kp* error  # + self.ki * self.integral + self.kd * self.derivative
        if abs(output_control_x) > 2:
            output_control_x  /= (abs(output_control_x))
        
        return output_control_x




