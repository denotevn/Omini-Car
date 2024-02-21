from geometry_msgs.msg import Twist

def go_straight(speed_x):
    twist = Twist()
    twist.linear.x = speed_x
    return twist

def turn_lef_right(speed_ang_x, speed_x, speed_y):
    twist = Twist()
    twist.linear.x = speed_x
    twist.linear.y = speed_y
    # turn left speed_ang_x < 0
    # turn right speed_ang_x > 0
    twist.angular.z = speed_ang_x
    return twist

def go_back(speed_x):
    twist = Twist()
    twist.linear.x = -0.5
    return twist

def set_velocities(x,y,z,phi,theta,psi):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = phi
    twist.angular.y = theta
    twist.angular.z = psi
    return twist
