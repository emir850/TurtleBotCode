#!/usr/bin/env python3

############ Turtlebot, Obstacle detection and movement ############
"""
This script controls the movement of a Turtlebot robot and performs obstacle avoidance using laser scan data.
Simply, the script implements a PID controller for yaw control and provides functions
for setting the setpoint yaw, calculating the error and output for yaw control.

Team: AIR08
"""

############ Importing Libraries ############
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2
import time
from sensor_msgs.msg import LaserScan
import time
import os, math

# Main Class
class TurtlebotMove():

    def __init__(self):
        rospy.init_node('turtlebotObjectDetection')                # initialize node
        ############ subscribe to ROS topics ############
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        ############ Publish ROS Topics ############
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.init_time      = time.time()           # Save code initializing time
        self.distance       = 1.5                   # Minimum distance to keep from obstacles
        self.detection_cone = 15                    # Obsacle Detection Cone
        self.increment      = 0                     # Obstacle Detection Cone list position
        self.change_turn_angle_flag = True          # Disable moving straight when turning
        self.twist = Twist()                        # Message to save velocity message
        self.turnlock = True                        # Lock a turning side 
        self.obstacle_detected = False
        
        ############ Variables for yaw Control ############
        self.setpoint_yaw = 0.0                     # Desired yaw angle (setpoint)
        self.error_yaw = 0.0                        # Current error in yaw angle
        self.last_error_yaw = 0.0                   # Error in yaw angle at the previous iteration
        self.integral_error_yaw = 0.0               # Accumulated error over time for integral control

        ############ PID Parameters ############
        self.kp = 0.5                               # Proportional gain
        self.ki = 0.0                               # Integral gain
        self.kd = 0.1                               # Derivative gain
    
    ############ PID Functions for Yaw Control ############
    def set_setpoint_yaw(self, setpoint_yaw):       # Function to set the desired yaw angle (setpoint) for the yaw control.
        self.setpoint_yaw = setpoint_yaw

    def get_error_yaw(self):                        # Function to get the current error in yaw angle.
        return self.error_yaw

    def get_integral_error_yaw(self):               # Function to get the accumulated error over time for integral control.
        return self.integral_error_yaw

    def get_derivative_error_yaw(self):             # Function to get the derivative of the error in yaw angle.
        return self.error_yaw - self.last_error_yaw

    def calculate_output_yaw(self, setpoint_yaw):   # Function to calculate the output for yaw control.
        self.error_yaw = self.setpoint_yaw - self.current_yaw
        if self.current_yaw > self.setpoint_yaw:
            self.error_yaw = self.current_yaw - self.setpoint_yaw
        self.integral_error_yaw += self.error_yaw
        output = self.kp * self.error_yaw + self.ki * self.integral_error_yaw + self.kd * self.get_derivative_error_yaw()
        self.last_error_yaw = self.error_yaw
        return output
    
    def scan_callback(self, msg):
        """
            Subscribe to LaserScan data and based on the data navigate the robot in the arena.
        """
        front_ranges1 = msg.ranges[0:self.detection_cone]       # Data from 0 - (+detection_cone)
        front_ranges2 = msg.ranges[-self.detection_cone:-1]     # Data from (-detection_cone) - 0
        self.change_turn_angle_flag = True
        twist = Twist()
        if not self.obstacle_detected:
            rospy.loginfo('Going Straight')
            twist.linear.x = 0.4
            twist.angular.z = 0.0
            self.pub.publish(twist)
            for obstacle in front_ranges1:
                if obstacle < self.distance:
                    self.obstacle_detected = True
                    self.init_angle = True
                    break
            for obstacle in front_ranges2:
                if obstacle < self.distance:
                    self.obstacle_detected = True
                    self.init_angle = True
                    break
        else:
            self.turn(setpoint_yaw=math.pi/2, max_speed=0.4)

    def odom_callback(self, msg):
        """
            This callback subscribe to odometry data of the Turtlebot and convert it into Roll, Pitch and Yaw angles.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        ############ Convert quaternian to Euler angles ############
        q0 = msg.pose.pose.orientation.w                        # Quaternion scalar component
        q1 = msg.pose.pose.orientation.x                        # Quaternion x component
        q2 = msg.pose.pose.orientation.y                        # Quaternion y component
        q3 = msg.pose.pose.orientation.z                        # Quaternion z component

        ysqr = q2 * q2                                          # Calculate the squared value of q2

        sin = 2.0 * (q0 * q3 + q1 * q2)                         # Calculate sine and cosine terms for yaw angle calculation
        cosine = 1.0 - 2.0 * (ysqr + q3 * q3)

        yaw = atan2(sin, cosine)                                # Calculate the yaw angle using the arctangent function

        self.current_yaw = yaw                                  # Set the current yaw angle

        if self.current_yaw<0:                                  # Adjust the yaw angle to be in the range of 0 to 2*pi
            self.current_yaw = 2*math.pi + self.current_yaw

    ############ Yaw Control ############
    def turn(self, setpoint_yaw, max_speed):
        
        twist = Twist()
        if self.init_angle:
            desired_angle = self.current_yaw + setpoint_yaw
            if math.degrees(desired_angle) > 360:               # Wrap the desired angle within the range of 0 to 2*pi
                desired_angle = desired_angle - 2 * math.pi
            self.set_setpoint_yaw(desired_angle)                # Set the desired yaw angle for yaw control
            self.init_angle = False


        output = self.calculate_output_yaw(setpoint_yaw)
        error_yaw = self.get_error_yaw()
        rospy.loginfo(f'Turning| current {math.degrees(self.current_yaw):1f} error {math.degrees(error_yaw):1f} setpoint {math.degrees(self.setpoint_yaw):1f}')
        if abs(error_yaw) < 0.015:
            rospy.loginfo(f'Rotation Completed')
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.obstacle_detected = False
        else:
            output = max(-max_speed, min(output, max_speed))
            twist.linear.x = 0.0
            twist.angular.z = output
            self.pub.publish(twist)

def main():
    controller = TurtlebotMove()
    rospy.spin()

if __name__ == '__main__':
    main()