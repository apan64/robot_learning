#!/usr/bin/env python
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

from nav_msgs.msg import Odometry

import subprocess
import time
import math

class DataGenerator(object):
    def __init__(self):
        rospy.init_node("data_generator")    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angle = 0
        self.rotate_speed = .5 
        self.rotate_time = 1 # amount of time rotating between recording, used in rotate function

    def rotate(self):
        self.vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -1 * self.rotate_speed)))
        time.sleep(self.rotate_time)
        self.vel_pub(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

    def odometry_callback(self):
        """ Update the angle of the neato """
        transformed_pos = self.convert_pose_to_xy_and_theta(msg.pose.pose) # transform to x,y,yaw
        self.angle = math.degrees(transformed_pos[2])

    def executeBashFile(self):
        """ Execute the bash file that will record audio"""
        bashCommand = "_________________COMMAND TO RUN THE BASH FILE______________________"
        output = subprocess.check_output(['bash','-c', bashCommand])

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = t.euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def run(self):
        """ Main function to rotate neato, record and save audio"""
        while not rospy.is_shutdown():
            rospy.Subscriber('/odom', Odometry, self.odometry_callback)
            self.rotate()
            self.executeBashFile()

if __name__ == '__main__':
    DataGenerator().run()