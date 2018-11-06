#!/usr/bin/env python
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

from nav_msgs.msg import Odometry
import tf.transformations as t

import subprocess
import time
import math

import wave 
import numpy as np

import sys
sys.path.insert(0, '../data_processing_utilities')
import lin_regression

class RotateToSnap(object):
    def __init__(self):
        rospy.init_node("data_generator")    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rotate_speed = .25 
        self.recorded_audio_filename = "audioTest"

        self.settings = termios.tcgetattr(sys.stdin)
        self.valid_keys = ['r']

        self.lin = lin_regression.LinearRegressionLearning()

    def executeAudioRecording(self):
        """ Execute the bash file that will record audio"""
        bash_command = "../data/collect.sh"
        angle_string = "audioTest"
        subprocess.call([bash_command, angle_string])

    def predict_angle(self):
        wav_filename = "recording_" + self.recorded_audio_filename + ".wav"
        # channel_0, channel_1 = self.lin.extract_channel_data('data/recordings2/{}'.format(wav_filename))
        channel_0, channel_1 = self.lin.extract_channel_data('../data_processing_utilities/data/recordings2/recording_audio0_0.wav')
        inputs = [[self.lin.calculate_offset(channel_0, channel_1)], [(np.average(channel_0) + np.average(channel_1))/2]]
        predicted_angle = np.dot(inputs, self.lin.weights)
        print predicted_angle
        return predicted_angle

    def rotate(self, angle):
        """ Rotate the neato for an amount of time to turn to the specified angle """


    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = t.euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """ Main function to record audio and rotate neato towards it """
        while not rospy.is_shutdown():
            key = self.getKey()
            if key and key in self.valid_keys:
                # self.executeAudioRecording()
                # time.sleep(3.5) # Pause for the amount of time recording takes
                predicted_angle = self.predict_angle()
                # self.rotate(predicted_angle)
            else:
                self.vel_pub.publish(Twist())
                if key == '\x03':
                    break

if __name__ == '__main__':
    RotateToSnap().run()