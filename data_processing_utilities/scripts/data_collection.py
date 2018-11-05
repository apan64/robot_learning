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
import threading
import pyaudio
import wave 

class DataGenerator(object):
    def __init__(self):
        rospy.init_node("data_generator")    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angle = 0
        self.rotate_speed = .25 
        self.rotate_time = .5 # amount of time rotating between recording, used in rotate function
        self.semaphore = threading.Semaphore(2)
        self.p = pyaudio.PyAudio()  # for playing .wav file
        self.chunk = 1024
        self.thread1_bool = True
        self.thread2_bool = True
        self.thread3_bool1 = False
        self.thread3_bool2 = False

        self.data_count = 100
        self.wav_file_count = 1
        self.wav_file = "../data/snap_audio_1.wav"


    def rotate(self):
        """ Rotate the neato for a set speed and time """
        while self.thread3_bool1 and self.thread3_bool2:
            # set thread3 booleans to false so thread 3 will not run until 1 and 2 have finished again
            self.thread3_bool1 = False 
            self.thread3_bool2 = False

            # Rotate for rotate_time
            self.vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -1 * self.rotate_speed)))  
            print("Rotating") # for debugging purposes
            time.sleep(self.rotate_time)
            print("Done Rotating") # for debugging purposes
            self.vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

            # Update neato's angle
            rospy.Subscriber('/odom', Odometry, self.odometry_callback)

            # thread 1 and 2 can run again
            self.thread1_bool = True
            self.thread2_bool = True

            self.data_count += 1
            self.data_count = self.data_count%400
            self.wav_file_count = self.data_count/100
            if self.wav_file_count == 0:
                self.wav_file = "../data/snap_audio_0.wav"
            elif self.wav_file_count == 1:
                self.wav_file = "../data/snap_audio_1.wav"
            elif self.wav_file_count == 2:
                self.wav_file = "../data/snap_audio_2.wav"
            else:
                self.wav_file = "../data/no_snap_audio.wav"


    def odometry_callback(self, msg):
        """ Update the stored angle of the neato """
        transformed_pos = self.convert_pose_to_xy_and_theta(msg.pose.pose) # transform to x,y,yaw
        self.angle = math.degrees(transformed_pos[2])

    def executeBashFile(self):
        """ Execute the bash file that will record audio"""
        while self.thread2_bool:
            self.thread2_bool = False
            bash_command = "../data/collect.sh"
            angle_string = "audio" + str(self.wav_file_count) + "_" + str(self.angle)

            print(angle_string) # debugging

            subprocess.call([bash_command, angle_string])
            self.thread3_bool2 = True

    def playAudioFile(self):
        """ Plays the .wav file to record """ 
        while self.thread1_bool:
            self.thread1_bool = False
            bash_command = "../data/play_audio.sh"

            # wav_file = "../data/snap_audio_"

            time.sleep(.5)
            subprocess.call([bash_command, self.wav_file])
            self.thread3_bool1 = True

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
            # rospy.Subscriber('/odom', Odometry, self.odometry_callback)
            t1 = threading.Thread(target = self.playAudioFile)
            t2 = threading.Thread(target = self.executeBashFile)
            t1.start()
            t2.start()
            t3 = threading.Thread(target = self.rotate)
            t3.start()

if __name__ == '__main__':
    DataGenerator().run()