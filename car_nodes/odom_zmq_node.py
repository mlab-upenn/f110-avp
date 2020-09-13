#!/usr/bin/env python
# odom_zmq_node.py
# Zirui Zang
# 20200310

from __future__ import print_function
import rospy
import zmq
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import csv
#import pandas as pd

def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    A = np.frombuffer(msg, dtype=md['dtype'])
    return A.reshape(md['shape'])

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)



class odom_zmq_node:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback)

        context_odom = zmq.Context()
        self.socket_odom = context_odom.socket(zmq.PUB)
        self.socket_odom.setsockopt(zmq.SNDHWM, 1)
        self.socket_odom.bind("tcp://*:5559")
        print('Sending odom info...')

        self.odom_time = 0

    def odom_callback(self, msg):
        odom_info = np.zeros((3, ))
#        print(msg.twist.twist.linear.x)
#        print(msg.twist.twist.angular.z)
        now = rospy.get_rostime()
        odom_info[0] = msg.twist.twist.linear.x
        odom_info[1] = msg.twist.twist.angular.z
        odom_info[2] = now.secs + (now.nsecs)/1e9
        diff_time = odom_info[2] - self.odom_time
        self.odom_time = odom_info[2]

        if diff_time < 1 and diff_time > 0.02 and (np.abs(odom_info[0]) > 0.0001 or np.abs(odom_info[1]) > 0.0001):
            send_array(self.socket_odom, odom_info)

        
def main():
    rospy.init_node("odom_zmq_node", anonymous=True)
    node = odom_zmq_node()
    rospy.spin()

if __name__ == '__main__':
    main()
