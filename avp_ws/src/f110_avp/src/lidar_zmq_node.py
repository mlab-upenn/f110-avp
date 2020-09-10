#!/usr/bin/env python
# lidar_zmq_node.py
# Zirui Zang
# 20200224

from __future__ import print_function
import rospy
import ros_numpy
# import geometry_msgs.msg
import zmq
import numpy as np
from sensor_msgs.msg import PointCloud2#, PointField
# import sensor_msgs.point_cloud2 as pc2
# import time

class lidar_zmq_node:
    def __init__(self):
        lidarscan_topic = '/os_cloud_node/points'

        # adjust the numbers here to center the pc
        self.yaw_rotation = -45/180.0*np.pi
        self.pitch_rotation = 28/180.0*np.pi
        self.roll_rotation = 2.65/180.0*np.pi
        self.x_shift = 0
        self.y_shift = 0
        self.z_shift = 1.6 # mounting height
        self.y_limit = [1, 4]
        self.x_limit = [-1.1, 1.1]
        self.z_limit = [-0.5, 0.3]
        self.count = 0

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, PointCloud2, self.lidar_callback)
        self.pc_pub = rospy.Publisher('pc_transformed', PointCloud2, queue_size = 1)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind("tcp://*:5556")

    def send_array(self, socket, A, flags=0, copy=True, track=False):
        """send a numpy array with metadata"""
        md = dict(
            dtype = str(A.dtype),
            shape = A.shape,
        )
        socket.send_json(md, flags|zmq.SNDMORE)
        return socket.send(A, flags, copy=copy, track=track)

    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def rotation(self, input, m):
        return np.dot(m, input)

    def lidar_callback(self, msg):
        pc = ros_numpy.numpify(msg)
        pc = pc.flatten()
        pc = pc[np.where( (pc['intensity'] > 100) )]

        pc_transformed = np.empty((3, pc['x'].shape[0]))
        pc_transformed[0, :] = pc['x']
        pc_transformed[1, :] = pc['y']
        pc_transformed[2, :] = pc['z']
        m = self.rotation_matrix([0, 0, 1], self.yaw_rotation)
        pc_transformed = self.rotation(pc_transformed, m)
        m = self.rotation_matrix([0, 1, 0], self.pitch_rotation)
        pc_transformed = self.rotation(pc_transformed, m)
        m = self.rotation_matrix([1, 0, 0], self.roll_rotation)
        pc_transformed = self.rotation(pc_transformed, m)
        m = self.rotation_matrix([0, 0, 1], 90/180.0*np.pi)
        pc_transformed = self.rotation(pc_transformed, m)

        pc_transformed[2, :] += self.z_shift
        pc['x'] = pc_transformed[0, :]
        pc['y'] = pc_transformed[1, :]
        pc['z'] = pc_transformed[2, :]
        pc = pc[np.where( (pc['x'] > self.x_limit[0]) & (pc['x'] < self.x_limit[1]) )]
        pc = pc[np.where( (pc['y'] > self.y_limit[0]) & (pc['y'] < self.y_limit[1]) )]
        pc = pc[np.where( (pc['z'] > self.z_limit[0]) & (pc['z'] < self.z_limit[1]) )]

        points_range = [np.max(pc['x']), np.min(pc['x']), np.max(pc['y']), np.min(pc['y']), np.max(pc['z']), np.min(pc['z'])]
        self.count = (self.count + 1) if self.count < 1e4 else 1e3
        self.x_shift = (self.x_shift  * (self.count-1)  + (points_range[0] + points_range[1]) / 2) / self.count
        self.y_shift = (self.y_shift  * (self.count-1)  + (points_range[2] + points_range[3]) / 2) / self.count
        pc['x'] -= self.x_shift
        pc['y'] -= self.y_shift

        del pc_transformed
        pc_transformed = np.empty((4, pc['x'].shape[0]))
        pc_transformed[0, :] = pc['x']
        pc_transformed[1, :] = pc['y']
        pc_transformed[2, :] = pc['z']
        pc_transformed[3, :] = pc['intensity']
        points_range = [np.max(pc_transformed[0, :]), np.max(pc_transformed[1, :]), np.max(pc_transformed[2, :]), np.min(pc_transformed[0, :]), np.min(pc_transformed[1, :]), np.min(pc_transformed[2, :])]
        print('points_range', points_range)

        msg_transformed = ros_numpy.msgify(PointCloud2, pc) 
        msg_transformed.header.frame_id = '/map'
        self.pc_pub.publish(msg_transformed)
        self.send_array(self.socket, pc_transformed)
        
def main():
    rospy.init_node("lidar_zmq_node", anonymous=True)
    node = lidar_zmq_node()
    print('Sending point cloud...')
    rospy.spin()

if __name__ == '__main__':
    main()