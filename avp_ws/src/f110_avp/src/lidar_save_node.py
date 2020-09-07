#!/usr/bin/env python
# lidar_zmq_node.py
# Zirui Zang
# 20200224

from __future__ import print_function
import rospy
import ros_numpy
import geometry_msgs.msg
import zmq
import numpy as np
from sensor_msgs.msg import PointCloud2#, PointField
import sensor_msgs.point_cloud2 as pc2
import time
# import numba
# from numba import jit

class lidar_save_node:
    def __init__(self):
        # lidarscan_topic = '/os_cloud_node/points'
        lidarscan_topic = 'pc_transformed'
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, PointCloud2, self.lidar_callback)
        # self.count = 0
        # self.points_save = np.zeros((64*1024, 3))

        self.pc_pub = rospy.Publisher('pc_transformed', PointCloud2, queue_size = 1)
        # self.context = zmq.Context()
        # self.socket = self.context.socket(zmq.PUB)
        # self.socket.bind("tcp://*:5556")
        self.ind_list = np.array(range(0, 2048, 1))

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

    def rotation(self, input):
        return np.dot(self.m, input)

    def lidar_callback(self, msg):
        pc = ros_numpy.numpify(msg)
        pc = pc.flatten()
        pc_transformed = np.empty((4, pc['x'].shape[0]))
        pc_transformed[0, :] = pc['x']
        pc_transformed[1, :] = pc['y']
        pc_transformed[2, :] = pc['z']
        pc_transformed[3, :] = pc['intensity']
        print(str(msg.header.stamp))
        np.save(str(msg.header.stamp) + '.npy', pc_transformed)
        rospy.sleep(1)
        
        # points[:, 0] = pc['x'][self.ind_list]
        # points[:, 1] = pc['y'][self.ind_list]
        # points[:, 2] = pc['z'][self.ind_list]
        # points[:, 3] = pc['intensity'][self.ind_list]
        # print('numpify time ', time.time() - time2)
        # self.send_array(self.socket, points)

        # pc = ros_numpy.numpify(msg)
        # points = np.zeros((pc['x'].flatten().shape[0], 4))
        # points[:, 0] = pc['x'].flatten()
        # points[:, 1] = pc['y'].flatten()
        # points[:, 2] = pc['z'].flatten()
        # points[:, 3] = pc['intensity'].flatten()
        # self.send_array(self.socket, points)

        
def main():
    rospy.init_node("lidar_save_node", anonymous=True)
    node = lidar_save_node()
    print('Saving point cloud...')
    rospy.spin()

if __name__ == '__main__':
    main()