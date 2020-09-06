#!/usr/bin/env python
# visualization_node_m.py
# Zirui Zang
# 20200420

from __future__ import print_function
import rospy
import ros_numpy
import zmq
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

class visualization_node:
    def __init__(self):
        self.bbox_pub = rospy.Publisher('car_bbox', Marker, queue_size = 1)
        self.arrow_pub = rospy.Publisher('car_arrow', Marker, queue_size = 1)
        self.grid_image_pub = rospy.Publisher('grid_image', Image, queue_size = 1)
        self.waypoints_pub = rospy.Publisher('waypoints', Marker, queue_size = 1)


def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = memoryview(msg)
    A = np.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])

def add_line(line_msg, ind1, ind2, bbox):
    p1 = Point()
    p1.x = bbox[ind1, 0]
    p1.y = bbox[ind1, 1]
    p1.z = bbox[ind1, 2]
    p2 = Point()
    p2.x = bbox[ind2, 0]
    p2.y = bbox[ind2, 1]
    p2.z = bbox[ind2, 2]
    line_msg.points.append(p1)
    line_msg.points.append(p2)
    return line_msg

def make_box_msg(bbox, id):
    line_msg = Marker()
    line_msg.type = line_msg.LINE_LIST
    line_msg.action = line_msg.ADD
    line_msg.scale.x = 0.01
    line_msg.scale.y = 0.1
    line_msg.scale.z = 0.1
    line_msg.color.a = 1
    if id == 1:
        line_msg.color.r = 1
        line_msg.color.g = 1
        line_msg.color.b = 1
    if id == 2:
        line_msg.color.r = 72/255
        line_msg.color.g = 146/255
        line_msg.color.b = 219/255
    line_msg.id = id
    line_msg.header.frame_id = 'os1_lidar'
    line_msg.lifetime = rospy.Duration(0.5)

    line_msg = add_line(line_msg, 0, 1, bbox)
    line_msg = add_line(line_msg, 1, 2, bbox)
    line_msg = add_line(line_msg, 2, 3, bbox)
    line_msg = add_line(line_msg, 3, 0, bbox)

    line_msg = add_line(line_msg, 4, 5, bbox)
    line_msg = add_line(line_msg, 5, 6, bbox)
    line_msg = add_line(line_msg, 6, 7, bbox)
    line_msg = add_line(line_msg, 7, 4, bbox)
    
    line_msg = add_line(line_msg, 0, 4, bbox)
    line_msg = add_line(line_msg, 1, 5, bbox)
    line_msg = add_line(line_msg, 2, 6, bbox)
    line_msg = add_line(line_msg, 3, 7, bbox)
    return line_msg

def make_arrow(bbox, id):
    arrow_msg = Marker()
    arrow_msg.type = arrow_msg.ARROW
    arrow_msg.action = arrow_msg.ADD
    arrow_msg.scale.x = 0.4
    arrow_msg.scale.y = 0.02
    arrow_msg.scale.z = 0.02
    arrow_msg.color.a = 1
    if id == 1:
        arrow_msg.color.r = 1
        arrow_msg.color.g = 1
        arrow_msg.color.b = 1
    if id == 2:
        arrow_msg.color.r = 122/255
        arrow_msg.color.g = 189/255
        arrow_msg.color.b = 255/255
    arrow_msg.id = id
    arrow_msg.header.frame_id = 'os1_lidar'
    arrow_msg.lifetime = rospy.Duration(0.5)

    arrow_msg.pose.position.x = bbox[0, 0]
    arrow_msg.pose.position.y = bbox[0, 1]
    arrow_msg.pose.position.z = bbox[0, 2] + 0.2
    yaw = bbox[1, 0] + np.pi/2
    pitch = 0
    roll = 0
    cy = np.cos(-yaw * 0.5)
    sy = np.sin(-yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    arrow_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
    arrow_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
    arrow_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
    arrow_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
    return arrow_msg

def make_arrow_waypoint(waypoints, id):
    arrow_msg = Marker()
    arrow_msg.type = arrow_msg.ARROW
    arrow_msg.action = arrow_msg.ADD
    arrow_msg.scale.x = 0.08
    arrow_msg.scale.y = 0.02
    arrow_msg.scale.z = 0.02
    arrow_msg.color.a = 1
    arrow_msg.color.r = 249/255
    arrow_msg.color.g = 166/255
    arrow_msg.color.b = 2/255
    arrow_msg.id = id
    arrow_msg.header.frame_id = 'os1_lidar'
    arrow_msg.lifetime = rospy.Duration(2)


    arrow_msg.pose.position.x = waypoints[0]
    arrow_msg.pose.position.y = waypoints[1]
    arrow_msg.pose.position.z = waypoints[2] + 0.2
    yaw = (waypoints[3] + np.pi/2)
    pitch = 0
    roll = 0
    cy = np.cos(-yaw * 0.5)
    sy = np.sin(-yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    arrow_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
    arrow_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
    arrow_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
    arrow_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
    return arrow_msg

def main():
    rospy.init_node("visualization_node", anonymous=True)
    node = visualization_node()

    context_box = zmq.Context()
    socket_box = context_box.socket(zmq.SUB)
    socket_box.setsockopt(zmq.SUBSCRIBE, b"")
    socket_box.setsockopt(zmq.RCVHWM, 1)
    print("Collecting bboxs...")
    socket_box.connect("tcp://localhost:5557")

    context_grid = zmq.Context()
    socket_grid = context_grid.socket(zmq.SUB)
    socket_grid.setsockopt(zmq.SUBSCRIBE, b"")
    socket_grid.setsockopt(zmq.RCVHWM, 1)
    print("Collecting occupancy grid...")
    socket_grid.connect("tcp://localhost:5558")

    # context_odom = zmq.Context()
    # socket_odom = context_odom.socket(zmq.SUB)
    # socket_odom.setsockopt(zmq.SUBSCRIBE, b"")
    # socket_odom.setsockopt(zmq.RCVHWM, 1)
    # print("Collecting odom info...")
    # socket_odom.connect("tcp://192.168.1.2:5559")

    # f = open('way.csv','ab')
    # waypoints = np.loadtxt('/home/lucerna/MEGAsync/project/AVP/car_nodes/way.csv', delimiter=",")
    
    while not rospy.is_shutdown():
        if socket_box.poll(timeout = 1) != 0:
            bboxs = recv_array(socket_box)
            num_dt = bboxs.shape[0]
            for ind in range(num_dt):
                bbox = bboxs[ind]
                # print(int(bbox[9, 1]))
                line_msg = make_box_msg(bbox[0:8, :], int(bbox[9, 1]))
                node.bbox_pub.publish(line_msg)

                arrow_msg = make_arrow(bbox[8:10, :], int(bbox[9, 1]))
                node.arrow_pub.publish(arrow_msg)

        if socket_grid.poll(timeout = 1) != 0:
            grid_image = recv_array(socket_grid)
            grid_image_msg = ros_numpy.msgify(Image, grid_image, encoding='rgb8')
            node.grid_image_pub.publish(grid_image_msg) 

        # for ind in range(waypoints.shape[0]):
        #     waypoint_msg = make_arrow_waypoint(waypoints[ind, :], ind+1)
        #     node.waypoints_pub.publish(waypoint_msg)

if __name__ == '__main__':
    main()
