#!/usr/bin/env python
# driver_node
# Zirui Zang
# 20200315
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
import zmq
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import csv

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

class driver_node:
    def __init__(self):        
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

#    def lidar_callback(self, data):
#        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
#        """
#        global prev_ranges
#        global prev_angle

#        # process the laser ranges
#        # ranges = np.array(data.ranges)[180:180+720]
#        ranges = np.array(data.ranges)[270:270+540]
#        ranges = np.where(np.invert(np.isnan(ranges)), ranges, 4)
#        ranges = np.where(ranges < 4, ranges, 4)
#        ranges = (ranges + prev_ranges) / 2 # filter the ranges
#        prev_ranges = ranges

#        # find the bubble and remove the bubble
#        points_x, points_y = self.Range_to_Coordinates(ranges)
#        min_ind = np.argmin(ranges)
#        distances = (points_x - points_x[min_ind]) ** 2 + (points_y - points_y[min_ind]) ** 2
#        bubble_size = 0.5
#        ranges = np.where(distances > bubble_size, ranges, 0)
#        ranges = np.where(ranges > 1, ranges, 0) # ranges closer than a distance is not considered as gap

#        # find the center direction of the gap
#        gap_size = 0
#        max_size = 0
#        max_ind = 0
#        for value_ind, value in enumerate(ranges):
#            if value > 0:
#                gap_size += 1
#            else:
#                gap_size = 0
#            if gap_size > max_size:
#                max_size = gap_size
#                max_ind = value_ind
#        aim_angle = np.pi/2/540 * (max_ind - int(max_size / 2)) - np.pi/4

#        # limit output steering angle
#        steering_limit = 0.4
#        if aim_angle > steering_limit:
#            aim_angle = steering_limit
#        elif aim_angle < -steering_limit:
#            aim_angle = -steering_limit
        
#        # filter the output steering angle
#        if abs(prev_angle - aim_angle) < 0.1:
#            aim_angle = prev_angle
#        aim_angle = (prev_angle + aim_angle) / 2
#        prev_angle = aim_angle
        
#        # set suitable speed
#        nominal_velosity = 1
#        velocity = nominal_velosity - np.abs(aim_angle) * ((nominal_velosity - 1) / 0.4)
#        drive_msg = AckermannDriveStamped()
#        drive_msg.header.stamp = rospy.Time.now()
#        drive_msg.header.frame_id = "ouster_os1"
#        drive_msg.drive.steering_angle = aim_angle
#        drive_msg.drive.speed = velocity
#        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("driver_node", anonymous=True)
    node = driver_node()
    angle_factor = 0.02

    context_box = zmq.Context()
    socket_box = context_box.socket(zmq.SUB)
    socket_box.setsockopt(zmq.SUBSCRIBE, b"")
    socket_box.setsockopt(zmq.RCVHWM, 1)
    socket_box.connect("tcp://192.168.1.5:5557")
    print("Collecting bboxs...")

    waypoints = np.loadtxt('way.csv', delimiter=",")
    waypoints[11+37:, 1] -= 0.45
    waypoints = np.delete(waypoints, np.s_[37:(37+11)], axis=0)
    waypoints_1 = waypoints[5:37, :]
    waypoints_2 = waypoints[37:, :]
    section2 = 0
#    goal = np.array([ 0.7045296,   0.06313692,  0.        ])
    goal = np.array([ 0.7045296,   0.06013692,  0.        ])
    goal1 = np.array([ 0.82,   -0.32,  0.        ])
    goal2 = np.array([ 0.55,   -0.02,  0.        ])
    while not rospy.is_shutdown():
        if socket_box.poll(timeout = 1) != 0:
            bbox = recv_array(socket_box)
            pose = np.zeros((1, 3))
            pose[0, 0:2] = bbox[8, 0:2]
            current_theta = -(bbox[9, 0] + np.pi/2)
            distances = np.sqrt(np.power(waypoints_1[:, 0] - pose[0, 0], 2) + np.power(waypoints_1[:, 1] - pose[0, 1], 2))
            point_now_ind = np.argmin(distances)
#            print(point_now_ind, waypoints_1.shape[0], section2)
#            print(pose)
            if point_now_ind < waypoints_1.shape[0]-1 and section2 == 0:
                point_now_ind += 1
                waypoint_x = waypoints_1[point_now_ind, 0]
                waypoint_y = waypoints_1[point_now_ind, 1]
                rot_waypoint_x = (waypoint_x - pose[0, 0]) * np.cos(-current_theta) - (waypoint_y - pose[0, 1]) * np.sin(-current_theta)
                rot_waypoint_y = (waypoint_x - pose[0, 0]) * np.sin(-current_theta) + (waypoint_y - pose[0, 1]) * np.cos(-current_theta)
                steering_angle = angle_factor * (2 * rot_waypoint_y) / (rot_waypoint_x ** 2 + rot_waypoint_y ** 2)
                steering_angle = np.min(steering_angle, 0.4189)
                steering_angle = np.max(steering_angle, -0.4189)

                nominal_speed = 0.6
                angle_speed = 0.5
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = 'ouster_os1'
                drive_msg.drive.speed = nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189
                node.drive_pub.publish(drive_msg)
#             elif point_now_ind >= waypoints_1.shape[0]-2 and section2 == 0:
# #                waypoints_1 = waypoints_2
#                 section2 = 1
#                point_now_ind = 0
            # elif section2 == 1:
            #     distance = np.sqrt(np.power(goal1[0] - pose[0, 0], 2) + np.power(goal1[1] - pose[0, 1], 2))
            #     print(distance)
            #     nominal_speed = 0.5
            #     angle_speed = 0.3
            #     if distance < 0.085:
            #         section2 = 2
            #     else:
            #         drive_msg = AckermannDriveStamped()
            #         drive_msg.drive.steering_angle = -0.4189
            #         drive_msg.header.stamp = rospy.Time.now()
            #         drive_msg.header.frame_id = 'ouster_os1'
            #         drive_msg.drive.speed = -(nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189)
            #         node.drive_pub.publish(drive_msg)
            # elif section2 == 2:
            #     distance = np.sqrt(np.power(goal1[0] - pose[0, 0], 2) + np.power(goal2[1] - pose[0, 1], 2))
            #     print(distance)
            #     nominal_speed = 0.5
            #     angle_speed = 0.3
            #     if distance < 0.085:
            #         section2 = 3
            #     else:
            #         drive_msg = AckermannDriveStamped()
            #         drive_msg.drive.steering_angle = 0.4189
            #         drive_msg.header.stamp = rospy.Time.now()
            #         drive_msg.header.frame_id = 'ouster_os1'
            #         drive_msg.drive.speed = -(nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189)
            #         node.drive_pub.publish(drive_msg)
#                point_now_ind += 1
#                waypoint_x = waypoints_1[point_now_ind, 0]
#                waypoint_y = waypoints_1[point_now_ind, 1]
#                rot_waypoint_x = (waypoint_x - pose[0, 0]) * np.cos(-current_theta) - (waypoint_y - pose[0, 1]) * np.sin(-current_theta)
#                rot_waypoint_y = (waypoint_x - pose[0, 0]) * np.sin(-current_theta) + (waypoint_y - pose[0, 1]) * np.cos(-current_theta)
#                steering_angle = angle_factor * (2 * rot_waypoint_y) / (rot_waypoint_x ** 2 + rot_waypoint_y ** 2)
#                steering_angle = np.min(steering_angle, 0.4189)
#                steering_angle = np.max(steering_angle, -0.4189)

#                nominal_speed = 0.6
#                angle_speed = 0.3
#                drive_msg = AckermannDriveStamped()
#                drive_msg.drive.steering_angle = steering_angle
#                drive_msg.header.stamp = rospy.Time.now()
#                drive_msg.header.frame_id = 'ouster_os1'
#                drive_msg.drive.speed = -(nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189)
#                node.drive_pub.publish(drive_msg)
#            elif point_now_ind >= waypoints_1.shape[0]-1 and section2 == 1:
#                distance = np.sqrt(np.power(goal[0] - pose[0, 0], 2) + np.power(goal[1] - pose[0, 1], 2))
#                print(distance)
#                nominal_speed = 0.5
#                angle_speed = 0.3
#                if distance < 0.085:
#                    section2 = 2
#                else:
#                    drive_msg = AckermannDriveStamped()
#                    drive_msg.drive.steering_angle = 0.4189
#                    drive_msg.header.stamp = rospy.Time.now()
#                    drive_msg.header.frame_id = 'ouster_os1'
#                    drive_msg.drive.speed = -(nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189)
#                    node.drive_pub.publish(drive_msg)
            else:
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = 0
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = 'ouster_os1'
                drive_msg.drive.speed = 0
                node.drive_pub.publish(drive_msg)





if __name__ == '__main__':
    main(sys.argv)
