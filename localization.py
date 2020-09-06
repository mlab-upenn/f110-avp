# occupancy_grid.py
# Zirui Zang
# 20200229

import zmq
import numpy as np
import time
from matplotlib import pyplot as plt
from skimage.transform import rescale
from skimage.draw import polygon, polygon_perimeter
from skimage.measure import label
import second.core.box_np_ops as box_np_ops


class OccupancyGrid():
    def __init__(self, dim1, dim2):
        self.matrix = np.zeros((dim1, dim2))
        self.image = np.ones((dim1, dim2, 3), dtype=np.uint8) * 225

    def update_image(self):
        matrix = self.matrix
        self.image[np.where(matrix == 0)] = [255, 255, 255]
        self.image[np.where(matrix == 1)] = [204, 0, 0]
        self.image[np.where(matrix == 2)] = [0, 125, 255]
        self.image[np.where(matrix == 3)] = [0, 51, 153]

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

def recv_array(socket, flags=0, copy=True, track=False):
    """ZMQ recv a numpy array"""
    md = socket.recv_json(flags=flags) 
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = bytes(memoryview(msg))
    A = np.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])

def find_coords(input, resolution, is_y = 0):
    if is_y == 0:
        return (np.floor(input * resolution))
    else:
        return (np.floor(input * resolution) + is_y)

def find_neighbours(coord_x, coord_y, limit_x, limit_y, option = 0):
    if option == 0:
        neighbors = np.array([[np.min([limit_x-1, coord_x+1]), coord_y], \
                            [np.max([0, coord_x-1]), coord_y], \
                            [coord_x, np.min([limit_y-1, coord_y+1])], \
                            [coord_x, np.max([0, coord_y-1])], \
                            [np.min([limit_x-1, coord_x+1]), np.min([limit_y-1, coord_y+1])], \
                            [np.min([limit_x-1, coord_x+1]), np.max([0, coord_y-1])], \
                            [np.max([0, coord_x-1]), np.min([limit_y-1, coord_y+1])], \
                            [np.max([0, coord_x-1]), np.max([0, coord_y-1])]])
    elif option == 1:
        neighbors = np.array([[np.min([limit_x-1, coord_x+1]), np.min([limit_y-1, coord_y+1])], \
                            [np.min([limit_x-1, coord_x+1]), np.max([0, coord_y-1])], \
                            [np.max([0, coord_x-1]), np.min([limit_y-1, coord_y+1])], \
                            [np.max([0, coord_x-1]), np.max([0, coord_y-1])]])
    return neighbors

def convert_odom(v, omega, theta):
    # print(v, omega, theta)
    result = np.zeros((3,))
    result[0] = -np.sin(theta) * v
    result[1] = -np.cos(theta) * v
    result[2] = -omega
    return result

def main():
    np.set_printoptions(threshold=np.inf)
    # pc_file = '/home/lucerna/MEGA/project/AVP/2427439726050.npy'
    # pc = np.transpose(np.load(pc_file))

    context_cloud = zmq.Context()
    socket_cloud = context_cloud.socket(zmq.SUB)
    socket_cloud.setsockopt(zmq.SUBSCRIBE, b"")
    socket_cloud.setsockopt(zmq.RCVHWM, 1)
    socket_cloud.connect("tcp://localhost:5556")
    print("Collecting point clouds...")

    context_result = zmq.Context()
    socket_result = context_result.socket(zmq.SUB)
    socket_result.setsockopt(zmq.SUBSCRIBE, b"")
    socket_result.setsockopt(zmq.RCVHWM, 1)
    socket_result.connect("tcp://localhost:5560")
    print("Collecting inference...")

    context_odom = zmq.Context()
    socket_odom = context_odom.socket(zmq.SUB)
    socket_odom.setsockopt(zmq.SUBSCRIBE, b"")
    socket_odom.setsockopt(zmq.RCVHWM, 1)
    print("Collecting odom info...")
    socket_odom.connect("tcp://192.168.1.2:5559")

    context_box = zmq.Context()
    socket_box = context_box.socket(zmq.PUB)
    socket_box.setsockopt(zmq.SNDHWM, 1)
    socket_box.bind("tcp://*:5557")
    print('Sending bbox')

    context_grid = zmq.Context()
    socket_grid = context_grid.socket(zmq.PUB)
    socket_grid.setsockopt(zmq.SNDHWM, 1)
    socket_grid.bind("tcp://*:5558")
    print('Sending occupancy grid')

    x_clip = np.array([0, 1.5])
    y_clip = np.array([-1.5, 1.5])
    z_clip = 0.14
    grid_res = 100
    object_res = 50

    # create occupancy grid
    dim_x = int((x_clip[1] - x_clip[0]) * grid_res)
    dim_y = int((y_clip[1] - y_clip[0]) * grid_res)
    dim_x_object = int((x_clip[1] - x_clip[0]) * object_res)
    dim_y_object = int((y_clip[1] - y_clip[0]) * object_res)
    object_matrix = np.zeros([dim_x_object, dim_y_object])
    object_matrix_inflated = object_matrix.copy()
    car_matrix = np.zeros([dim_x_object, dim_y_object])
    car_matrix_object = np.zeros([dim_x_object, dim_y_object])
    car_peri_coords_x = []
    car_peri_coords_y = []
    pc_grid = np.zeros((1, 2))
    pc_in = None
    inference_in = None
    odom_info = None

    last_orientation = 0
    last_postion = np.zeros((1, 3))
    last_time = 0
    flip_count = 0
    detection_count = 1
    calibration_count = 10
    filter_flag = 0
    first_time = 1
    first_filter_time = 0
    point_update = 0
    filter_count = 0

    mu_now = np.zeros((4, ))
    mu_pre = np.zeros((4, ))
    z_now = np.zeros((4, ))
    z_pre = np.zeros((4, ))
    odom_now = np.zeros((4, ))
    odom_pre = np.zeros((4, ))
    
    while True:
        if socket_cloud.poll(timeout = 5) != 0:
            # add objects in the grid
            point_update = 1
            pc_in = np.transpose(recv_array(socket_cloud))
            pc = pc_in[np.where( (pc_in[:, 1] > y_clip[0]) & (pc_in[:, 1] < y_clip[1]) )]
            pc = pc[np.where( pc[:, 0] < x_clip[1] )]
            pc[:, 2] += -z_clip
            pc = pc[np.where( (pc[:, 2] > 0) )]
            pc = pc[np.where( (pc[:, 3] > 150) )]
            pc_grid = pc[:, 0:2]
            pc_grid[:, 0] = (np.floor(pc_grid[:, 0] * object_res))
            pc_grid[:, 1] = (np.floor(pc_grid[:, 1] * object_res) + dim_y_object/2)
            pc_grid = pc_grid.astype(int)

            if pc_grid.shape[0] > 1:
                object_matrix = np.zeros([dim_x_object, dim_y_object])
                pc_grid, counts = np.unique(pc_grid, return_counts = True, axis = 0)
                pc_grid = pc_grid[np.where(counts > grid_res/object_res)]
                object_matrix[pc_grid[:, 0], pc_grid[:, 1]] = 1
                object_matrix_inflated = object_matrix.copy()

        if socket_result.poll(timeout = 1) != 0:
            inference_in = recv_array(socket_result)
            first_time = 0
            dt_box_lidar = inference_in[0, :].copy()
            if len(dt_box_lidar.shape) == 1: 
                dt_box_lidar = np.expand_dims(dt_box_lidar, axis=0)
            # dt_box_lidar[:, 3] *= 1.2 # width
            # dt_box_lidar[:, 4] *= 1.1 # length
            dt_box_lidar[:, 3] = 0.28 # width
            dt_box_lidar[:, 4] = 0.57 # length
            
            # if we encounter a lost of track, recalibrate
            if inference_in[1, 0] - last_time > 5:
                last_orientation = 0
                flip_count = 0
                detection_count = 1
                print('Calibrating...')
            
            if detection_count == calibration_count:
                print('Calibrated')
                detection_count += 1
            elif detection_count < calibration_count and inference_in[1, 1] > 0.6:
                # print(flip_count, detection_count)
                detection_count += 1
                
            if last_time != 0:
                delta_time = inference_in[1, 0] - last_time
                
                # dealing with orientation fluctuation
                if np.abs(np.pi - np.abs(last_orientation - dt_box_lidar[:, 6])) < 0.6 and delta_time:
                    if detection_count < calibration_count and inference_in[1, 1] > 0.6:
                        flip_count += 1
                    if not (detection_count == calibration_count and flip_count / detection_count >= 0.5): 
                        dt_box_lidar[:, 6] -= np.pi

                delta_angular_speed = np.abs(last_orientation - dt_box_lidar[:, 6]) / delta_time
                delta_linear_speed = np.abs(last_postion - dt_box_lidar[:, :3]) / delta_time
                # print('delta_time', delta_time, 'delta_angular_speed', delta_angular_speed, 'delta_linear_speed', delta_linear_speed)

                # filter jetter 
                if delta_time < 0.2 and (delta_angular_speed > 2 or np.any(delta_linear_speed > 3)) and detection_count > calibration_count:
                    # print(delta_angular_speed, delta_linear_speed)
                    print('filtered', filter_count)
                    if filter_flag == 0:
                        first_filter_time = inference_in[1, 0]
                    filter_flag = 1
                    
            if filter_flag == 0:
                # record current detection
                last_orientation = dt_box_lidar[:, 6]
                last_time = inference_in[1, 0]
                last_postion = dt_box_lidar[:, :3]
                filter_count = 0
            elif filter_count < 5:
                dt_box_lidar[:, 6] = last_orientation
                last_time = inference_in[1, 0]
                dt_box_lidar[:, :3] = last_postion
                filter_flag = 0
                filter_count += 1
            elif filter_count >= 5:
                last_orientation = dt_box_lidar[:, 6]
                last_time = inference_in[1, 0]
                last_postion = dt_box_lidar[:, :3]
                filter_count = 0
                filter_flag = 0 
                last_orientation = 0
                flip_count = 0
                detection_count = 1
                print('Calibrating...')

            z_now[0] = dt_box_lidar[:, 0] # x
            z_now[1] = dt_box_lidar[:, 1] # y
            z_now[2] = dt_box_lidar[:, 6] # theta
            z_now[3] = inference_in[1, 0]
        
        # get odom from car
        if socket_odom.poll(timeout = 1) != 0 and detection_count > calibration_count:
            odom_info = recv_array(socket_odom)
            # print(odom_info)
            odom_now[3] = odom_info[2]
            linear_speed_correction = 0.7
            angular_speed_correction = 0.7
            # delta_time_odom = odom_now[3] - odom_pre[3]
            delta_time_odom = 0.04
            short_time = 0
            # if delta_time_odom < 0.01:
            #     short_time = delta_time_odom
            # if delta_time_odom > 0.01 and z_now[2] != 0:
            if z_now[2] != 0:
                delta_time_odom += short_time
                odom_now[0:3] = convert_odom(odom_info[0] * linear_speed_correction, odom_info[1] * angular_speed_correction, mu_pre[2])
            
            # predict step
            if odom_now[3] > odom_pre[3]:
                # if delta_time_odom > 1000:
                    # delta_time_odom = 0.04
                # print(odom_now[0:3])
                mu_now[0:3] = delta_time_odom * odom_now[0:3] + mu_pre[0:3]
            odom_pre = odom_now.copy()
        
        # update step
        if z_now[3] > z_pre[3]:
            K = 0.8
            mu_now[0:3] = mu_now[0:3] + K * (z_now[0:3] - mu_now[0:3])
            z_pre = z_now.copy()

        mu_pre = mu_now.copy()
        # print(mu_now)
        if first_time == 0 and pc_grid.shape[0] > 1: 
            dt_box_lidar[:, 0:2] = mu_now[0:2]
            dt_box_lidar[:, 6] = mu_now[2]
            # get the cornor coords from the bbox
            dt_boxes_corners = box_np_ops.center_to_corner_box3d(
                dt_box_lidar[:, :3],
                dt_box_lidar[:, 3:6],
                dt_box_lidar[:, 6],
                origin=[0.5, 0.5, 0],
                axis=2)
            bbox_array = np.zeros((10, 3))
            bbox_array[0:8, :] = dt_boxes_corners
            bbox_array[8, :] = dt_box_lidar[0, :3] # center position
            bbox_array[9, 0] = dt_box_lidar[0, 6] # orientation
            send_array(socket_box, bbox_array)

            # add car detection in the grid
            rect_x = np.zeros((4, ))
            rect_y = np.zeros((4, ))
            rect_x[0] = find_coords(bbox_array[0, 0], object_res)
            rect_y[0] = find_coords(bbox_array[0, 1], object_res, dim_y_object/2)
            rect_x[1] = find_coords(bbox_array[4, 0], object_res)
            rect_y[1] = find_coords(bbox_array[4, 1], object_res, dim_y_object/2)
            rect_x[2] = find_coords(bbox_array[6, 0], object_res)
            rect_y[2] = find_coords(bbox_array[6, 1], object_res, dim_y_object/2)
            rect_x[3] = find_coords(bbox_array[2, 0], object_res)
            rect_y[3] = find_coords(bbox_array[2, 1], object_res, dim_y_object/2)
            car_coords_x, car_coords_y = np.array(polygon(rect_x, rect_y, shape = (dim_x_object, dim_y_object)))
            car_matrix = np.zeros([dim_x_object, dim_y_object])
            car_matrix[car_coords_x, car_coords_y] = 1

            rect_x[0] = find_coords(bbox_array[0, 0], grid_res)
            rect_y[0] = find_coords(bbox_array[0, 1], grid_res, dim_y/2)
            rect_x[1] = find_coords(bbox_array[4, 0], grid_res)
            rect_y[1] = find_coords(bbox_array[4, 1], grid_res, dim_y/2)
            rect_x[2] = find_coords(bbox_array[6, 0], grid_res)
            rect_y[2] = find_coords(bbox_array[6, 1], grid_res, dim_y/2)
            rect_x[3] = find_coords(bbox_array[2, 0], grid_res)
            rect_y[3] = find_coords(bbox_array[2, 1], grid_res, dim_y/2)
            car_peri_coords_x, car_peri_coords_y = np.array(polygon_perimeter(rect_x, rect_y, shape = (dim_x, dim_y), clip = True))

            # if an occupied grid's neighbor is in car, then that grid is also in car
            car_matrix_object = np.zeros([dim_x_object, dim_y_object])
            pc_grid = np.transpose(np.array(np.where(object_matrix == 1)))
            for ind in range(pc_grid.shape[0]):
                if car_matrix[pc_grid[ind, 0], pc_grid[ind, 1]] == 1:
                    car_matrix_object[pc_grid[ind, 0], pc_grid[ind, 1]] = 1
                else:
                    find_neighbor = 0
                    neighbors = find_neighbours(pc_grid[ind, 0], pc_grid[ind, 1], dim_x_object, dim_y_object, option = 0)
                    for neighbor in neighbors:
                        if object_matrix[neighbor[0], neighbor[1]] == 1:
                            find_neighbor = 1
                        if car_matrix[neighbor[0], neighbor[1]] == 1:
                            car_matrix_object[pc_grid[ind, 0], pc_grid[ind, 1]] = 1
                            find_neighbor = 1
                            continue
                    if find_neighbor == 0:
                        # remove isolated points
                        object_matrix_inflated[pc_grid[ind, 0], pc_grid[ind, 1]] = 0
            
            # use connected body to find the car
            matrix_labels, num = label(object_matrix_inflated, connectivity=2, return_num=True)
            find_flag = 0
            for num_ind in range(num+1):
                label_xy = np.array(np.where(matrix_labels == num_ind))
                if num_ind != 0:
                    for ind in range(label_xy.shape[1]):
                        if car_matrix_object[label_xy[0, ind], label_xy[1, ind]] == 1:
                            car_matrix_object[label_xy[0, :], label_xy[1, :]] = 1
                            find_flag = 1
                        if find_flag == 1:
                            break
                        # print(label_xy.shape[1], ind)

            pc_grid = np.transpose(np.array(np.where(car_matrix_object == 1)))
            object_matrix_inflated[pc_grid[:, 0], pc_grid[:, 1]] = 0

            # inflate the rest of the object matrix 
            # if point_update == 1:
            #     pc_grid = np.transpose(np.array(np.where(object_matrix_inflated == 1)))
            #     for ind in range(pc_grid.shape[0]):
            #         neighbors = find_neighbours(pc_grid[ind, 0], pc_grid[ind, 1], dim_x_object, dim_y_object, option = 1)
            #         for neighbor in neighbors:
            #             object_matrix_inflated[neighbor[0], neighbor[1]] = 1
            #     pc_grid = np.transpose(np.array(np.where(object_matrix_inflated == 1)))
            #     point_update = 0
        
        car_matrix_object_big = rescale(car_matrix_object, grid_res/object_res, anti_aliasing=False)
        object_matrix_big = rescale(object_matrix_inflated, grid_res/object_res, anti_aliasing=False)

        occupancy_grid = OccupancyGrid( dim_x, dim_y )
        occupancy_grid.matrix[np.where(car_matrix_object_big > 0)] = 2
        occupancy_grid.matrix[car_peri_coords_x, car_peri_coords_y] = 3 # perimeter line
        occupancy_grid.matrix[np.where(object_matrix_big > 0)] = 1
        occupancy_grid.update_image()
        send_array(socket_grid, occupancy_grid.image)

            # fig2 = plt.figure()
            # plt.imshow(matrix_labels)
            # fig1 = plt.figure()
            # plt.imshow(car_matrix_object)
            # fig3 = plt.figure()
            # plt.imshow(map1)
            # plt.show()


if __name__ == '__main__':
    main()