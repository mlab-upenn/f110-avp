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
import avp_utils.avp_utils

# global parameters
initial_car_position = np.array([[1.05668068, -0.42237449, 0.03715369], [1.06900644, 0.42561084, 0.04487766]])
ip_car_1 = "tcp://192.168.1.2:5559"

class OccupancyGrid():
    def __init__(self, dim1, dim2):
        self.matrix = np.zeros((dim1, dim2))
        self.image = np.ones((dim1, dim2, 3), dtype=np.uint8) * 30

    def update_image(self):
        matrix = self.matrix
        self.image[np.where(matrix == 0)] = [34, 40, 49] # background
        self.image[np.where(matrix == 1)] = [242, 163, 101] # obstacle
        self.image[np.where(matrix == 2)] = [136, 176, 75] # vehicle
        self.image[np.where(matrix == 11)] = [236, 236, 236] # box 1
        self.image[np.where(matrix == 12)] = [72, 146, 219] # box 2
        

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
    socket_odom_car_1 = context_odom.socket(zmq.SUB)
    socket_odom_car_1.setsockopt(zmq.SUBSCRIBE, b"")
    socket_odom_car_1.setsockopt(zmq.RCVHWM, 1)
    print("Collecting odom info from car 1...")
    socket_odom_car_1.connect(ip_car_1)

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
    object_matrix_copy = object_matrix.copy()
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
    matching_row = np.zeros((5, 12))
    matching_distance_threshold = 0.07
    

    mu_now = np.zeros((4, ))
    mu_pre = np.zeros((4, ))
    z_now = np.zeros((4, ))
    z_pre = np.zeros((4, ))
    odom_now = np.zeros((4, ))
    odom_pre = np.zeros((4, ))
    
    while True:
        occupancy_grid = OccupancyGrid( dim_x, dim_y )
        # get new point cloud
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
                object_matrix_copy = object_matrix.copy()

        # get new bounding box
        if socket_result.poll(timeout = 1) != 0:
            inference_in = recv_array(socket_result).copy()
            first_time = 0
            # clean the matching row 
            for row_ind in range(matching_row.shape[0]):
                if matching_row[row_ind, 0] > 0:
                    if inference_in[1, 1, 0] - matching_row[row_ind, 7] > 3:
                        matching_row[row_ind, :] = np.zeros((12))

            num_dt = inference_in[0, 0, 0]
            match_inds = []
            matched_row_ind = []
            # looking for matches in the matching_row
            for dt_ind in range(int(num_dt)):
                if dt_ind >= matching_row.shape[0]:
                    break
                distances = np.sqrt((matching_row[:, 1] - inference_in[dt_ind+1, 0, 0]) ** 2 + (matching_row[:, 2] - inference_in[dt_ind+1, 0, 1]) ** 2)
                matches = np.argsort(distances)
                if distances[matches[0]] >= matching_distance_threshold:
                    for row_ind in range(matching_row.shape[0]):
                        if matching_row[row_ind, 0] == 0:
                            # trying to determine which car is this
                            cali_distances = np.sqrt((inference_in[dt_ind+1, 0, 0] - initial_car_position[:, 0]) ** 2 + (inference_in[dt_ind+1, 0, 1] - initial_car_position[:, 1]) ** 2)
                            cali_matches = np.argsort(cali_distances)
                            if distances[cali_matches[0]] >= matching_distance_threshold:
                                matching_row[row_ind, 0] = cali_matches[0]+1 # indicator, 9 means it's unknown
                            else:
                                matching_row[row_ind, 0] = 9 # indicator, 9 means it's unknown

                            matching_row[row_ind, 1] = inference_in[dt_ind+1, 0, 0] # x
                            matching_row[row_ind, 2] = inference_in[dt_ind+1, 0, 1] # y
                            matching_row[row_ind, 3] = inference_in[dt_ind+1, 0, 2] # z
                            matching_row[row_ind, 4] = 0.28 # width
                            matching_row[row_ind, 5] = 0.57 # length
                            matching_row[row_ind, 6] = inference_in[dt_ind+1, 0, 5] # height
                            matching_row[row_ind, 7] = inference_in[dt_ind+1, 1, 0] # time
                            matching_row[row_ind, 8] = inference_in[dt_ind+1, 0, 6] # theta
                            matching_row[row_ind, 9] = inference_in[dt_ind+1, 1, 1] # score
                            # matching_row[row_ind, 10]  detection_count
                            # matching_row[row_ind, 11]  flip_count
                            match_inds.append([dt_ind, row_ind])
                            matched_row_ind.append(row_ind)
                            break
                else:
                    row_ind = matches[0]
                    if not np.isin(row_ind, matched_row_ind):
                        match_inds.append([dt_ind, row_ind])
                        matched_row_ind.append(row_ind)
            match_inds = np.asarray(match_inds)
            # print(match_inds)
            
            # create and send bbox info
            bbox_array = np.zeros((match_inds.shape[0], 10, 3))
            for count, inds in enumerate(match_inds):
                dt_ind = inds[0]
                row_ind = inds[1]
                if matching_row[row_ind, 10] == calibration_count:
                    print('Calibrated')
                    matching_row[row_ind, 10] += 1
                    print('Position:', matching_row[row_ind, 1:4])
                if matching_row[row_ind, 10] < calibration_count and inference_in[dt_ind+1, 1, 1] > 0.6:
                    matching_row[row_ind, 10] += 1
                if matching_row[row_ind, 7] != 0:
                    delta_time = inference_in[dt_ind+1, 1, 0] - matching_row[row_ind, 7]
                    if np.abs(np.pi - np.abs(matching_row[row_ind, 8] - inference_in[dt_ind+1, 0, 6])) < 0.6 and delta_time:
                        if matching_row[row_ind, 10] < calibration_count and inference_in[dt_ind+1, 1, 1] > 0.6:
                            matching_row[row_ind, 11] += 1 # flip_count
                        if not (matching_row[row_ind, 10] == calibration_count and matching_row[row_ind, 11] / matching_row[row_ind, 10] >= 0.5): 
                            inference_in[dt_ind+1, 0, 6] -= np.pi # only make adjustment at the end of calibration
                
                matching_row[row_ind, 1] = inference_in[dt_ind+1, 0, 0] # x
                matching_row[row_ind, 2] = inference_in[dt_ind+1, 0, 1] # y
                matching_row[row_ind, 3] = inference_in[dt_ind+1, 0, 2] # z
                matching_row[row_ind, 6] = inference_in[dt_ind+1, 0, 5] # height
                matching_row[row_ind, 7] = inference_in[dt_ind+1, 1, 0] # time
                matching_row[row_ind, 8] = inference_in[dt_ind+1, 0, 6] # theta
                matching_row[row_ind, 9] = inference_in[dt_ind+1, 1, 1] # score

                # print(matching_row[row_ind, 11])
                
                dt_box_lidar = np.zeros((1, 7))
                dt_box_lidar[0, :3] = matching_row[row_ind, 1:4]
                dt_box_lidar[0, 3:6] = matching_row[row_ind, 4:7]
                dt_box_lidar[0, 6] = matching_row[row_ind, 8]
                dt_boxes_corners = avp_utils.center_to_corner_box3d(
                    dt_box_lidar[:, :3],
                    dt_box_lidar[:, 3:6],
                    dt_box_lidar[:, 6],
                    origin=[0.5, 0.5, 0],
                    axis=2)
                bbox_array[count, 0:8, :] = dt_boxes_corners
                bbox_array[count, 8, :] = matching_row[row_ind, 1:4] # center position
                bbox_array[count, 9, 0] = matching_row[row_ind, 8] # orientation
                bbox_array[count, 9, 1] = matching_row[row_ind, 0] # calibration
            send_array(socket_box, bbox_array)

            
            # inference_in = inference_in_0[1]
            # first_time = 0
            # dt_box_lidar = inference_in[0, :].copy()
            # if len(dt_box_lidar.shape) == 1: 
            #     dt_box_lidar = np.expand_dims(dt_box_lidar, axis=0)

            # # dt_box_lidar[:, 3] *= 1.2 # width
            # # dt_box_lidar[:, 4] *= 1.1 # length
            # dt_box_lidar[:, 3] = 0.28 # width
            # dt_box_lidar[:, 4] = 0.57 # length
            
            # # if we encounter a lost of track, recalibrate
            # if inference_in[1, 0] - last_time > 5:
            #     last_orientation = 0
            #     flip_count = 0
            #     detection_count = 1
            #     print('Calibrating...')
            
            # if detection_count == calibration_count:
            #     print('Calibrated')
            #     detection_count += 1
            # elif detection_count < calibration_count and inference_in[1, 1] > 0.6:
            #     # print(flip_count, detection_count)
            #     detection_count += 1
                
            # if last_time != 0:
            #     delta_time = inference_in[1, 0] - last_time
                
            #     # dealing with orientation fluctuation
            #     if np.abs(np.pi - np.abs(last_orientation - dt_box_lidar[:, 6])) < 0.6 and delta_time:
            #         if detection_count < calibration_count and inference_in[1, 1] > 0.6:
            #             flip_count += 1
            #         if not (detection_count == calibration_count and flip_count / detection_count >= 0.5): 
            #             dt_box_lidar[:, 6] -= np.pi

            #     delta_angular_speed = np.abs(last_orientation - dt_box_lidar[:, 6]) / delta_time
            #     delta_linear_speed = np.abs(last_postion - dt_box_lidar[:, :3]) / delta_time
            #     # print('delta_time', delta_time, 'delta_angular_speed', delta_angular_speed, 'delta_linear_speed', delta_linear_speed)

            #     # filter jetter 
            #     if delta_time < 0.2 and (delta_angular_speed > 2 or np.any(delta_linear_speed > 3)) and detection_count > calibration_count:
            #         # print(delta_angular_speed, delta_linear_speed)
            #         print('filtered', filter_count)
            #         if filter_flag == 0:
            #             first_filter_time = inference_in[1, 0]
            #         filter_flag = 1
                    
            # if filter_flag == 0:
            #     # record current detection
            #     last_orientation = dt_box_lidar[:, 6]
            #     last_time = inference_in[1, 0]
            #     last_postion = dt_box_lidar[:, :3]
            #     filter_count = 0
            # elif filter_count < 5:
            #     dt_box_lidar[:, 6] = last_orientation
            #     last_time = inference_in[1, 0]
            #     dt_box_lidar[:, :3] = last_postion
            #     filter_flag = 0
            #     filter_count += 1
            # elif filter_count >= 5:
            #     last_orientation = dt_box_lidar[:, 6]
            #     last_time = inference_in[1, 0]
            #     last_postion = dt_box_lidar[:, :3]
            #     filter_count = 0
            #     filter_flag = 0 
            #     last_orientation = 0
            #     flip_count = 0
            #     detection_count = 1
            #     print('Calibrating...')

            # z_now[0] = dt_box_lidar[:, 0] # x
            # z_now[1] = dt_box_lidar[:, 1] # y
            # z_now[2] = dt_box_lidar[:, 6] # theta
            # z_now[3] = inference_in[1, 0]
        
        # get new odom from car 1
        # for row_ind in range(matching_row.shape[0]):
        #     if matching_row[row_ind, 0] == 1 and socket_odom_car_1.poll(timeout = 1) != 0 and matching_row[row_ind, 10] > calibration_count:
        #         odom_info = recv_array(socket_odom_car_1)

        #         odom_now[3] = odom_info[2]
        #         linear_speed_correction = 0.7
        #         angular_speed_correction = 0.7
        #         delta_time_odom = 0.04
        #         short_time = 0

        #         if z_now[2] != 0:
        #             delta_time_odom += short_time
        #             odom_now[0:3] = convert_odom(odom_info[0] * linear_speed_correction, odom_info[1] * angular_speed_correction, mu_pre[2])
        #         break
            
        #     # predict step
        #     if odom_now[3] > odom_pre[3]:
        #         # if delta_time_odom > 1000:
        #             # delta_time_odom = 0.04
        #         # print(odom_now[0:3])
        #         mu_now[0:3] = delta_time_odom * odom_now[0:3] + mu_pre[0:3]
        #     odom_pre = odom_now.copy()
        
        # # update step
        # if z_now[3] > z_pre[3]:
        #     K = 0.8
        #     mu_now[0:3] = mu_now[0:3] + K * (z_now[0:3] - mu_now[0:3])
        #     z_pre = z_now.copy()

        # mu_pre = mu_now.copy()
        # print(mu_now)

        # occupancy grid
        if first_time == 0 and pc_grid.shape[0] > 1:
            car_matrix = np.zeros([dim_x_object, dim_y_object])
            for count, inds in enumerate(match_inds):
                # add car detection in the grid
                rect_x = np.zeros((4, ))
                rect_y = np.zeros((4, ))
                rect_x[0] = find_coords(bbox_array[count, 0, 0], object_res)
                rect_y[0] = find_coords(bbox_array[count, 0, 1], object_res, dim_y_object/2)
                rect_x[1] = find_coords(bbox_array[count, 4, 0], object_res)
                rect_y[1] = find_coords(bbox_array[count, 4, 1], object_res, dim_y_object/2)
                rect_x[2] = find_coords(bbox_array[count, 6, 0], object_res)
                rect_y[2] = find_coords(bbox_array[count, 6, 1], object_res, dim_y_object/2)
                rect_x[3] = find_coords(bbox_array[count, 2, 0], object_res)
                rect_y[3] = find_coords(bbox_array[count, 2, 1], object_res, dim_y_object/2)
                car_coords_x, car_coords_y = np.array(polygon(rect_x, rect_y, shape = (dim_x_object, dim_y_object)))
                car_matrix[car_coords_x, car_coords_y] = 1

                rect_x[0] = find_coords(bbox_array[count, 0, 0], grid_res)
                rect_y[0] = find_coords(bbox_array[count, 0, 1], grid_res, dim_y/2)
                rect_x[1] = find_coords(bbox_array[count, 4, 0], grid_res)  
                rect_y[1] = find_coords(bbox_array[count, 4, 1], grid_res, dim_y/2)
                rect_x[2] = find_coords(bbox_array[count, 6, 0], grid_res)
                rect_y[2] = find_coords(bbox_array[count, 6, 1], grid_res, dim_y/2)
                rect_x[3] = find_coords(bbox_array[count, 2, 0], grid_res)
                rect_y[3] = find_coords(bbox_array[count, 2, 1], grid_res, dim_y/2)
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
                            object_matrix_copy[pc_grid[ind, 0], pc_grid[ind, 1]] = 0
                
                # use connected body to find the car
                matrix_labels, num = label(object_matrix_copy, connectivity=2, return_num=True)
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
                object_matrix_copy[pc_grid[:, 0], pc_grid[:, 1]] = 0
        
                car_matrix_object_big = rescale(car_matrix_object, grid_res/object_res, anti_aliasing=False)
                occupancy_grid.matrix[np.where(car_matrix_object_big > 0)] = 2
                occupancy_grid.matrix[car_peri_coords_x, car_peri_coords_y] = 10 + matching_row[inds[1], 0] # perimeter line
            
            object_matrix_big = rescale(object_matrix_copy, grid_res/object_res, anti_aliasing=False)
            occupancy_grid.matrix[np.where(object_matrix_big > 0)] = 1
            occupancy_grid.update_image()
            car_matrix_big = rescale(car_matrix, grid_res/object_res, anti_aliasing=False)
            car_matrix_big[np.where(car_matrix_big > 0)] = 1
            # np.savez('car_info', occupancy_grid.matrix, occupancy_grid.image, np.array([find_coords(bbox_array[:, 8, 0], grid_res), find_coords(bbox_array[:, 8, 1], grid_res, dim_y/2)]), np.array(bbox_array[:, 9, 0]), car_matrix_big) 
            send_array(socket_grid, occupancy_grid.image)
        else:
            object_matrix_big = rescale(object_matrix_copy, grid_res/object_res, anti_aliasing=False)
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