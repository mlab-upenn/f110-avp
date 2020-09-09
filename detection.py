# zmq_inference.py
# Zirui Zang
# 20200224

import zmq
import io as sysio
import os
import sys
import time
from pathlib import Path
import numpy as np
import second.core.box_np_ops as box_np_ops
from second.pytorch.inference import TorchInferenceContext

w_x_shift = 2
w_y_shift = 1
w_z_shift = -0.01
scale_up = 6.8
detection_thresh = 0.4

def Preprocess(points, scale_up):
    points = np.transpose(points)
    points = points.copy()

    points[:, 0] += w_x_shift
    points[:, 1] += w_y_shift
    points[:, 2] += w_z_shift
    points = points[np.where( points[:, 2] > 0 )]
    points[:, 3] = 0
    
    if points.shape[0] < 200:
        return None, None

    points_range = [np.max(points[:, 0]), np.max(points[:, 1]), np.max(points[:, 2]), np.min(points[:, 0]), np.min(points[:, 1]), np.min(points[:, 2])]
    # print('points_range', points_range)
    points = np.array(points) * scale_up

    return points, points_range

def BuildVoxelNet():
    config_path = Path('second.pytorch/second/configs/xyres_16.proto')
    ckpt_path = Path('second.pytorch/second/voxelnet-331653.tckpt')
    inference_ctx = TorchInferenceContext()
    inference_ctx.build(config_path)
    inference_ctx.restore(ckpt_path)
    return inference_ctx

def PointPillarsInference(inference_ctx, points, points_range, scale_up):
    inputs = inference_ctx.get_inference_input_dict(points)
    with inference_ctx.ctx():
        predictions_dicts = inference_ctx.inference(inputs)
    # print(predictions_dicts)
    detection_anno = predictions_dicts[0]
    if detection_anno["box3d_lidar"] is None:
        return None
    dt_box_lidar = np.array([detection_anno["box3d_lidar"].detach().cpu().numpy()])[0]
    scores = np.array([detection_anno["scores"].detach().cpu().numpy()])[0]

    # filter by score
    keep_list = np.where(scores > detection_thresh)[0]
    dt_box_lidar = dt_box_lidar[keep_list, :]
    scores = scores[keep_list]
    dt_box_lidar[:, :6] /= scale_up
    dt_box_lidar[:, 0] -= w_x_shift
    dt_box_lidar[:, 1] -= w_y_shift
    dt_box_lidar[:, 2] -= w_z_shift
    points_range[0] -= w_x_shift
    points_range[3] -= w_x_shift
    points_range[1] -= w_y_shift
    points_range[4] -= w_y_shift

    # filter bbox by its center
    centers = dt_box_lidar[:, :3]
    keep_list = np.where((centers[:, 0] < points_range[0]) & (centers[:, 0] > points_range[3]) & \
                            (centers[:, 1] < points_range[1]) & (centers[:, 1] > points_range[4]))[0]

    num_dt = keep_list.shape[0]
    if num_dt == 0:
        print('miss')
        return None
    print('num_dt', num_dt)

    result_array = np.zeros((num_dt+1, 2, 7))
    result_array[0, 0, 0] = num_dt
    
    for ind_dt in range(num_dt):
        result_array[ind_dt+1, 0, :] = dt_box_lidar[keep_list[ind_dt], :]
        result_array[ind_dt+1, 1, 0] = time.time()
        result_array[ind_dt+1, 1, 1] = scores[keep_list[ind_dt]]
        print('scores: {:.8f}'.format(result_array[ind_dt+1, 1, 1]), 'angles: {:.8f}'.format(dt_box_lidar[keep_list[ind_dt], 6]), 'time: {:.4f}'.format(result_array[ind_dt+1, 1, 0]))
    return result_array


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

def main():
    inference_ctx = BuildVoxelNet()

    context = zmq.Context()
    socket_pc = context.socket(zmq.SUB)
    socket_pc.setsockopt(zmq.SUBSCRIBE, b"")
    socket_pc.setsockopt(zmq.RCVHWM, 1)
    socket_pc.connect("tcp://localhost:5556")
    socket_pc.setsockopt(zmq.CONFLATE, 1)
    print("Collecting point clouds...")

    context_result = zmq.Context()
    socket_result = context_result.socket(zmq.PUB)
    socket_result.setsockopt(zmq.SNDHWM, 1)
    socket_result.bind("tcp://*:5560")
    print('Sending inference')

    while True:
        points_raw = recv_array(socket_pc)
        points, points_range = Preprocess(points_raw, scale_up)
        if points is None:
            print('no point cloud received.')
            continue
        result_array = PointPillarsInference(inference_ctx, points, points_range, scale_up)
        if result_array is not None:
            # print(result_array)
            send_array(socket_result, result_array)


if __name__ == '__main__':
    main()
