# F110-AVP
F110 Autonomout Valet Parking with Ouster LiDAR

## Installation
1. Clone this repo.
2. Please refer to installation in [PointPillars](https://github.com/nutonomy/second.pytorch) repo to install dependencies for PointPillars. Our repo contains needed code from PointPillars.
3. Install [SparseConvNet](https://github.com/facebookresearch/SparseConvNet) and [ros_numpy](https://github.com/eric-wieser/ros_numpy).
4. Install [ZeroMQ](http://wiki.zeromq.org/intro:get-the-software).
5. Be sure to add second.pytorch/ to your PYTHONPATH.

## To Run the Experiment

### on Host
1. Connect the lidar to LAN and check host ip address. To run Ouster ROS node:
```
cd avp_ws
source devel/setup.bash
roslaunch ouster_ros ouster.launch sensor_hostname:=os1-992006000706.local udp_dest:=[Host IP Address] lidar_mode:=2048x10 viz:=false
```

### on Vehicle

## Result
