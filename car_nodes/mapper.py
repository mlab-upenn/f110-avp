# executor.py
# Zirui Zang
# 20200315

import zmq
import numpy as np
import time
import matplotlib.pyplot as plt

# waypoints = np.loadtxt('waypoints/way2.csv', delimiter=',')
# print(waypoints.shape)
# waypoints[13+37:, 1] -= 0.5
# waypoints = np.delete(waypoints, np.s_[37:(37+13)], axis=0)

# points_range [1.5448766488976016, 1.4993395805358887, 0.36811161866560216, -0.00530358821833099, -1.4998959302902222, -0.02058618658758493]

waypoints = np.zeros((15, 4))
x1 = 1.15
x2 = 1.15
y1 = 0.5
y2 = -1

waypoints[:, 0] = x1
waypoints[:, 1] = np.arange(y1, y2, -(y1-(y2))/waypoints.shape[0])
# waypoints[:, 3] = np.tan((y2-y1)/(x2-x1))
f = open('way.csv','ab')
np.savetxt(f, waypoints, delimiter=",")

# v_x = np.cos(-(waypoints[:, 3]+np.pi/2))
# v_y = np.sin(-(waypoints[:, 3]+np.pi/2))
# plt.quiver(waypoints[:, 0], waypoints[:, 1], v_x, v_y)
# plt.axis('scaled')
# plt.show()


# print(section)
# v_x = np.cos(-(section[:, 3]+np.pi/2))
# v_y = np.sin(-(section[:, 3]+np.pi/2))

# plt.quiver(section[:, 0], section[:, 1], v_x, v_y)
# plt.axis('scaled')
# plt.show()