# 1: stack: { [0, 0, 0.72, 0, 0, 0], [0, 0, 0.74, 0, 0, 0],  }
# 2: gantry_0: { [0, 0, 0, 0, 0.1],  }
# 3: gantry_0: { [0, 0, -7.963e-05, 0, 0.12],  }
# 8: blue_0: { [0, 0, 0.72, 0, 0, 0],  }
# 9: blue_1--gantry_0--: { [0.0002456, 0.0799, 0.8064, -0.7493, 0.4996, 0.7507], [0.0001, 0, -7.963e-05, 0, 0.12],  }
# 11: blue_1--gantry_0--: { [-0.09976, -0.0201, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -0.1001, -0.1, 0.1],  }
# 12: blue_1: { [-0.1, -0.1, 0.72, 0, 0, 0],  }
# 13: gantry_0: { [0, 0, -0.1001, -0.1, 0.1],  }
# 17: gantry_0: { [0, 0, -7.963e-05, 0, 0.1],  }
# 19: blue_0--gantry_0--: { [0.0002455, 0.0799, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -7.963e-05, 0, 0.1],  }
# 20: blue_0--gantry_0--: { [0.1249, 0.176, 0.7864, -0.7488, 0.4996, 0.6512], [0.0001, 0.1, 0.09992, 0.09998, 0.1],  }
# 21: blue_0: { [0.1, 0.1, 0.72, 0, 0, 0.1],  }
# 22: gantry_0: { [0, 0.1, 0.09992, 0.09998, 0.1],  }
# 26: gantry_0: { [0, 0, -0.1001, -0.1, 0.1],  }
# 28: blue_1--gantry_0--: { [-0.09976, -0.0201, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -0.1001, -0.1, 0.1],  }
# 30: blue_1--gantry_0--: { [0.1249, 0.176, 0.8064, -0.7488, 0.4996, 0.6512], [0.0001, 0.1, 0.09992, 0.09998, 0.12],  }
# 31: stack: { [0.1, 0.1, 0.72, 0, 0, 0.1], [0.1, 0.1, 0.74, 0, -0, 0.1],  }
# 32: gantry_0: { [0, 0.1, 0.09992, 0.09998, 0.12],  }






# 2: gantry_0: { [0, 0, 0, 0, 0.1],  }
# 3: gantry_0: { [0, 0, -7.963e-05, 0, 0.12],  }
# 9: blue_1--gantry_0--: { [0.0002456, 0.0799, 0.8064, -0.7493, 0.4996, 0.7507], [0.0001, 0, -7.963e-05, 0, 0.12],  }
# 11: blue_1--gantry_0--: { [-0.09976, -0.0201, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -0.1001, -0.1, 0.1],  }
# 13: gantry_0: { [0, 0, -0.1001, -0.1, 0.1],  }
# 17: gantry_0: { [0, 0, -7.963e-05, 0, 0.1],  }
# 19: blue_0--gantry_0--: { [0.0002455, 0.0799, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -7.963e-05, 0, 0.1],  }
# 20: blue_0--gantry_0--: { [0.1249, 0.176, 0.7864, -0.7488, 0.4996, 0.6512], [0.0001, 0.1, 0.09992, 0.09998, 0.1],  }
# 22: gantry_0: { [0, 0.1, 0.09992, 0.09998, 0.1],  }
# 26: gantry_0: { [0, 0, -0.1001, -0.1, 0.1],  }
# 28: blue_1--gantry_0--: { [-0.09976, -0.0201, 0.7864, -0.7493, 0.4996, 0.7507], [0.0001, 0, -0.1001, -0.1, 0.1],  }
# 30: blue_1--gantry_0--: { [0.1249, 0.176, 0.8064, -0.7488, 0.4996, 0.6512], [0.0001, 0.1, 0.09992, 0.09998, 0.12],  }
# 32: gantry_0: { [0, 0.1, 0.09992, 0.09998, 0.12],  }

import numpy as np
import imageio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

global mat
mat=np.matrix

def draw_arrow(ax, origin, direction, color):
    ax.quiver(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2], color=color, length=0.03)

def compute_direction_vectors(roll, pitch, yaw):
    # Calculate direction vectors
    roll_vec = np.array([1, 0, 0])
    pitch_vec = np.array([0, 1, 0])
    yaw_vec = np.array([0, 0, 1])

    # Apply rotations
    rotation_matrix = np.matmul(np.matmul(
        np.array([[np.cos(yaw), -np.sin(yaw), 0],
                  [np.sin(yaw), np.cos(yaw), 0],
                  [0, 0, 1]]),
        np.array([[np.cos(pitch), 0, np.sin(pitch)],
                  [0, 1, 0],
                  [-np.sin(pitch), 0, np.cos(pitch)]])),
        np.array([[1, 0, 0],
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll), np.cos(roll)]]))
    
    roll_vec = np.matmul(rotation_matrix, roll_vec)
    pitch_vec = np.matmul(rotation_matrix, pitch_vec)
    yaw_vec = np.matmul(rotation_matrix, yaw_vec)

    return roll_vec, pitch_vec, yaw_vec


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

gantry = [[0, 0, 0, 0, 0.1], 
          [0, 0, -7.963e-05, 0, 0.12], 
          [0, 0, -0.1001, -0.1, 0.1], 
          [0, 0, -7.963e-05, 0, 0.1],
           [0, 0.1, 0.09992, 0.09998, 0.1],
             [0, 0, -0.1001, -0.1, 0.1],
             [0, 0.1, 0.09992, 0.09998, 0.12]]
gantryb1 = [[0.0001, 0, -7.963e-05, 0, 0.12],
            [0.0001, 0, -0.1001, -0.1, 0.1],
            [0.0001, 0, -0.1001, -0.1, 0.1],
            [0.0001, 0.1, 0.09992, 0.09998, 0.12]]
gantryb0 = [[0.0001, 0, -7.963e-05, 0, 0.1],
            [0.0001, 0.1, 0.09992, 0.09998, 0.1]]





fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d([-0.16,0.16])
ax.set_ylim3d([-0.16, 0.16])
ax.set_zlim3d([0, .15])

roll = 0
pitch = 0

for i in gantry:
    _, yaw, x, y, z = gantry[i]
    ax.scatter(x,y,z, color='red', s=15)
    roll_vec, pitch_vec, yaw_vec = compute_direction_vectors(roll, pitch, yaw)
    draw_arrow(ax, [x,y,z], roll_vec, 'r')
    draw_arrow(ax, [x,y,z], pitch_vec, 'g')
    draw_arrow(ax, [x,y,z], yaw_vec, 'b')


for i in gantryb1:
    _, yaw, x, y, z = gantry[i]
    ax.scatter(x,y,z, color='orange', s=15)
    roll_vec, pitch_vec, yaw_vec = compute_direction_vectors(roll, pitch, yaw)
    draw_arrow(ax, [x,y,z], roll_vec, 'r')
    draw_arrow(ax, [x,y,z], pitch_vec, 'g')
    draw_arrow(ax, [x,y,z], yaw_vec, 'b')


for i in gantryb0:
    _, yaw, x, y, z = gantry[i]
    ax.scatter(x,y,z, color='blue', s=15)
    roll_vec, pitch_vec, yaw_vec = compute_direction_vectors(roll, pitch, yaw)
    draw_arrow(ax, [x,y,z], roll_vec, 'r')
    draw_arrow(ax, [x,y,z], pitch_vec, 'g')
    draw_arrow(ax, [x,y,z], yaw_vec, 'b')


ax.view_init(elev=30, azim=150)

plt.show()
plt.close()