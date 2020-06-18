import numpy as np
from matplotlib import pyplot as plt

from motion_planner import global2local, Rectangle


def plot_local_shape_obstacles(obstacle_points, current_coordinates, shape=Rectangle(0, 0)):
    local_obstacle_points = np.apply_along_axis(global2local, 1, obstacle_points, current_coordinates)
    x, y = shape._shape.exterior.xy
    x_obstacle = local_obstacle_points[:, 0]
    y_obstacle = local_obstacle_points[:, 1]

    fig = plt.figure(1, figsize=(5, 5), dpi=90)
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.plot(x_obstacle, y_obstacle, 'o')

    ax.set_aspect(1)
    ax.set_title('Robot shape and local obstacle coordinates')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    plt.show()
