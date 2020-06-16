import numpy as np

from motion_planner.utils.math import global2local
from .robot_shapes import Rectangle


class CollisionChecker(object):
    def __init__(self, robot_shape=Rectangle(0, 0)):
        self._robot_shape = robot_shape

    def check(self, robot_center_coordinates, global_obstacle_points):
        local_obstacle_points = np.apply_along_axis(global2local, 1, global_obstacle_points, robot_center_coordinates)
        return self._robot_shape.check(local_obstacle_points)
