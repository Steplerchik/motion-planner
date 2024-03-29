import numpy as np

from motion_planner.utils import global2local
from .robot_shapes import Rectangle


class CollisionChecker(object):
    def __init__(self, robot_shape=Rectangle(0, 0), global_obstacle_points=np.array([])):
        self._robot_shape = robot_shape
        self._global_obstacle_points = global_obstacle_points

    def check(self, robot_center_coordinates):
        local_obstacle_points = np.apply_along_axis(global2local, 1, self._global_obstacle_points,
                                                    robot_center_coordinates)
        return self._robot_shape.check(local_obstacle_points)
