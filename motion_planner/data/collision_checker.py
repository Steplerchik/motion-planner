from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon
from motion_planner.utils.math import global2local
import numpy as np


class CollisionChecker(object):
    def __init__(self, robot_center, obstacle_points):

        self._robot_center = robot_center
        self._global_obstacle_points = obstacle_points

    def is_collision(self, robot_shape=Polygon()):
        local_obstacle_points = np.apply_along_axis(global2local, 1, self._global_obstacle_points, self._robot_center)
        points = MultiPoint(local_obstacle_points)
        if points.within(robot_shape):
            return True
        return False

