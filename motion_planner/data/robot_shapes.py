from shapely.geometry import box
from abc import ABC, abstractmethod
from shapely.geometry import MultiPoint
from motion_planner.utils.math import global2local
import numpy as np


class RobotShapes(ABC):
    @abstractmethod
    def check_robot_shape_within_points(self, robot_center, global_obstacle_points):
        raise NotImplementedError('Must implement a prediction step for the filter.')


class Rectangle(RobotShapes):
    def __init__(self, length, width):
        self._shape = box(-float(length) / 2, -float(width) / 2, float(length) / 2, float(width) / 2)

    def check_robot_shape_within_points(self, robot_center, global_obstacle_points):
        local_obstacle_points = np.apply_along_axis(global2local, 1, global_obstacle_points, robot_center)
        points = MultiPoint(local_obstacle_points)
        if points.within(self._shape):
            return True
        return False
