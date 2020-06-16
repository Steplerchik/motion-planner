from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from motion_planner.math import global2local


class CollisionChecker(object):
    def __init__(self, robot_center, obstacle_points, robot_shape=Polygon()):

        self._robot_center = robot_center
        self._robot_shape = robot_shape
        self._obstacle_points = obstacle_points

    def is_collision(self):
        for global_point in self._obstacle_points:
            local_point = global2local(global_point, self._robot_center)
            point = Point(local_point)
            if point.within(self._robot_shape):
                return True
        return False
