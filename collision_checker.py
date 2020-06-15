from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from change_coordinate_system import global2local


class CollisionChecker(object):
    def __init__(self, robot_center, obstacle_points, robot_shape=Polygon()):

        self.robot_center = robot_center
        self.robot_shape = robot_shape
        self.obstacle_points = obstacle_points

    def is_collision(self):
        for global_point in self.obstacle_points:
            local_point = global2local(global_point, self.robot_center)
            point = Point(local_point)
            if point.within(self.robot_shape):
                return True
        return False
