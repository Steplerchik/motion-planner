from abc import ABC

from shapely.geometry import MultiPoint, box


class RobotShape(ABC):
    def __init__(self, shape):
        self._shape = shape

    def check(self, local_obstacle_points):
        points = MultiPoint(local_obstacle_points)
        if self._shape.intersects(points):
        # if points.within(self._shape):
            return True
        return False


class Rectangle(RobotShape):
    def __init__(self, length, width):
        self._shape = box(-float(length) / 2, -float(width) / 2, float(length) / 2, float(width) / 2)
        super().__init__(self._shape)
