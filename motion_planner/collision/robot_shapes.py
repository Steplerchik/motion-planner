from abc import ABC

import numpy as np
from shapely.geometry import Point, MultiPoint, box, Polygon


class RobotShape(ABC):
    def __init__(self, shape):
        self._shape = shape

    def check(self, local_obstacle_points):
        points = MultiPoint(local_obstacle_points)
        if self._shape.intersects(points):
            return True
        return False

    @property
    def shape(self):
        return self._shape


class Rectangle(RobotShape):
    def __init__(self, length, width):
        self._shape = box(-float(length) / 2, -float(width) / 2, float(length) / 2, float(width) / 2)
        super().__init__(self._shape)


class RectangleWithCircles(RobotShape):
    def __init__(self, length, width, front_radius, rear_radius, steps=200):
        if (front_radius < width / 2) or (rear_radius < width / 2):
            raise ValueError('Circle radius should be more than or equal to width / 2')

        def polar_point(origin_point, angle, distance):
            return [origin_point.x + np.cos(angle) * distance, origin_point.y + np.sin(angle) * distance]

        front_center = Point(float(length) / 2 - front_radius, 0)
        front_half_angle = np.arcsin(width / 2 / front_radius)
        front_step_angle_width = 2 * front_half_angle / steps
        front_segment_vertices = []
        front_segment_vertices.append(polar_point(front_center, -front_half_angle, front_radius))

        rear_center = Point(-float(length) / 2 + rear_radius, 0)
        rear_half_angle = np.arcsin(width / 2 / rear_radius)
        rear_step_angle_width = 2 * rear_half_angle / steps
        rear_segment_vertices = []
        rear_segment_vertices.append(polar_point(rear_center, rear_half_angle - np.pi, rear_radius))

        for z in range(1, steps):
            front_segment_vertices.append(
                (polar_point(front_center, -front_half_angle + z * front_step_angle_width, front_radius)))
            rear_segment_vertices.append(
                (polar_point(rear_center, rear_half_angle - np.pi - z * rear_step_angle_width, rear_radius)))
        front_segment_vertices.append(polar_point(front_center, front_half_angle, front_radius))
        front_segment_vertices.append(polar_point(front_center, -front_half_angle, front_radius))
        rear_segment_vertices.append(polar_point(rear_center, -rear_half_angle + np.pi, rear_radius))
        rear_segment_vertices.append(polar_point(rear_center, rear_half_angle - np.pi, rear_radius))
        front_polygon = Polygon(front_segment_vertices)
        rear_polygon = Polygon(rear_segment_vertices)

        rectangle = box(-float(length) / 2 + rear_radius - rear_radius * np.cos(rear_half_angle), -float(width) / 2, float(length) / 2 - front_radius + front_radius * np.cos(front_half_angle), float(width) / 2)
        self._shape = rectangle.union(front_polygon).union(rear_polygon)
        super().__init__(self._shape)
