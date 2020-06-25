from functools import lru_cache
from random import uniform

import numpy as np

from motion_planner import wrap_angle

max_boundary_coordinate = 1e9


class SE2(object):
    def __init__(self, alpha=0.0, boundaries=None):
        self._alpha = alpha
        if boundaries is None:
            self.boundaries = [-max_boundary_coordinate, max_boundary_coordinate, -max_boundary_coordinate,
                               max_boundary_coordinate]
        else:
            self.boundaries = boundaries

    @staticmethod
    @lru_cache(maxsize=32)
    def interpolate(first_position, second_position, t=0.5):
        x_coordinate = second_position[0] * t + first_position[0] * (1 - t)
        y_coordinate = second_position[1] * t + first_position[1] * (1 - t)
        angle = first_position[2]

        if (len(first_position) == 3) and (len(second_position) == 3):
            angle = wrap_angle(second_position[2] * t + first_position[2] * (1 - t))

        interpolated_coordinates = np.array([x_coordinate, y_coordinate, angle])
        return interpolated_coordinates

    def distance(self, first_position, second_position):
        linear_distance = np.linalg.norm(second_position[:2] - first_position[:2])
        angle_difference = 0

        if (len(first_position) == 3) and (len(second_position) == 3):
            angle_difference = wrap_angle(second_position[2] - first_position[2])

        distance = np.sqrt(np.square(linear_distance) + np.square(self._alpha * angle_difference))
        return distance

    @staticmethod
    def sample(min_x=-1e9, max_x=1e9, min_y=-1e9, max_y=1e9, end_position=np.zeros(3), end_position_probability=0.0):
        sampled_x_position = uniform(min_x, max_x)
        sampled_y_position = uniform(min_y, max_y)
        sampled_angle = uniform(-np.pi, np.pi)
        sampled_position = np.array([sampled_x_position, sampled_y_position, sampled_angle])
        is_end_position_sampled = np.random.choice([True, False],
                                                   p=[end_position_probability, 1 - end_position_probability])
        if is_end_position_sampled:
            return end_position
        return sampled_position

    def steer(self, first_position, second_position, edge_size=0.0):
        distance = self.distance(first_position, second_position)
        if edge_size > distance:
            return second_position
        t = edge_size / distance
        new_point = self.interpolate(tuple(first_position), tuple(second_position), t)
        return new_point
