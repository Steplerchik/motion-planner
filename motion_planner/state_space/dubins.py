from random import uniform

from .dubins_source_code import *


class Dubins(object):
    def __init__(self, curvature, boundaries=None, dubins_step_size=0.1):
        self._curvature = curvature
        self._dubins_step_size = dubins_step_size
        if boundaries is None:
            self.boundaries = [-1e9, 1e9, -1e9, 1e9]
        else:
            self.boundaries = boundaries

    def interpolate(self, first_position, second_position, t=0.5):
        if t == 0:
            return first_position
        x_first, y_first, angle_first = first_position
        x_second, y_second, angle_second = second_position
        path_x, path_y, path_angle, _, _ = dubins_path_planning(x_first, y_first, angle_first,
                                                                x_second, y_second, angle_second,
                                                                self._curvature, self._dubins_step_size, t)
        interpolated_coordinates = np.array([path_x[-1], path_y[-1], path_angle[-1]])
        return interpolated_coordinates

    def distance(self, first_position, second_position):
        x_first, y_first, angle_first = first_position
        x_second, y_second, angle_second = second_position

        _, _, _, _, distance = dubins_path_planning(x_first, y_first, angle_first,
                                                    x_second, y_second, angle_second,
                                                    self._curvature, self._dubins_step_size)
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
        new_point = self.interpolate(first_position, second_position, t)
        return new_point
