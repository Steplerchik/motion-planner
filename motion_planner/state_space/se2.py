import numpy as np

from motion_planner import wrap_angle


class SE2(object):
    def __init__(self, alpha=0.0):
        self._alpha = alpha

    @staticmethod
    def interpolate(first_coordinates, second_coordinates, t=0.5):
        x_coordinate = second_coordinates[0] * t + first_coordinates[0] * (1 - t)
        y_coordinate = second_coordinates[1] * t + first_coordinates[1] * (1 - t)
        angle = first_coordinates[2]

        if second_coordinates.shape[-1] == 3:
            angle = wrap_angle(second_coordinates[2] * t + first_coordinates[2] * (1 - t))

        interpolated_coordinates = np.array([x_coordinate, y_coordinate, angle])
        return interpolated_coordinates

    def distance(self, first_coordinates, second_coordinates):
        linear_distance = np.linalg.norm(second_coordinates[:2] - first_coordinates[:2])
        angle_difference = 0

        if second_coordinates.shape[-1] == 3:
            angle_difference = wrap_angle(second_coordinates[2] - first_coordinates[2])

        distance = np.sqrt(np.square(linear_distance) + np.square(self._alpha * angle_difference))
        return distance







