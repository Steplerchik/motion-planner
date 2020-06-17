import numpy as np

from motion_planner import wrap_angle


class SE2(object):
    def __init__(self, current_robot_coordinates, target_robot_coordinates):
        self._current_coordinates = np.array(current_robot_coordinates)
        self._target_coordinates = np.array(target_robot_coordinates)
        self._target_dimension = self._target_coordinates.shape[-1]

    def interpolate(self, t=0.5):
        x_coordinate = self._target_coordinates[0] * t + self._current_coordinates[0] * (1 - t)
        y_coordinate = self._target_coordinates[1] * t + self._current_coordinates[1] * (1 - t)
        angle = self._current_coordinates[2]

        if self._target_dimension == 3:
            angle = wrap_angle(self._target_coordinates[2] * t + self._current_coordinates[2] * (1 - t))

        interpolated_coordinates = np.array([x_coordinate, y_coordinate, angle])
        return interpolated_coordinates

    def distance(self, alpha=0.0):
        linear_distance = np.linalg.norm(self._target_coordinates[:2] - self._current_coordinates[:2])
        angle_difference = 0

        if self._target_dimension == 3:
            angle_difference = wrap_angle(self._target_coordinates[2] - self._current_coordinates[2])

        distance = np.sqrt(np.square(linear_distance) + alpha * np.square(angle_difference))
        return distance







