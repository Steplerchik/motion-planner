from motion_planner import SE2
from .bezier_source_code import *


class Bezier(SE2):
    def __init__(self, offset, boundaries=None):
        self._offset = offset
        if boundaries is None:
            self.boundaries = [-1e9, 1e9, -1e9, 1e9]
        else:
            self.boundaries = boundaries
        super().__init__(boundaries=boundaries)

    def interpolate(self, first_position, second_position, t=0.5):
        x_first, y_first, angle_first = first_position
        x_second, y_second, angle_second = second_position

        _, control_points, _ = calc_4points_bezier_path(x_first, y_first, angle_first,
                                                        x_second, y_second, angle_second, self._offset)

        derivatives_cp = bezier_derivatives_control_points(control_points, 2)
        point = bezier(t, control_points)
        dt = bezier(t, derivatives_cp[1])

        interpolated_coordinates = np.append(point, np.arctan2(dt[1], dt[0]))
        return interpolated_coordinates

    def distance(self, first_position, second_position):
        x_first, y_first, angle_first = first_position
        x_second, y_second, angle_second = second_position

        _, _, distance = calc_4points_bezier_path(x_first, y_first, angle_first,
                                                  x_second, y_second, angle_second, self._offset)
        return distance
