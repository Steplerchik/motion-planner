from motion_planner import SE2
from .dubins_source_code import *

max_boundary_coordinate = 1e9


class Dubins(SE2):
    def __init__(self, curvature, boundaries=None, dubins_step_size=0.1):
        self._curvature = curvature
        self._dubins_step_size = dubins_step_size
        if boundaries is None:
            self.boundaries = [-max_boundary_coordinate, max_boundary_coordinate, -max_boundary_coordinate,
                               max_boundary_coordinate]
        else:
            self.boundaries = boundaries
        super().__init__(boundaries=boundaries)

    @lru_cache(maxsize=32)
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
