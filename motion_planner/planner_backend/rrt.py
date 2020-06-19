import numpy as np

from motion_planner import *


class RRT(RRTBasedPlanner):
    def __init__(self,
                 start_position,
                 end_position,
                 space_info=SpaceInfo(),
                 boundaries=None,
                 global_obstacle_points=np.array([]),
                 number_of_samples=0,
                 end_position_probability_sampling=0,
                 edge_size=0,
                 collision_check_step_size=0):
        self._global_obstacle_points = global_obstacle_points
        self._number_of_samples = number_of_samples
        self._end_position_probability_sampling = end_position_probability_sampling
        self._edge_size = edge_size
        if boundaries is None:
            boundaries = [-1e9, 1e9, -1e9, 1e9]
        self._min_x, self._max_x, self._min_y, self._max_y = boundaries
        self._collision_check_step_size = collision_check_step_size

        super().__init__(start_position, end_position, space_info)

    def create_tree(self):
        for _ in range(self._number_of_samples):
            random_point = self.space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y,
                                                        self.end_position, self._end_position_probability_sampling)
            nearest_tree_point = self.nearest(random_point)
            new_point = self.space_info.state.steer(nearest_tree_point, random_point, self._edge_size)
            if self.is_obstacle_free(nearest_tree_point, new_point, self._collision_check_step_size,
                                     self._global_obstacle_points):
                self.insert_node(nearest_tree_point, new_point)
            if (new_point == self.end_position).all():
                break
