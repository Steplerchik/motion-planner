from motion_planner import *


class RRT(RRTBasedPlanner):
    def __init__(self, space_info, iteration_count=200, end_position_probability_sampling=0.1, edge_size=0.5):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self._iteration_count = iteration_count
        self._end_position_probability_sampling = end_position_probability_sampling
        self._edge_size = edge_size

        super().__init__(space_info)

    def create_tree(self, start_position, end_position):
        self._tree = Graph(start_position)
        new_point = np.array([None] * 3)
        counter = 0

        while (not np.array_equiv(new_point, end_position)) and (
                counter != self._iteration_count):
            counter += 1
            random_point = self.space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y,
                                                        end_position, self._end_position_probability_sampling)
            nearest_tree_point = self.nearest(random_point)
            new_point = self.space_info.state.steer(nearest_tree_point, random_point, self._edge_size)
            if self.space_info.check_trajectory(nearest_tree_point, new_point):
                self.insert_node(nearest_tree_point, new_point)
