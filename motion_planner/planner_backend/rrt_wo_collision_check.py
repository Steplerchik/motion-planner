from motion_planner import *


class RRTWithoutCollisionCheck(RRTBasedPlanner):
    def __init__(self, space_info, iteration_count=0, end_position_probability_sampling=0, edge_size=0):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self.iteration_count = iteration_count
        self.end_position_probability_sampling = end_position_probability_sampling
        self.edge_size = edge_size

        super().__init__(space_info)

    def create_tree(self, start_position, end_position):
        self._tree = Graph(start_position)
        new_point = np.array([None] * 3)
        counter = 0

        while (not np.array_equiv(new_point, end_position)) and (
                counter != self.iteration_count):
            counter += 1
            random_point = self.space_info.state.sample(self._min_x, self._max_x, self._min_y, self._max_y,
                                                        end_position, self.end_position_probability_sampling)
            nearest_tree_point = self.nearest(random_point)
            new_point = self.space_info.state.steer(nearest_tree_point, random_point, self.edge_size)
            self.insert_node(nearest_tree_point, new_point)
