from motion_planner import *


class RRTStar(RRTBasedPlanner):
    def __init__(self, space_info, iteration_count=200, end_position_probability_sampling=0.1, edge_size=0,
                 near_radius=0.5):
        self._min_x, self._max_x, self._min_y, self._max_y = space_info.state.boundaries
        self._iteration_count = iteration_count
        self._end_position_probability_sampling = end_position_probability_sampling
        self._edge_size = edge_size
        self._near_radius = near_radius

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
                near_tree_points = self.near(new_point)
                parent_of_new_point = self.choose_parent(near_tree_points, nearest_tree_point, new_point)
                self.insert_node(parent_of_new_point, new_point)
                self.rewire(near_tree_points, parent_of_new_point, new_point)

    def near(self, position):
        near_tree_points = [np.array(list(vertex)) for vertex in self._tree.vertices if
                            self.space_info.distance(list(vertex), position) <= self._near_radius]
        return near_tree_points

    def choose_parent(self, near_points, nearest_point, new_point):
        remove(near_points, nearest_point)
        parent_point = nearest_point
        _, min_cost = self._tree.find_trajectory(parent_point)
        min_cost += self.space_info.distance(parent_point, new_point)
        for near_point in near_points:
            new_steered_point = self.space_info.state.steer(near_point, new_point, self._edge_size)
            if self.space_info.check_trajectory(near_point, new_steered_point) and np.array_equiv(new_steered_point,
                                                                                                  new_point):
                _, new_cost = self._tree.find_trajectory(near_point)
                new_cost += self.space_info.distance(near_point, new_point)
                if new_cost < min_cost:
                    parent_point = near_point
                    min_cost = new_cost
        return parent_point

    def rewire(self, near_points, parent_of_new_point, new_point):
        remove(near_points, parent_of_new_point)
        for near_point in near_points:
            near_steered_point = self.space_info.state.steer(new_point, near_point, self._edge_size)
            if self.space_info.check_trajectory(new_point, near_steered_point) and np.array_equiv(near_steered_point,
                                                                                                  near_point):
                _, new_cost = self._tree.find_trajectory(new_point)
                cost_between_points = self.space_info.distance(new_point, near_point)
                new_cost += cost_between_points
                _, current_cost = self._tree.find_trajectory(near_point)
                if new_cost < current_cost:
                    self._tree.reconnect(new_point, near_point, cost_between_points)
