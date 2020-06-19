from abc import ABC, abstractmethod

import numpy as np

from motion_planner import *


class RRTBasedPlanner(ABC):
    def __init__(self, start_position, end_position, space_info=SpaceInfo()):
        self.start_position = start_position
        self.end_position = end_position
        self._tree = Graph(start_position)
        self.space_info = space_info

    @property
    def tree(self):
        return self._tree

    def insert_node(self, parent_position, child_position):
        parent_index = self._tree.add_vertex(parent_position)
        child_index = self._tree.add_vertex(child_position)
        cost = self.space_info.state.distance(parent_position, child_position)
        self._tree.add_edge(parent_index, child_index, cost)

    def nearest(self, position):
        distances = [self.space_info.state.distance(list(vertex), position) for vertex in self._tree.vertices]
        return np.array(list(self._tree.vertices[distances.index(min(distances))]))

    def is_obstacle_free(self, first_position, second_position, collision_check_step_size, global_obstacle_points):
        distance = self.space_info.state.distance(first_position, second_position)
        normed_step_size = collision_check_step_size / distance
        t_points = np.arange(0, 1, normed_step_size)
        for t in t_points:
            interpolated_point = self.space_info.state.interpolate(first_position, second_position, t)
            if self.space_info.collision.check(interpolated_point, global_obstacle_points):
                return False
        if self.space_info.collision.check(second_position, global_obstacle_points):
            return False
        return True

    @abstractmethod
    def create_tree(self):
        raise NotImplementedError()

    # def find_path(self):
    #     start_index = self._tree._start_index

