from abc import ABC, abstractmethod

import numpy as np

from motion_planner import *
from collections import deque


class RRTBasedPlanner(ABC):
    def __init__(self, start_position, end_position, space_info=SpaceInfo()):
        self.start_position = start_position
        self.end_position = end_position
        self._tree = Graph(start_position)
        self.space_info = space_info
        self._path = None

    @property
    def tree(self):
        return self._tree

    @property
    def path(self):
        return self._path

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

    def find_path(self):
        start_index = self.tree.add_vertex(self.start_position)
        end_index = self.tree.add_vertex(self.end_position)

        tree_node_indexes = list(self.tree.neighbours.keys())
        tree_node_distances = {node_index: float('inf') for node_index in tree_node_indexes}
        parent_node_indexes = {node_index: None for node_index in tree_node_indexes}
        tree_node_distances[start_index] = 0

        while tree_node_indexes:
            current_node_index = min(tree_node_indexes, key=lambda node_index: tree_node_distances[node_index])
            tree_node_indexes.remove(current_node_index)
            if tree_node_distances[current_node_index] == float('inf'):
                break

            for neighbour_index, neighbour_cost in self.tree.neighbours[current_node_index]:
                full_neighbour_cost = tree_node_distances[current_node_index] + neighbour_cost
                if full_neighbour_cost < tree_node_distances[neighbour_index]:
                    tree_node_distances[neighbour_index] = full_neighbour_cost
                    parent_node_indexes[neighbour_index] = current_node_index

        path = deque()
        current_node_index = end_index
        while parent_node_indexes[current_node_index] is not None:
            path.appendleft(self.tree.vertices[current_node_index])
            current_node_index = parent_node_indexes[current_node_index]
        path.appendleft(self.tree.vertices[current_node_index])
        self._path = path
