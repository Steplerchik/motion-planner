from abc import ABC, abstractmethod

import numpy as np

from motion_planner import *


class RRTBasedPlanner(ABC):
    def __init__(self, space_info):
        self._tree = Graph(np.array([None] * 3))
        self.space_info = space_info
        self._trajectory = None
        self._cost = None

    @property
    def tree(self):
        return self._tree

    @property
    def trajectory(self):
        return self._trajectory

    @property
    def cost(self):
        return self._cost

    def insert_node(self, parent_position, child_position):
        self._tree.add_vertex(child_position)
        cost = self.space_info.distance(parent_position, child_position)
        self._tree.add_edge(parent_position, child_position, cost)

    def nearest(self, position):
        distances = [self.space_info.distance(list(vertex), position) for vertex in self._tree.vertices]
        return np.array(list(self._tree.vertices[distances.index(min(distances))]))

    @abstractmethod
    def create_tree(self, start_position, end_position):
        raise NotImplementedError()

    def get_trajectory(self, start_position, end_position):
        self.create_tree(start_position, end_position)
        self._trajectory, self._cost = self.tree.find_trajectory(end_position)
        return self._trajectory
