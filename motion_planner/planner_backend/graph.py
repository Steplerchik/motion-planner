import numpy as np


class Graph(object):
    def __init__(self, root_position):
        root_position = tuple(root_position)
        self.vertices = [root_position]
        self.edges = []

        self._start_index = 0
        self._vertex2index = {root_position: self._start_index}
        self._neighbours = {self._start_index: []}
        # self.distances = {self._start_index: 0.0}

    def add_vertex(self, position):
        position = tuple(position)
        if position in self._vertex2index.keys():
            return self._vertex2index[position]
        index = len(self.vertices)
        self.vertices.append(position)
        self._vertex2index[position] = index
        self._neighbours[index] = []
        return index

    def add_edge(self, index1, index2, distance):
        self.edges.append((index1, index2))
        self._neighbours[index1].append((index2, distance))
        self._neighbours[index2].append((index1, distance))
