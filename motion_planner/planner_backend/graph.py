from collections import deque


class Graph(object):
    def __init__(self, root_position):
        root_position = tuple(root_position)
        self.vertices = [root_position]
        self.edges = []

        self._costs = {root_position: 0}
        self._parents = {root_position: tuple()}

    def add_vertex(self, position):
        position = tuple(position)
        if position not in self.vertices:
            self.vertices.append(position)
            self._costs[position] = 0
            self._parents[position] = tuple()

    def add_edge(self, parent_position, child_position, distance):
        parent_position = tuple(parent_position)
        child_position = tuple(child_position)
        self.edges.append((parent_position, child_position))
        self._costs[child_position] = distance
        self._parents[child_position] = parent_position

    def find_trajectory(self, end_position):
        end_position = tuple(end_position)
        trajectory = deque()
        cost = 0
        parent_vertices = self._parents
        if end_position in parent_vertices:
            current_vertex = end_position
            while parent_vertices[current_vertex] is not tuple():
                trajectory.appendleft(current_vertex)
                cost += self._costs[current_vertex]
                current_vertex = parent_vertices[current_vertex]
            trajectory.appendleft(current_vertex)
        return trajectory, cost

    def reconnect(self, new_parent_position, child_position, distance):
        child_position = tuple(child_position)
        old_parent_position = self._parents[child_position]
        self.edges.remove((old_parent_position, child_position))
        self.add_edge(new_parent_position, child_position, distance)
