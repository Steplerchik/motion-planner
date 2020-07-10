from queue import PriorityQueue

from motion_planner import *

unfeasible_cost = 1e9


class NavigationFunctionNF1(object):
    def __init__(self, goal_position, the_labyrinth=labyrinth.first(), resolution=1.0):
        self._goal_position = goal_position
        self.labyrinth = the_labyrinth
        boundaries, self._obstacle_points = the_labyrinth
        self._boundaries_polygon = box(boundaries[0], boundaries[2], boundaries[1], boundaries[3])
        self._obstacle_points_polygon = MultiPoint(self._obstacle_points)
        self.resolution = resolution
        self._cost_map = {tuple(goal_position[:2]): 0}
        self._neighbours = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        self.build_cost_map()

    def build_cost_map(self):
        cell_queue = PriorityQueue()
        cell_queue.put((0, tuple(self._goal_position[:2])))

        while not cell_queue.empty():
            current_cell = cell_queue.get()[1]
            for neighbour in self._neighbours:
                neighbour_x = current_cell[0] + neighbour[0] * self.resolution
                neighbour_y = current_cell[1] + neighbour[1] * self.resolution
                global_neighbour = (neighbour_x, neighbour_y)
                cell_neighbour = Point(global_neighbour).buffer(self.resolution / 2).envelope
                if not self._boundaries_polygon.contains(cell_neighbour) or cell_neighbour.intersects(
                        self._obstacle_points_polygon):
                    pass
                else:
                    move_cost = np.sqrt(neighbour[0] ** 2 + neighbour[1] ** 2) * self.resolution
                    new_cost = self._cost_map[current_cell] + move_cost
                    if global_neighbour not in self._cost_map:
                        self._cost_map[global_neighbour] = new_cost
                        cell_queue.put((new_cost, global_neighbour))
        return self._cost_map

    @property
    def get_cost_map(self):
        return self._cost_map

    def get_cost(self, current_position):
        position = Point(tuple([current_position[0], current_position[1]]))
        cost = unfeasible_cost
        for point in self._cost_map.keys():
            cell = Point(point).buffer(self.resolution / 2).envelope
            if cell.contains(position):
                cost = self._cost_map[point]
                break
        return cost
