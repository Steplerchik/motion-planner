from queue import PriorityQueue

from motion_planner import *

unfeasible_cost = float('inf')


class NavigationFunctionNF1(object):
    def __init__(self, goal_position, labyrinth=labyrinth.first(), resolution=1.0, **kwargs):
        self._goal_position = goal_position
        self.labyrinth = labyrinth
        boundaries, self._obstacle_points = labyrinth
        self._boundaries_polygon = box(boundaries[0], boundaries[2], boundaries[1], boundaries[3])
        self._obstacle_points_polygon = MultiPoint(self._obstacle_points)
        self.resolution = resolution
        self._neighbours = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        self._bilinear_point_groups = [[(1, 0), (1, 1), (0, 1)],
                                       [(0, 1), (-1, 1), (-1, 0)],
                                       [(-1, 0), (-1, -1), (0, -1)],
                                       [(0, -1), (1, -1), (1, 0)]]
        self._cost_map = self.build_cost_map(goal_position)

    def build_cost_map(self, goal_position):
        cost_map = {tuple(goal_position[:2]): 0}
        cell_queue = PriorityQueue()
        cell_queue.put((0, tuple(goal_position[:2])))

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
                    new_cost = cost_map[current_cell] + move_cost
                    if global_neighbour not in cost_map:
                        cost_map[global_neighbour] = new_cost
                        cell_queue.put((new_cost, global_neighbour))
        return cost_map

    @property
    def get_cost_map(self):
        return self._cost_map

    def get_cost(self, current_position):
        resolution = self.resolution
        cell_position_x = self._goal_position[0] + round(
            (current_position[0] - self._goal_position[0]) / resolution) * resolution
        cell_position_y = self._goal_position[1] + round(
            (current_position[1] - self._goal_position[1]) / resolution) * resolution
        cell_point = (cell_position_x, cell_position_y)
        cost = unfeasible_cost
        if cell_point in self._cost_map.keys():
            cost = self._cost_map[cell_point]
            for point_group in self._bilinear_point_groups:
                global_point_group = []
                for point in point_group:
                    point_x = cell_position_x + point[0] * resolution
                    point_y = cell_position_y + point[1] * resolution
                    global_point_group.append((point_x, point_y))

                if all(point in self._cost_map for point in global_point_group):
                    points = [(cell_point[0], cell_point[1], self._cost_map[cell_point])]
                    for point in global_point_group:
                        points.append((point[0], point[1], self._cost_map[point]))
                    bilinear_cost = self.bilinear_interpolation(current_position[0], current_position[1], points)
                    if bilinear_cost:
                        return bilinear_cost
        return cost

    @staticmethod
    def bilinear_interpolation(x, y, points):
        sorted_points = sorted(points)
        (x1, y1, q11), (_x1, y2, q12), (x2, _y1, q21), (_x2, _y2, q22) = sorted_points
        if x1 != _x1 or x2 != _x2 or y1 != _y1 or y2 != _y2:
            return False
        if not x1 <= x <= x2 or not y1 <= y <= y2:
            return False
        return (q11 * (x2 - x) * (y2 - y) +
                q21 * (x - x1) * (y2 - y) +
                q12 * (x2 - x) * (y - y1) +
                q22 * (x - x1) * (y - y1)
                ) / ((x2 - x1) * (y2 - y1) + 0.0)
