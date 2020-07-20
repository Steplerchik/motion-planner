from motion_planner import *

unfeasible_cost = float('inf')


class NavigationFunctionDoubleNF1(NavigationFunctionNF1):
    def __init__(self, goal_position, second_goal_position, labyrinth=labyrinth.first(), resolution=1.0, **kwargs):
        super().__init__(goal_position, labyrinth, resolution)

        second_goal_position_x = self._goal_position[0] + round(
            (second_goal_position[0] - self._goal_position[0]) / resolution) * resolution
        second_goal_position_y = self._goal_position[1] + round(
            (second_goal_position[1] - self._goal_position[1]) / resolution) * resolution

        self._second_goal_position = [second_goal_position_x, second_goal_position_y]
        self._second_cost_map = self.build_cost_map(self._second_goal_position)

        resultant_cost_map = {key: (self._cost_map[key] - self._second_cost_map[key]) / 2 for key in
                              self._cost_map.keys()}
        self._cost_map = resultant_cost_map
