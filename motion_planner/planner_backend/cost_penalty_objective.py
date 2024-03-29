import numpy as np

from .navigation_function import unfeasible_cost


class CostPenaltyObjective(object):
    def __init__(self, space_info, heuristic, penalty_weight=1.0):
        self.space_info = space_info
        self.navigation_function = heuristic
        self._penalty_weight = penalty_weight

    def cost(self, trajectory):
        if not trajectory:
            return 0
        trajectory = [np.array(list(point)) for point in trajectory]
        distance = self.distance(trajectory)
        penalty = self.penalty(trajectory)
        cost = distance + self._penalty_weight * penalty
        return cost

    def distance(self, trajectory):
        distance = 0
        previous_point = trajectory[0]
        for point in trajectory[1:]:
            distance += self.space_info.distance(previous_point, point)
            previous_point = point
        return distance

    def penalty(self, trajectory):
        penalty = 0
        previous_point = trajectory[0]
        previous_point_cost = self.navigation_function.get_cost(previous_point)
        cost_before_obstacle = None
        cost_after_obstacle = None
        for point in trajectory[1:]:
            point_cost = self.navigation_function.get_cost(point)

            previous_point_in_collision = self.space_info.check(previous_point)
            point_in_collision = self.space_info.check(point)

            if ((previous_point_cost != unfeasible_cost) and (not previous_point_in_collision)) and (point_cost == unfeasible_cost or point_in_collision):
                cost_before_obstacle = previous_point_cost
            elif (previous_point_cost == unfeasible_cost or previous_point_in_collision) and ((point_cost != unfeasible_cost) and (not point_in_collision)):
                cost_after_obstacle = point_cost
            elif ((previous_point_cost != unfeasible_cost) and (not previous_point_in_collision)) and ((point_cost != unfeasible_cost) and (not point_in_collision)) and (not self.space_info.check_trajectory(previous_point, point)):
                cost_before_obstacle = previous_point_cost
                cost_after_obstacle = point_cost

            if cost_before_obstacle is not None and cost_after_obstacle is not None:
                penalty += abs(cost_before_obstacle - cost_after_obstacle)
                cost_before_obstacle = None
                cost_after_obstacle = None
            previous_point = point
            previous_point_cost = point_cost
        return penalty
