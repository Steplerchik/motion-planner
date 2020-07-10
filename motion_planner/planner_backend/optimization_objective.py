from .navigation_function import unfeasible_cost


class OptimizationObjective(object):
    def __init__(self, euristics):
        self.navigation_function = euristics

    def cost(self, trajectory, planner_distance):
        cost = planner_distance
        first_point = trajectory[0]
        previous_point_cost = self.navigation_function.get_cost(first_point)
        cost_before_obstacle = None
        cost_after_obstacle = None
        for point in trajectory:
            point_cost = self.navigation_function.get_cost(point)
            if previous_point_cost != unfeasible_cost and point_cost == unfeasible_cost:
                cost_before_obstacle = previous_point_cost
            elif previous_point_cost == unfeasible_cost and point_cost != unfeasible_cost:
                cost_after_obstacle = point_cost
            if cost_before_obstacle is not None and cost_after_obstacle is not None:
                cost += abs(cost_before_obstacle - cost_after_obstacle)
                cost_before_obstacle = None
                cost_after_obstacle = None
            previous_point_cost = point_cost
        return cost
