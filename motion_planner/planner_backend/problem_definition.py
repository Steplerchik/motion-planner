class ProblemDefinition(object):
    def __init__(self, start, finish, planner, optimization_objective):
        self.start_position = start
        self.end_position = finish
        self.planner = planner
        self.optimization_objective = optimization_objective

    def solve(self):
        self.planner.build_trajectory(self.start_position, self.end_position)

    @property
    def trajectory(self):
        return self.planner.trajectory

    @property
    def cost(self):
        return self.optimization_objective.cost(self.trajectory)
