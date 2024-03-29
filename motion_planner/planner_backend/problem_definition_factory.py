from motion_planner import *


class ProblemDefinitionFactory(object):
    def __init__(self, start_position=np.array([3.5, 1, np.pi / 2]),
                 end_position=np.array([9, 6.5, 0]),
                 robot_shape=Rectangle(1, 0.5),
                 collision_check_step_size=0.025,
                 labyrinth=labyrinth.first(),
                 state_space=SE2,
                 state_space_parameter=0,
                 planner_type=RRTWithoutCollisionCheck,
                 planner_parameters=None,
                 heuristic=NavigationFunctionNF1,
                 heuristic_resolution=0.5,
                 optimization_objective=CostPenaltyObjective,
                 optimization_objective_parameters=None
                 ):
        if optimization_objective_parameters is None:
            optimization_objective_parameters = {}
        self.planner_type = planner_type
        self.planner_parameters = planner_parameters
        self.planner_factory = PlannerFactory(robot_shape=robot_shape,
                                              collision_check_step_size=collision_check_step_size,
                                              labyrinth=labyrinth,
                                              state_space=state_space,
                                              state_space_parameter=state_space_parameter)

        self.navigation_function = heuristic(goal_position=end_position,
                                             labyrinth=labyrinth,
                                             resolution=heuristic_resolution,
                                             second_goal_position=start_position
                                             )
        self.optimization_objective = optimization_objective(self.planner_factory.space_info, self.navigation_function,
                                                             **optimization_objective_parameters)
        self._start_position = start_position
        self._end_position = end_position

    def make_problem(self):
        planner = self.planner_factory.make_planner(self.planner_type, self.planner_parameters)
        return ProblemDefinition(self._start_position, self._end_position, planner, self.optimization_objective)

    def make_optimization_problem(self):
        planner = self.planner_factory.make_optimization_planner(self.planner_type, self.optimization_objective,
                                                                 self.planner_parameters)
        return ProblemDefinition(self._start_position, self._end_position, planner, self.optimization_objective)
