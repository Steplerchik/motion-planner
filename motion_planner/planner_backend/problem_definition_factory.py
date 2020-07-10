from motion_planner import *


class ProblemDefinitionFactory(object):
    def __init__(self, start_position=np.array([3.5, 1, np.pi / 2]),
                 end_position=np.array([9, 6.5, 0]),
                 robot_shape=Rectangle(1, 0.5),
                 collision_check_step_size=0.025,
                 the_labyrinth=labyrinth.first(),
                 state_space=SE2,
                 state_space_parameter=0,
                 iteration_count=200,
                 end_position_probability_sampling=0.1,
                 step_size=0.5,
                 planner_type=RRTWithoutCollisionCheck,
                 euristics=NavigationFunctionNF1,
                 euristics_resolution=0.5,
                 optimization_objective=CostPenaltyObjective,
                 ):
        self.planner_factory = PlannerFactory(start_position=start_position,
                                         end_position=end_position,
                                         robot_shape=robot_shape,
                                         collision_check_step_size=collision_check_step_size,
                                         the_labyrinth=the_labyrinth,
                                         state_space=state_space,
                                         state_space_parameter=state_space_parameter,
                                         iteration_count=iteration_count,
                                         end_position_probability_sampling=end_position_probability_sampling,
                                         step_size=step_size)

        planner = self.planner_factory.make_planner(planner_type)
        self.navigation_function = euristics(end_position, the_labyrinth, euristics_resolution)
        self.optimization_objective = optimization_objective(self.planner_factory.space_info, self.navigation_function)
        self.problem = ProblemDefinition(start_position, end_position, planner, self.optimization_objective)
