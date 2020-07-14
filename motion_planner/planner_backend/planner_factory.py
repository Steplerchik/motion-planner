from motion_planner import *


class PlannerFactory(object):
    def __init__(self, start_position=np.array([3.5, 1, np.pi / 2]),
                 end_position=np.array([9, 6.5, 0]),
                 robot_shape=Rectangle(1, 0.5),
                 collision_check_step_size=0.025,
                 labyrinth=labyrinth.first(),
                 state_space=SE2,
                 state_space_parameter=0
                 ):
        boundaries, self.obstacle_points = labyrinth
        state_space = state_space(state_space_parameter, boundaries)
        self.space_info = SpaceInfo(state_space, robot_shape, collision_check_step_size, self.obstacle_points)
        self.start_position = start_position
        self.end_position = end_position

    def make_planner(self, planner_type, parameters=None):
        if parameters is None:
            parameters = {}
        rrt_planner = planner_type(self.space_info, **parameters)
        return rrt_planner

    def make_optimization_planner(self, planner_type, optimization_objective, parameters=None):
        if parameters is None:
            parameters = {}
        rrt_planner = planner_type(self.space_info, optimization_objective, **parameters)
        return rrt_planner
