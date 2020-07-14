from motion_planner import *

planner_parameters = {
    'iteration_count': 200,
    'end_position_probability_sampling': 0.1,
    'edge_size': 0.5
}


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
        planner_parameters.update(parameters)
        rrt_planner = planner_type(self.space_info)
        rrt_planner.__dict__.update(planner_parameters)
        return rrt_planner
