import numpy as np

from motion_planner import *


class PlannerFactory(object):
    def __init__(self, start_position=np.array([3.5, 1, np.pi/2]),
                 end_position=np.array([9, 6.5, 0]),
                 robot_shape=Rectangle(1, 0.5),
                 collision_check_step_size=0.025,
                 the_labyrinth=labyrinth.first(),
                 state_space=SE2,
                 state_space_parameter=0,
                 iteration_count=200,
                 end_position_probability_sampling=0.1,
                 step_size=0.5,
                 ):
        boundaries, self.obstacle_points = the_labyrinth
        state_space = state_space(state_space_parameter, boundaries)
        self._space_info = SpaceInfo(state_space, robot_shape, collision_check_step_size, self.obstacle_points)
        self._iteration_count = iteration_count
        self._end_position_probability_sampling = end_position_probability_sampling
        self._step_size = step_size
        self.start_position = start_position
        self.end_position = end_position

    def get_planner(self, planner_type):
        rrt_planner = planner_type(self._space_info, self._iteration_count, self._end_position_probability_sampling, self._step_size)
        return rrt_planner
