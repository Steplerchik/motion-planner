import numpy as np

from motion_planner import *


class SpaceInfo(object):
    def __init__(self, state_space, robot_shape, collision_check_step_size, global_obstacle_points):
        self._collision_check_step_size = collision_check_step_size
        self.state = state_space
        self.collision = CollisionChecker(robot_shape, global_obstacle_points)

    def distance(self, first_position, second_position):
        return self.state.distance(first_position, second_position)

    def check(self, current_position):
        return self.collision.check(current_position)

    def check_trajectory(self, start, finish):
        distance = self.distance(start, finish)
        normed_step_size = self._collision_check_step_size / distance
        t_points = np.arange(0, 1, normed_step_size)
        for t in t_points:
            interpolated_point = self.state.interpolate(start, finish, t)
            if self.check(interpolated_point):
                return False
        if self.check(finish):
            return False
        return True
