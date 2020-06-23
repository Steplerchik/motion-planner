import unittest

import numpy as np

from motion_planner import *


class TestPlanner(unittest.TestCase):
    def test_box_1(self):
        state_space = SE2()
        sample = state_space.sample(0, 1, 0, 1)
        print(sample)
        if (sample[:2].all() <= 1) and (sample[:2].all() >= 0) and (sample[2] >= -np.pi) and (sample[2] <= np.pi):
            result = True
        else:
            result = False
        self.assertTrue(result)

    def test_nearest(self):
        start_position = np.array([0, 0, 0])
        end_position = np.array([0, 1, np.pi/2])

        # vertices = [(0, 0, 0), (1, 0, np.pi/2)]
        point = np.array([1, 1, 0])
        state_space = SE2(4.0)
        space_info = SpaceInfo(state_space, Rectangle(0, 0), 0.025, np.array([]))
        rrt = RRT(space_info)
        rrt._tree = Graph(start_position)
        rrt._tree.add_vertex((1, 0, np.pi/2))
        result = rrt.nearest(point)
        print(result)
        self.assertEqual(list(result), [0, 0, 0])

    def test_RRT(self):
        start_position = np.array([3.5, 1, 0])
        end_position = np.array([9, 6.5, 0])
        robot_shape = Rectangle(1, 0.5)
        alpha = 0
        boundaries = [0, 10, 0, 10]
        nx, ny = (10, 10)
        collision_check_step_size = 0.025
        obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
        obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
        obstacle_x = np.linspace(5, 10, 10)[np.newaxis].T
        obstacle_y = np.linspace(0, 5, 10)[np.newaxis].T
        obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
        obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
        obstacle_points = np.vstack([obstacle_1, obstacle_2])
        state_space = SE2(alpha, boundaries)
        space_info = SpaceInfo(state_space, robot_shape, collision_check_step_size, obstacle_points)
        iteration_count = 200
        end_position_probability_sampling = 0.1
        step_size = 0.5
        rrt_planner = RRT(space_info, iteration_count, end_position_probability_sampling, step_size)

        rrt_planner.get_trajectory(start_position, end_position)
        plot_rrt(rrt_planner, start_position, end_position, obstacle_points)
        print(rrt_planner.cost)
        self.assertTrue(True)

    def test_boundaries(self):
        sampled_point = SE2().sample()
        print(sampled_point)
        self.assertTrue(True)

    def test_RRT_infinite_boundaries(self):
        start_position = np.array([2, 2, 0])
        end_position = np.array([5, 8, 0])
        robot_shape = Rectangle(1, 0.5)
        alpha = 0
        boundaries = None
        nx, ny = (10, 10)
        collision_check_step_size = 0.025
        obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
        obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
        obstacle_x = np.linspace(5, 10, 10)[np.newaxis].T
        obstacle_y = np.linspace(0, 5, 10)[np.newaxis].T
        obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
        obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
        obstacle_points = np.vstack([obstacle_1, obstacle_2])
        state_space = SE2(alpha, boundaries)
        space_info = SpaceInfo(state_space, robot_shape, collision_check_step_size, obstacle_points)
        iteration_count = 200
        end_position_probability_sampling = 0.1
        step_size = 0.5
        rrt_planner = RRT(space_info, iteration_count, end_position_probability_sampling, step_size)
        rrt_planner.get_trajectory(start_position, end_position)
        print(rrt_planner.cost)
        self.assertTrue(True)

    def test_RRT_Dubins(self):
        start_position = np.array([3.5, 1, np.pi/2])
        end_position = np.array([9, 6.5, 0])
        robot_shape = Rectangle(1, 0.5)
        alpha = 0
        boundaries = [0, 10, 0, 10]
        nx, ny = (10, 10)
        collision_check_step_size = 0.025
        obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
        obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
        obstacle_x = np.linspace(5, 10, 10)[np.newaxis].T
        obstacle_y = np.linspace(0, 5, 10)[np.newaxis].T
        obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
        obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
        obstacle_points = np.vstack([obstacle_1, obstacle_2])
        curvature = 1.0
        state_space = Dubins(curvature, boundaries)
        space_info = SpaceInfo(state_space, robot_shape, collision_check_step_size, obstacle_points)
        iteration_count = 200
        end_position_probability_sampling = 0.2
        step_size = 0.5
        rrt_planner = RRT(space_info, iteration_count, end_position_probability_sampling, step_size)
        rrt_planner.get_trajectory(start_position, end_position)
        plot_rrt(rrt_planner, start_position, end_position, obstacle_points)
        print(rrt_planner.cost)
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
