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
        rrt = RRT(start_position, end_position, space_info=SpaceInfo(alpha=4.0))
        rrt._tree.add_vertex((1, 0, np.pi/2))
        result = rrt.nearest(point)
        print(result)
        self.assertEqual(list(result), [0, 0, 0])

    def test_RRT(self):
        start_position = np.array([2, 2, 0])
        end_position = np.array([5, 8, 0])
        robot_shape = Rectangle(1, 0.5)
        alpha = 0
        space_info = SpaceInfo(robot_shape, alpha)
        boundaries = [0, 10, 0, 10]
        nx, ny = (10, 10)
        obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
        obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
        obstacle_x = np.linspace(5, 10, 10)[np.newaxis].T
        obstacle_y = np.linspace(0, 5, 10)[np.newaxis].T
        obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
        obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
        obstacle_points = np.vstack([obstacle_1, obstacle_2])
        number_of_samples = 200
        end_position_probability_sampling = 0.1
        step_size = 0.5
        collision_check_step_size = 0.025
        rrt_planner = RRT(start_position,
                          end_position,
                          space_info,
                          boundaries,
                          obstacle_points,
                          number_of_samples,
                          end_position_probability_sampling,
                          step_size,
                          collision_check_step_size)

        rrt_planner.create_tree()
        plot_rrt(rrt_planner, obstacle_points)
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
        space_info = SpaceInfo(robot_shape, alpha)
        boundaries = None
        nx, ny = (10, 10)
        obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
        obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
        obstacle_x = np.linspace(5, 10, 10)[np.newaxis].T
        obstacle_y = np.linspace(0, 5, 10)[np.newaxis].T
        obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
        obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
        obstacle_points = np.vstack([obstacle_1, obstacle_2])
        number_of_samples = 200
        end_position_probability_sampling = 0.1
        step_size = 0.5
        collision_check_step_size = 0.025
        rrt_planner = RRT(start_position,
                          end_position,
                          space_info,
                          boundaries,
                          obstacle_points,
                          number_of_samples,
                          end_position_probability_sampling,
                          step_size,
                          collision_check_step_size)

        rrt_planner.create_tree()
        plot_rrt(rrt_planner, obstacle_points)
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
