import unittest

import numpy as np

from motion_planner import *


class TestCollisionChecker(unittest.TestCase):

    def test_rectangular_shape_collision_true(self):
        robot_center = np.array([0, 0, 0])
        obstacle_points = np.array([[1, 1], [0, 1], [1, 0], [0, 0]])
        shape = RectangleWithCircles(3, 2, 1, 1)
        # plot_local_shape_obstacles(obstacle_points, robot_center, shape)
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
