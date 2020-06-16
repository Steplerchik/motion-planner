import unittest
import numpy as np
from motion_planner.utils.math import global2local, local2global

from motion_planner.data.collision_checker import CollisionChecker
from motion_planner.data.robot_shapes import Rectangle


class TestCollisionChecker(unittest.TestCase):
    def test_absence_of_shape(self):
        robot_center = np.array([0, 0, 0])
        obstacle_points = np.array([[1, 1], [0, 1], [1, 0], [0, 0]])
        result = CollisionChecker(robot_center, obstacle_points).is_collision()
        self.assertFalse(result)

    def test_rectangular_shape_collision_true(self):
        robot_center = np.array([0, 0, 0])
        obstacle_points = np.array([[1, 1], [0, 1], [1, 0], [0, 0]])
        shape = Rectangle(2, 3).shape()
        result = CollisionChecker(robot_center, obstacle_points).is_collision(shape)
        self.assertTrue(result)

    def test_rectangular_shape_collision_false(self):
        robot_center = np.array([0.5, 0.5, np.pi/4])
        obstacle_points = np.array([[1, 1], [0, 1], [1, 0], [0, 0]])
        shape = Rectangle(1.1, 1.1).shape()
        result = CollisionChecker(robot_center, obstacle_points).is_collision(shape)
        self.assertFalse(result)

    def test_global2local(self):
        global_point = np.array([2, 2, 0])
        source_point = np.array([0, 0, np.pi/2])
        result = global2local(global_point, source_point)
        self.assertEqual(list(result), [2, -1.9999999999999998, -np.pi/2])

    def test_local2global(self):
        local_point = np.array([1, 0])
        source_point = np.array([1, 2, np.pi/2])
        result = local2global(local_point, source_point)
        self.assertEqual(list(result), [1, 3])

    def test_long_rectangular_shape_collision_false(self):
        robot_center = np.array([0.5, 0.5, 0])
        obstacle_points = np.array([[1, 1], [0, 1], [1, 0], [0, 0]])
        shape = Rectangle(0.2, 5).shape()
        result = CollisionChecker(robot_center, obstacle_points).is_collision(shape)
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
