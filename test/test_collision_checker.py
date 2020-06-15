import unittest
import numpy as np
from change_coordinate_system import global2local
from change_coordinate_system import local2global

from collision_checker import CollisionChecker
from robot_shapes import Rectangle


class TestCollisionChecker(unittest.TestCase):
    def test_absence_of_shape(self):
        robot_center = (0, 0, 0)
        obstacle_points = [(1, 1), (0, 1), (1, 0), (0, 0)]
        result = CollisionChecker(robot_center, obstacle_points).is_collision()
        self.assertFalse(result)

    def test_rectangular_shape_collision_true(self):
        robot_center = (0, 0, 0)
        obstacle_points = [(1, 1), (0, 1), (1, 0), (0, 0)]
        shape = Rectangle(2, 3).shape()
        result = CollisionChecker(robot_center, obstacle_points, shape).is_collision()
        self.assertTrue(result)

    def test_rectangular_shape_collision_false(self):
        robot_center = (0.5, 0.5, np.pi/4)
        obstacle_points = [(1, 1), (0, 1), (1, 0), (0, 0)]
        shape = Rectangle(1.1, 1.1).shape()
        result = CollisionChecker(robot_center, obstacle_points, shape).is_collision()
        self.assertFalse(result)

    def test_global2local(self):
        global_point = (2, 2, 0)
        source_point = (0, 0, np.pi/2)
        result = global2local(global_point, source_point)
        self.assertEqual(list(result), [2, -1.9999999999999998, -np.pi/2])

    def test_local2global(self):
        local_point = (1, 0)
        source_point = (1, 2, np.pi/2)
        result = local2global(local_point, source_point)
        self.assertEqual(list(result), [1, 3])

    def test_long_rectangular_shape_collision_false(self):
        robot_center = (0.5, 0.5, 0)
        obstacle_points = [(1, 1), (0, 1), (1, 0), (0, 0)]
        shape = Rectangle(0.2, 5).shape()
        result = CollisionChecker(robot_center, obstacle_points, shape).is_collision()
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
