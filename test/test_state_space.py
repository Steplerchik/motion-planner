import unittest

import numpy as np

from motion_planner import wrap_angle, SE2, Rectangle, CollisionChecker


class TestStateSpace(unittest.TestCase):
    def test_wrap_angle(self):
        angle = 5.0 / 4.0 * np.pi
        wrapped_angle = wrap_angle(angle)
        self.assertEqual(wrapped_angle, - 3.0 / 4.0 * np.pi)

    def test_interpolate_se2_1(self):
        current_coordinates = np.array([1, 2, np.pi / 4])
        target_coordinates = np.array([1, 4, np.pi * 3 / 4])
        interpolated_coordinates = SE2().interpolate(current_coordinates, target_coordinates)
        self.assertEqual(list(interpolated_coordinates), [1, 3, np.pi / 2])

    def test_interpolate_se2_2(self):
        current_coordinates = np.array([1, 2, np.pi / 8])
        target_coordinates = np.array([2, 6, np.pi * 5 / 8])
        interpolated_coordinates = SE2().interpolate(current_coordinates, target_coordinates, t=0.25)
        self.assertEqual(list(interpolated_coordinates), [1.25, 3, np.pi / 4])

    def test_distance_se2_1(self):
        current_coordinates = np.array([1, 2, np.pi / 8])
        target_coordinates = np.array([4, 6, np.pi / 4])
        distance = SE2(alpha=0.1).distance(current_coordinates, target_coordinates)
        self.assertEqual(distance, np.sqrt(25 + np.square(0.1 * np.pi / 8)))

    def test_collision_in_interpolated_point_1(self):
        current_robot_coordinates = np.array([0, 1, np.pi / 2])
        shape = Rectangle(2, 1)
        target_robot_coordinates = np.array([0, 11, np.pi / 2])
        interpolated_coordinates = SE2().interpolate(current_robot_coordinates, target_robot_coordinates, t=0.25)
        obstacle_points = np.array([[0, 2.499], [0.1, 2.4]])
        result = CollisionChecker(shape, global_obstacle_points=obstacle_points).check(interpolated_coordinates)
        self.assertFalse(result)

    def test_collision_in_interpolated_point_2(self):
        current_robot_coordinates = np.array([0, 1, np.pi / 2])
        shape = Rectangle(2, 1)
        target_robot_coordinates = np.array([0, 11, np.pi / 2])
        interpolated_coordinates = SE2().interpolate(current_robot_coordinates, target_robot_coordinates, t=0.25)
        obstacle_points = np.array([[0, 2.5], [0.1, 2.4]])
        result = CollisionChecker(shape, global_obstacle_points=obstacle_points).check(interpolated_coordinates)
        self.assertTrue(result)


if __name__ == '__main__':
    unittest.main()
