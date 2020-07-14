import numpy as np

from motion_planner import *


def test_RRT():
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

    rrt_planner.build_trajectory(start_position, end_position)
    print(rrt_planner.cost)


if __name__ == '__main__':
    test_RRT()
