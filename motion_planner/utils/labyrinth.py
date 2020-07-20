import numpy as np


def first():
    boundaries = [0, 10, 0, 10]
    nx, ny = (20, 20)
    obstacle_x5 = (np.ones(ny) * 5)[np.newaxis].T
    obstacle_y5 = (np.ones(nx) * 5)[np.newaxis].T
    obstacle_x = np.linspace(5, 10, nx)[np.newaxis].T
    obstacle_y = np.linspace(0, 5, ny)[np.newaxis].T
    obstacle_1 = np.hstack([obstacle_x5, obstacle_y])
    obstacle_2 = np.hstack([obstacle_x, obstacle_y5])
    obstacle_points = np.vstack([obstacle_1, obstacle_2])
    return boundaries, obstacle_points


def second():
    boundaries = [0, 10, 0, 10]
    nx, ny = (20, 5)
    obstacle_x8 = (np.ones(ny) * 8)[np.newaxis].T
    obstacle_y2 = (np.ones(nx) * 2)[np.newaxis].T
    obstacle_x08 = np.linspace(0, 8, nx)[np.newaxis].T
    obstacle_y24 = np.linspace(2, 4, ny)[np.newaxis].T
    obstacle_y4 = (np.ones(nx) * 4)[np.newaxis].T
    obstacle_x210 = np.linspace(2, 10, nx)[np.newaxis].T
    obstacle_y6 = (np.ones(nx) * 6)[np.newaxis].T

    obstacle_1 = np.hstack([obstacle_x08, obstacle_y2])
    obstacle_2 = np.hstack([obstacle_x8, obstacle_y24])
    obstacle_3 = np.hstack([obstacle_x08, obstacle_y4])
    obstacle_4 = np.hstack([obstacle_x210, obstacle_y6])

    obstacle_points = np.vstack([obstacle_1, obstacle_2, obstacle_3, obstacle_4])
    return boundaries, obstacle_points


def third():
    boundaries = [0, 10, 0, 10]
    nx, ny = (20, 5)
    obstacle_x8 = (np.ones(ny) * 8)[np.newaxis].T
    obstacle_y2 = (np.ones(nx) * 2)[np.newaxis].T
    obstacle_x08 = np.linspace(0, 8, nx)[np.newaxis].T
    obstacle_y24 = np.linspace(2, 4, ny)[np.newaxis].T
    obstacle_y4 = (np.ones(nx) * 4)[np.newaxis].T

    obstacle_1 = np.hstack([obstacle_x08, obstacle_y2])
    obstacle_2 = np.hstack([obstacle_x8, obstacle_y24])
    obstacle_3 = np.hstack([obstacle_x08, obstacle_y4])

    obstacle_points = np.vstack([obstacle_1, obstacle_2, obstacle_3])
    return boundaries, obstacle_points


def fourth():
    boundaries = [0, 10, 0, 10]
    nx, ny = (20, 5)
    obstacle_x8 = (np.ones(ny) * 8)[np.newaxis].T
    obstacle_y2 = (np.ones(nx) * 2)[np.newaxis].T
    obstacle_x08 = np.linspace(0, 8, nx)[np.newaxis].T
    obstacle_y24 = np.linspace(2, 4, ny)[np.newaxis].T
    obstacle_y4 = (np.ones(nx) * 4)[np.newaxis].T
    obstacle_x210 = np.linspace(2, 10, nx)[np.newaxis].T
    obstacle_y7 = (np.ones(nx) * 7)[np.newaxis].T

    obstacle_1 = np.hstack([obstacle_x08, obstacle_y2])
    obstacle_2 = np.hstack([obstacle_x8, obstacle_y24])
    obstacle_3 = np.hstack([obstacle_x08, obstacle_y4])
    obstacle_4 = np.hstack([obstacle_x210, obstacle_y7])

    obstacle_points = np.vstack([obstacle_1, obstacle_2, obstacle_3, obstacle_4])
    return boundaries, obstacle_points