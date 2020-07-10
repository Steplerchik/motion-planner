import numpy as np
from matplotlib import collections as mc
from matplotlib import pyplot as plt
from shapely.geometry import Point, box, MultiPoint
from descartes import PolygonPatch
from motion_planner import global2local, Rectangle, SpaceInfo


def plot_local_shape_obstacles(obstacle_points, current_coordinates, shape=Rectangle(0, 0)):
    local_obstacle_points = np.apply_along_axis(global2local, 1, obstacle_points, current_coordinates)
    x, y = shape.shape.exterior.xy
    x_obstacle = local_obstacle_points[:, 0]
    y_obstacle = local_obstacle_points[:, 1]

    fig = plt.figure(1, figsize=(5, 5), dpi=90)
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.plot(x_obstacle, y_obstacle, 'o')

    ax.set_aspect(1)
    ax.set_title('Robot shape and local obstacle coordinates')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    plt.show()


def plot_rrt(rrt, start_position, end_position, obstacle_points):
    fig, ax = plt.subplots(dpi=250)
    trajectory = rrt.trajectory
    vertices = rrt.tree.vertices
    edges = rrt.tree.edges
    tree_point_x_coordinates = [x for x, y, angle in vertices]
    tree_point_y_coordinates = [y for x, y, angle in vertices]
    obstacle_x_coordinates = obstacle_points[:, 0]
    obstacle_y_coordinates = obstacle_points[:, 1]

    ax.scatter(obstacle_x_coordinates, obstacle_y_coordinates, c='red')

    ax.scatter(tree_point_x_coordinates, tree_point_y_coordinates, c='cyan')
    tree_edges = [
        ((edge[0][0], edge[0][1]), (edge[1][0], edge[1][1]))
        for edge in edges]
    tree_lines = mc.LineCollection(tree_edges, colors='blue', linewidths=2)
    ax.add_collection(tree_lines)

    if trajectory is not None:
        trajectory_edges = [((trajectory[i][0], trajectory[i][1]), (trajectory[i + 1][0], trajectory[i + 1][1])) for i
                            in range(len(trajectory) - 1)]
        trajectory_lines = mc.LineCollection(trajectory_edges, colors='green', linewidths=3)
        ax.add_collection(trajectory_lines)

    ax.scatter(start_position[0], start_position[1], c='black', linewidths=2)
    ax.scatter(end_position[0], end_position[1], c='black', linewidths=2)

    ax.set_aspect(1)
    ax.set_title('Cost: %.2f [m]' % rrt.cost)
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    plt.show()


def plot_cost_map(navigation_function):
    fig, ax = plt.subplots(dpi=250)
    boundaries, obstacle_points = navigation_function.labyrinth
    resolution = navigation_function.resolution
    cost_map = navigation_function.get_cost_map
    max_distance = np.sqrt((boundaries[1] - boundaries[0]) ** 2 + (boundaries[3] - boundaries[2]) ** 2)
    for point in cost_map.keys():
        cell = Point(point).buffer(resolution / 2).envelope
        grey = 1 - np.exp(-3 * cost_map[point] / max_distance)
        cell_patch = PolygonPatch(cell, color=(grey, grey, grey))
        ax.add_patch(cell_patch)
    boundaries_polygon = box(boundaries[0], boundaries[2], boundaries[1], boundaries[3])
    x, y = boundaries_polygon.exterior.xy
    ax.plot(x, y)
    x = obstacle_points[:, 0]
    y = obstacle_points[:, 1]
    ax.scatter(x, y, c='red')
    ax.set_aspect(1)
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    plt.show()
