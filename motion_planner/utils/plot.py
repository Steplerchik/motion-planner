import numpy as np
from matplotlib import collections as mc
from matplotlib import pyplot as plt

from motion_planner import global2local, Rectangle, SpaceInfo


def plot_local_shape_obstacles(obstacle_points, current_coordinates, shape=Rectangle(0, 0)):
    local_obstacle_points = np.apply_along_axis(global2local, 1, obstacle_points, current_coordinates)
    x, y = shape._shape.exterior.xy
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


def plot_rrt(rrt, obstacle_points, space_info=SpaceInfo()):
    fig, ax = plt.subplots()
    tree = rrt.tree
    path = rrt.path

    tree_point_x_coordinates = [x for x, y, angle in tree.vertices]
    tree_point_y_coordinates = [y for x, y, angle in tree.vertices]
    obstacle_x_coordinates = obstacle_points[:, 0]
    obstacle_y_coordinates = obstacle_points[:, 1]

    ax.scatter(obstacle_x_coordinates, obstacle_y_coordinates, c='red')

    ax.scatter(tree_point_x_coordinates, tree_point_y_coordinates, c='cyan')
    tree_edges = [
        ((tree.vertices[edge[0]][0], tree.vertices[edge[0]][1]), (tree.vertices[edge[1]][0], tree.vertices[edge[1]][1]))
        for edge in tree.edges]
    tree_lines = mc.LineCollection(tree_edges, colors='blue', linewidths=2)
    ax.add_collection(tree_lines)

    if path is not None:
        path_edges = [((path[i][0], path[i][1]), (path[i + 1][0], path[i + 1][1])) for i in range(len(path) - 1)]
        path_lines = mc.LineCollection(path_edges, colors='green', linewidths=3)
        ax.add_collection(path_lines)

    ax.scatter(rrt.start_position[0], rrt.start_position[1], c='black', linewidths=2)
    ax.scatter(rrt.end_position[0], rrt.end_position[1], c='black', linewidths=2)

    ax.set_aspect(1)
    plt.show()
