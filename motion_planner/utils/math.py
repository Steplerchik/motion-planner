import numpy as np


def point_coordinates(point):
    local_point = np.array(point)
    dimension = local_point.shape[-1]
    local_x, local_y, local_angle = 0, 0, 0
    if dimension == 3:
        local_x, local_y, local_angle = local_point.T
    elif dimension == 2:
        local_x, local_y = local_point.T
    return local_x, local_y, local_angle, dimension


def local2global(local_point, source_point):
    local_point = np.array(local_point)
    source_point = np.array(source_point)
    local_x, local_y, local_angle, dimension = point_coordinates(local_point)
    source_x, source_y, source_angle = source_point.T
    global_x = local_x * np.cos(source_angle) - local_y * np.sin(source_angle) + source_x
    global_y = local_x * np.sin(source_angle) + local_y * np.cos(source_angle) + source_y
    global_angle = (local_angle + source_angle + np.pi) % (2 * np.pi) - np.pi
    if dimension == 3:
        return np.array([global_x, global_y, global_angle]).T
    elif dimension == 2:
        return np.array([global_x, global_y]).T
    else:
        return


def global2local(global_point, source_point):
    global_point = np.array(global_point)
    source_point = np.array(source_point)
    global_x, global_y, global_angle, dimension = point_coordinates(global_point)
    source_x, source_y, source_angle = source_point.T
    local_x = global_x * np.cos(source_angle) + global_y * np.sin(source_angle) - source_x * np.cos(
        source_angle) - source_y * np.sin(source_angle)
    local_y = -global_x * np.sin(source_angle) + global_y * np.cos(source_angle) + source_x * np.sin(
        source_angle) - source_y * np.cos(source_angle)
    local_angle = (global_angle - source_angle + np.pi) % (2 * np.pi) - np.pi
    if dimension == 3:
        return np.array([local_x, local_y, local_angle]).T
    elif dimension == 2:
        return np.array([local_x, local_y]).T
    else:
        return


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def remove(list_of_arrays, array):
    index = 0
    size = len(list_of_arrays)
    while index != size and not np.array_equal(list_of_arrays[index], array):
        index += 1
    if index != size:
        list_of_arrays.pop(index)
