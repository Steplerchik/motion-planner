from motion_planner import *


class SpaceInfo(object):
    def __init__(self, robot_shape=Rectangle(0, 0), alpha=0.0):
        self.state = SE2(alpha)
        self.collision = CollisionChecker(robot_shape)
