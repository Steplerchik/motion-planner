from motion_planner.data.robot_shapes import Rectangle


class CollisionChecker(object):
    def __init__(self, robot_shape=Rectangle(0, 0)):
        self._robot_shape = robot_shape

    def __getattr__(self, called_method):
        return getattr(self._robot_shape, called_method)

