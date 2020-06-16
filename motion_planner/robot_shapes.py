from shapely.geometry import box
from abc import ABC


class Rectangle(object):
    def __init__(self, length, width):
        self._shape = box(-float(length) / 2, -float(width) / 2, float(length) / 2, float(width) / 2)

    def shape(self):
        return self._shape
