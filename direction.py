import numpy as np

from enum import IntEnum

class Direction(IntEnum):
    DOWN  = 0
    LEFT  = 1
    UP    = 2
    RIGHT = 3

    def direction_from_radians(rads):
        rads *= 180/np.pi
        return Direction(int(rads + 45)%360//90)
    def towards(self, direction):
        if not isinstance(direction, Direction):
            raise ValueError("direction meant to have a Direction type")
        return Direction( (self.value + direction.value + Direction.UP) % 4)
    def get_normalized(self):
        horizontal = self.value % 2  == 0
        positive   = self.value // 2 == 1
        if positive:
            return np.array([ 0,  1]) if horizontal else np.array([ 1,  0])
        else:
            return np.array([ 0, -1]) if horizontal else np.array([-1,  0])
    def invert(self):
        return Direction( (self.value + Direction.UP) % 4 )
    def __add__(self, a):
        return Direction( (a.value + self.value) % 4 )
    def __sub__(self, a):
        return Direction( (self.value - a.value) % 4 )
