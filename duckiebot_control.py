import numpy as np
from researcher.direction import Direction


class DuckiebotControl:
    k_p = 2.0
    k_r = 6.5
    k_d = 2.6
    k_a = 2.6
    v_0 = 0.24

    def __init__(self, frame_skip=1):
        self._DEFAULT_TICS = 90 / frame_skip

        self.distance = 0.0
        self.angle = 0.0
        self._previos_distance = 0.0
        self._previos_angle = 0.0
        self._state = 'hui'
        self._tics = self._DEFAULT_TICS

    def update(self, angle, distance):
        self.distance = distance - 0.05
        self.angle = angle
        # print(self._state)
        if self._state != 'hui' and self._tics >= 1:
            self._tics -= 1
        else:
            self._state = 'hui'
            self._tics = self._DEFAULT_TICS

    def get_action(self):
        """
        calculate next action
        """
        action = np.array([0, 0])
        if self._state == 'hui':
            regul = self.angle
            dx = self.distance - self._previos_distance
            da = self.angle - self._previos_angle
            v_l = self.v_0 - self.k_p * self.distance - regul * self.k_r - self.k_d * dx - self.k_a * da
            v_r = self.v_0 + self.k_p * self.distance + regul * self.k_r + self.k_d * dx + self.k_a * da

            #  action = np.array([ (v_R + v_L) / abs(20*regul)  + self.k_a*dA, (v_R - v_L) / 2 + regul*self.k_r*2.1 + self.k_d*dX])  #some magic with angle and rotation

            action = np.array([(v_r + v_l) / 2, (v_r - v_l) / 2])  # like PD in basic_control
            self._previos_distance = self.distance
            self._previos_angle = self.angle
        elif self._state == 'r':
            action = np.array([self.v_0, -1.])
        elif self._state == 'l':
            action = np.array([self.v_0, 1.])
        elif self._state == 'u':
            action = np.array([self.v_0, 0.])

        return action

    def way(self, direction):
        if direction == Direction.RIGHT:
            self._state = 'r'
        if direction == Direction.UP:
            self._state = 'u'
        if direction == Direction.LEFT:
            self._state = 'l'
