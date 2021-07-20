import numpy as np


class DuckiebotControl:
    k_p = 2.1
    k_r = 6.0
    k_d = 2.0
    k_a = 2.0
    v_0 = 0.16
    def __init__(self):
        self.distance = 0.0
        self.angle = 0.0
        self._previos_distance = 0.0
        self._previos_angle = 0.0
    def on_stop_line(self):
        return self.__stop_line_found

    def update(self, angle, distance):
        self.distance = distance
        self.angle = angle

    def PID_action(self):
        """
        calculate next action
        """

        regul = self.angle
        dX = self.distance - self._previos_distance
        dA = self.angle - self._previos_angle
        v_L = self.v_0 - self.k_p * self.distance
        v_R = self.v_0 + self.k_p * self.distance

        action = np.array([ (v_R + v_L) / abs(20*regul)  + self.k_a*dA, (v_R - v_L) / 2 + regul*self.k_r*2.1 + self.k_d*dX])

        self._previos_distance = self.distance
        self._previos_angle = self.angle

        return action
