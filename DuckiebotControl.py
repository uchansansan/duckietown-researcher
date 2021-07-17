import numpy as np

class DuckiebotControl:
    def __init__(self):
        self.__stop_line_found = False

        self.observation = None
        self.distance    = 0.0
        self.angle       = 0.0
    
    def __find_stop_line(self):
        """
        find stop line on observation
        and update stop_line_found
        """
        self.__stop_line_found = False

    def on_stop_line(self):
        return self.__stop_line_found

    def update(self, observation, angle, distance):
        self.observation = observation
        self.distance    = distance
        self.angle       = angle

        self.__stop_line_found()

    def PID_action(self):
        """
        calculate next action
        """
        action = np.array([0.0, 0.0])
        return action