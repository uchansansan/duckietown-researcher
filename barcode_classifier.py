#!/usr/bin/python3
from enum import IntEnum

import numpy as np
import cv2
import cv2.aruco as aruco

class Direction(IntEnum):
    DOWN  = 0
    LEFT  = 1
    UP    = 2
    RIGHT = 3

    def __add__(self, a):
        return Direction( (a.value + self.value) % 4 )
    def __sub__(self, a):
        return Direction( (self.value - a.value) % 4 )

class BarcodeClassifier:
    PATHS = {
        1: (Direction.DOWN),

        5: (Direction.DOWN,
            Direction.LEFT,
            Direction.UP,
            Direction.RIGHT),

        8: (Direction.DOWN),

        9: (Direction.DOWN,
            Direction.UP,
            Direction.RIGHT),

        10:(Direction.DOWN,
            Direction.LEFT,
            Direction.UP),

        11:(Direction.DOWN,
            Direction.LEFT,
            Direction.RIGHT)
    }

    def __init__(self):
        self._aruco_dict       = aruco.Dictionary_get(20)
        self._aruco_parameters = aruco.DetectorParameters_create()
        self._aruco_parameters.minDistanceToBorder = 0

        self.show_markers = False
        self.show_lines   = False

        self.observation = None
        self.bbox        = None
        self.ids         = None
        self.rejected    = None

        self._gray = None
        self._hsv  = None

        self._hsv_red_bounds = ((np.array([0,   100, 100]), np.array([15,  255, 255])),
                                (np.array([165, 100, 100]), np.array([180, 255, 255])))
        self._dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._erode_kernel  = np.ones((3, 3), np.uint8)

        self.red_line_detected = False
        self.red_lines_pos = None


    def _aruco_show_detected_markers(self):        
        if self.observation is None:
            raise AttributeError("observation is None")
        if self.bbox is None:
            return
            # raise AttributeError("boundary box is None")

        aruco.drawDetectedMarkers(self.observation, self.bbox)

    def _aruco_detect_markers(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        
        self.bbox, self.ids, self.rejected = aruco.detectMarkers(self._gray, self._aruco_dict, parameters=self._aruco_parameters)

        # print(self.ids)


    def _cv_show_detected_redlines(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        if self.red_lines_pos is None:
            return

        if len(self.red_lines_pos) > 0:
            for line in self.red_lines_pos:
                for x1,y1,x2,y2 in line:
                    cv2.line(self.observation,   (x1,y1), (x2,y2), (0,     0, 255), 2)
                    cv2.circle(self.observation, (x1,y1),       2, (0,   255,   0))
                    cv2.circle(self.observation, (x2,y2),       2, (255,   0,   0))

    def _cv_redline_detection(self):
        if self._hsv is None:
            raise AttributeError("hsv filtered observation is None. Check convertion parametres")
        if self._gray is None:
            raise AttributeError("gray filtered observation is None. Check convertion parametres")
        
        red1 = cv2.inRange(self._hsv, *self._hsv_red_bounds[0])
        red2 = cv2.inRange(self._hsv, *self._hsv_red_bounds[1])
        red  = cv2.bitwise_or(red1, red2)
        red[:240,:] = 0
        red[:,-120:] = 0
        red[:,:120] = 0
        red  = cv2.dilate(red, self._dilate_kernel)

        # cv2.imshow("red_bitwise", red)

        edges = cv2.Canny(self._gray, 100, 350)
        # cv2.imshow("Canny", edges)

        edge_color_red=cv2.bitwise_and(edges, red)
        # cv2.imshow("red_lines", edge_color_red)

        self.red_lines_pos = cv2.HoughLinesP(edge_color_red, 1, np.pi/180, 10, np.empty(1), 1.5, 1)

        self.red_line_detected = False if self.red_lines_pos is None else True


    def show_observation(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        
        cv2.imshow("observation", self.observation)

    def get_paths(self, heading):
        """
        get crossroad directions
        """
        if self.ids is None:
            return []
        
        heading = Direction(int(np.floor((heading + 45)%360 / 90)))

        return [path - heading + Direction.UP for path in BarcodeClassifier.PATHS[max(self.ids[0])]]

    def update(self, observation):
        if observation is None:
            raise ValueError("observation is None")

        self.observation = observation
        self._hsv  = cv2.cvtColor(observation,  cv2.COLOR_BGR2HSV)
        self._gray = cv2.cvtColor(observation, cv2.COLOR_BGR2GRAY)

        self._cv_redline_detection()
        if self.red_line_detected:
            self._aruco_detect_markers()
        
        if self.show_lines:
            self._cv_show_detected_redlines()
        if self.show_markers:
            self._aruco_show_detected_markers()


if __name__ == "__main__":
    bc = BarcodeClassifier()
    bc.show_markers = True
    bc.show_lines   = True
    # cap = cv2.VideoCapture(2)
    
    # cv2.namedWindow('threshold')

    # cv2.createTrackbar('lowb', 'threshold', bc.thresh_lower_bound, 255, lambda:None)

    while True:
        # success, img = cap.read()

        img = cv2.imread("screen.png")

        # bc.thresh_lower_bound = cv2.getTrackbarPos('lowb','threshold')

        bc.update(img)

        bc.show_observation()

        print(*bc.get_paths(0))

        if cv2.waitKey(1) == 27:
            break
    
    cv2.destroyAllWindows()
