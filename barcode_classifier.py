#!/usr/bin/python3
from direction import Direction

import numpy as np
import cv2
import cv2.aruco as aruco

class BarcodeClassifier:
    PATHS = {
        1: tuple([Direction.DOWN]),

        8: (Direction.DOWN,
            Direction.LEFT,
            Direction.UP,
            Direction.RIGHT),

        5: tuple([Direction.DOWN]),

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
        self._aruco_dict       = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        self._aruco_parameters = aruco.DetectorParameters_create()
        self._aruco_parameters.minDistanceToBorder = 0

        self._dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._erode_kernel  = np.ones((3, 3), np.uint8)

        self._WINDOW_METRICS   = (640, 480)

        self._RED_WINDOW_LOWER = (120, 360-100)
        self._RED_WINDOW_BOUND = (
            (self._RED_WINDOW_LOWER[0], self._RED_WINDOW_LOWER[1]), 
            (self._WINDOW_METRICS[0] - self._RED_WINDOW_LOWER[0], self._WINDOW_METRICS[1]))
 
        self._HSV_RED_BOUNDS = (
            (np.array([0,   100, 100]), np.array([15,  255, 255])),
            (np.array([165, 100, 100]), np.array([180, 255, 255])))

        self._DETECTION_COOLDOWN = 10
        self._cooldown    = self._DETECTION_COOLDOWN
        self._on_cooldown = False

        self.show_markers = False
        self.show_lines   = False
        self.bbox_delete  = True

        self.observation = None
        self.bbox        = None
        self._ids        = []
        self.rejected    = None

        self._gray = None
        self._hsv  = None

        self.red_line_detected = False
        self.red_lines_pos     = None


    def _aruco_show_detected_markers(self):        
        if self.observation is None:
            raise AttributeError("observation is None")
        if self.bbox is None:
            return
            # raise AttributeError("boundary box is None")

        aruco.drawDetectedMarkers(self.observation, self.bbox)
        
        if self.bbox_delete:
            self.bbox = None

    def _aruco_detect_markers(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        
        self.bbox, ids, self.rejected = aruco.detectMarkers(self._gray, self._aruco_dict, parameters=self._aruco_parameters)

        self._ids.append(max(ids[0]))

        self._on_cooldown = True
        # print(self.ids)


    def _cv_show_detected_redlines(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        
        cv2.rectangle(self.observation, self._RED_WINDOW_BOUND[0], self._RED_WINDOW_BOUND[1], color=(255, 0, 0), thickness=1)
        
        if self.red_lines_pos is None:
            return

        if len(self.red_lines_pos) > 0:
            for line in self.red_lines_pos:
                for x1,y1,x2,y2 in line:
                    x_lower_bound = self._RED_WINDOW_BOUND[0][0]
                    y_lower_bound = self._RED_WINDOW_BOUND[0][1]
                    x1 += x_lower_bound
                    x2 += x_lower_bound
                    y1 += y_lower_bound
                    y2 += y_lower_bound
                    cv2.line(self.observation,   (x1,y1), (x2,y2), (0,     0, 255), thickness=2)
                    cv2.circle(self.observation, (x1,y1),       2, (0,   255,   0))
                    cv2.circle(self.observation, (x2,y2),       2, (255,   0,   0))

    def _cv_redline_detection(self):
        if self._hsv is None:
            raise AttributeError("hsv filtered observation is None. Check convertion parametres")
        if self._gray is None:
            raise AttributeError("gray filtered observation is None. Check convertion parametres")
        
        rwb = self._RED_WINDOW_BOUND

        hsv_windowed  = self._hsv[rwb[0][1]:rwb[1][1], rwb[0][0]:rwb[1][0]]
        # cv2.imshow("windowed", hsv_windowed)
        gray_windowed = self._gray[rwb[0][1]:rwb[1][1], rwb[0][0]:rwb[1][0]]

        red1 = cv2.inRange(hsv_windowed, *self._HSV_RED_BOUNDS[0])
        red2 = cv2.inRange(hsv_windowed, *self._HSV_RED_BOUNDS[1])
        red  = cv2.bitwise_or(red1, red2)
        red  = cv2.dilate(red, self._dilate_kernel)

        # cv2.imshow("red_bitwise", red)

        edges = cv2.Canny(gray_windowed, 100, 350)
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
        if len(self._ids) < 1:
            return []

        heading = Direction(int(np.floor((heading + 45) % 360 / 90)))

        return [path - heading for path in BarcodeClassifier.PATHS[self._ids.pop(0)]]

    def update(self, observation):
        if observation is None:
            raise ValueError("observation is None")

        self.observation = observation
        self._hsv  = cv2.cvtColor(observation,  cv2.COLOR_BGR2HSV)
        self._gray = cv2.cvtColor(observation, cv2.COLOR_BGR2GRAY)

        if self._cooldown < 1:
            self._on_cooldown = False
            self._cooldown = self._DETECTION_COOLDOWN
        if self._on_cooldown:
            self._cooldown -= 1
        
        self._cv_redline_detection()
        if self.red_line_detected:
            if not self._on_cooldown:
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

        img = cv2.imread("screen1.png")

        # bc.thresh_lower_bound = cv2.getTrackbarPos('lowb','threshold')

        bc.update(img)

        bc.show_observation()

        paths = bc.get_paths(0)
        if len(paths) > 0:
            print(*paths)

        if cv2.waitKey(1) == 27:
            break
    
    cv2.destroyAllWindows()
