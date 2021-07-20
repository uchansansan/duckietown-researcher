#!/usr/bin/python3

import numpy as np
import cv2
import cv2.aruco as aruco

class BarcodeClassifier:
    def __init__(self):
        self._aruco_dict       = aruco.Dictionary_get(8)
        self._aruco_parameters = aruco.DetectorParameters_create()

        self._aruco_parameters.minDistanceToBorder      = 0
        # self._aruco_parameters.adaptiveThreshWinSizeMax = 400

        self.thresh_lower_bound = 10

        self.observation = None
        self.bbox        = None
        self.ids         = None
        self.rejected    = None

    def _detect_markers(self):
        """
        get marker id
        """
        if self.observation is None:
            raise AttributeError("observation is None")
        
        gray = cv2.cvtColor(self.observation, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, self.thresh_lower_bound, 255, cv2.THRESH_BINARY)

        cv2.imshow("threshold", threshold)
        
        self.bbox, self.ids, self.rejected = aruco.detectMarkers(threshold, self._aruco_dict, parameters=self._aruco_parameters)

        print(self.ids)

    def show_observation(self):
        cv2.imshow("observation", self.observation)

    def show_detected_markers(self):
        attributes_not_empty = [self.bbox is not None, self.observation is not None]
        
        if not all(attributes_not_empty):
            raise AttributeError("Some neccesery attributes are Nones")

        aruco.drawDetectedMarkers(self.observation, self.bbox)

    def get_paths(self):
        """
        get crossroad directions
        """
        pass

    def update(self, observation):
        """
        """
        self.observation = observation
        self._detect_markers()

if __name__ == "__main__":
    bc = BarcodeClassifier()
    
    cap = cv2.VideoCapture(0)
    
    cv2.namedWindow('capture')

    # cv2.createTrackbar('lowb', 'capture', 0, 255, lambda:None)

    while True:
        # success, img = cap.read()

        img = cv2.imread("screen.png")

        # bc.thresh_lower_bound = cv2.getTrackbarPos('lowb','capture')

        bc.update(img)

        bc.show_detected_markers()

        bc.show_observation()

        if cv2.waitKey(1) == 27:
            break
    
    cv2.destroyAllWindows()
