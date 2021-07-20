#!/usr/bin/python3

import numpy as np
import cv2
import cv2.aruco as aruco

class BarcodeClassifier:
    def __init__(self):
        self._aruco_dict       = aruco.Dictionary_get(20)

        self._aruco_parameters = aruco.DetectorParameters_create()

        self._aruco_parameters.minDistanceToBorder      = 0

        self.thresh_lower_bound = 80

        self.observation = None
        self.bbox        = None
        self.ids         = None
        self.rejected    = None


    def _aruco_detect_markers(self):
        """
        get marker id
        """
        if self.observation is None:
            raise AttributeError("observation is None")
        
        gray = cv2.cvtColor(self.observation, cv2.COLOR_BGR2GRAY)
        
        self.bbox, self.ids, self.rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_parameters)

    def _aruco_show_detected_markers(self):
        attributes_not_empty = [self.bbox is not None, self.observation is not None]
        
        if not all(attributes_not_empty):
            raise AttributeError("Some neccesery attributes are Nones")

        aruco.drawDetectedMarkers(self.observation, self.bbox)

    def show_observation(self):
        if self.observation is None:
            raise AttributeError("observation is None")
        
            cv2.imshow("observation", self.observation)

    def get_paths(self):
        """
        get crossroad directions
        """
        pass

    def update(self, observation, show_markers=False):
        """
        """
        self.observation = observation
        self._aruco_detect_markers()
        if show_markers:
            self._aruco_show_detected_markers()

if __name__ == "__main__":
    bc = BarcodeClassifier()
    
    cap = cv2.VideoCapture(2)
    
    # cv2.namedWindow('threshold')

    # cv2.createTrackbar('lowb', 'threshold', bc.thresh_lower_bound, 255, lambda:None)

    while True:
        # success, img = cap.read()

        img = cv2.imread("screen.png")

        # bc.thresh_lower_bound = cv2.getTrackbarPos('lowb','threshold')

        bc.update(img, show_markers=True)

        bc.show_observation()

        if cv2.waitKey(1) == 27:
            break
    
    cv2.destroyAllWindows()
