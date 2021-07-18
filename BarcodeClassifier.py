#!/usr/bin/python3

import numpy as np
import cv2
import cv2.aruco as aruco

class BarcodeClassifier:
    def __init__(self):
        self._aruco_dict       = aruco.Dictionary_get(11)
        self._aruco_parameters = aruco.DetectorParameters_create()

        self._aruco_parameters.minDistanceToBorder      = 0
        self._aruco_parameters.adaptiveThreshWinSizeMax = 400

        self.observation = None
        self.corners     = None
        self.ids         = None
        self.rejected    = None

    def _detect_markers(self):
        """
        get marker id
        """
        if self.observation is None:
            raise AttributeError("observation is None")
        
        gray = cv2.cvtColor(self.observation, cv2.COLOR_BGR2GRAY)
        
        cv2.rectangle(gray, (10, 10), (100, 100), color=(255, 0, 0), thickness=3)
        
        self.corners, self.ids, self.rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_parameters)

        print(self.ids)

    def show_detected_markers(self):
        attributes_not_empty = [self.corners is not None, self.ids is not None, self.rejected is not None, self.observation is not None]
        if not all(attributes_not_empty):
            raise AttributeError("Some attributes are Nones")

        for (i, b) in enumerate(self.corners):
            print(i, b, self.ids[i])
            print("B0",  b[0])
            print("B00", b[0][0])
            print("B01", b[0][1])
            print("B02", b[0][2])

            c1 = (b[0][0][0], b[0][0][1])
            c2 = (b[0][1][0], b[0][1][1])
            c3 = (b[0][2][0], b[0][2][1])
            c4 = (b[0][3][0], b[0][3][1])
            
            print(c1)
            
            cv2.line(self.observation, c1, c2, (0, 0, 255), 3)
            cv2.line(self.observation, c2, c3, (0, 0, 255), 3)
            cv2.line(self.observation, c3, c4, (0, 0, 255), 3)
            cv2.line(self.observation, c4, c1, (0, 0, 255), 3)
            
            x = int((c1[0] + c2[0] + c3[0] + c4[0]) / 4)
            y = int((c1[1] + c2[1] + c3[1] + c4[1]) / 4)
            
            self.observation = cv2.putText(
                self.observation, str(self.ids[i]), (x, y), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 2, 
                cv2.LINE_AA)

            self.observation = cv2.putText(self.observation, "{0},{1}".format(int(c1[0]), int(c1[1])), 
                (c1[0], c1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 
            self.observation = cv2.putText(self.observation, "{0},{1}".format(int(c2[0]), int(c2[1])), 
                (c2[0], c2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 
            self.observation = cv2.putText(self.observation, "{0},{1}".format(int(c3[0]), int(c3[1])), 
                (c3[0], c3[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 
            self.observation = cv2.putText(self.observation, "{0},{1}".format(int(c4[0]), int(c4[1])), 
                (c4[0], c4[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 

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
