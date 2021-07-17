#!/usr/bin/python3

import numpy as np
import cv2
import cv2.aruco as aruco

class BarcodeClassifier:
    def __init__(self):
        self.__aruco_dict = aruco.Dictionary_get(aruco)

        self.observation = None

    def __detect_markers(self):
        pass

    def get_paths(self):
        pass

    def update(self, observation):
        self.observation = observation