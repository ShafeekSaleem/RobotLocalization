#!/usr/bin/env python

import numpy as np 
import cv2 
from cv2 import ORB_create

class OrbFeature2D:
    def __init__(self, num_features=2000, scale_factor=1.2, num_levels=8):
        print('Using Orb Feature 2D')
        self.orb_extractor = ORB_create(num_features, scale_factor, num_levels)

    # extract keypoints 
    def detect(self, img):  
        # detect 
        kps_tuples = self.orb_extractor.detect(img)            
        # convert keypoints 
        kps = [cv2.KeyPoint(*kp) for kp in kps_tuples]
        return kps
  
    def detectAndCompute(self, img):
	#detect and compute
        kps, des = self.orb_extractor.detectAndCompute(img, None) 
        return kps, des   

