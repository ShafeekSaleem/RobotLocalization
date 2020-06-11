#!/usr/bin/env python

import numpy as np 
import cv2
from enum import Enum

from feature_matcher import feature_matcher_factory, FeatureMatcherTypes
from feature_orb2D import OrbFeature2D
from parameters import Parameters 


MinNumFeatureDefault = Parameters.MinNumFeatureDefault
RatioTest = Parameters.FeatureMatchRatioTest

class FeatureTrackerTypes(Enum):
    DES_BF    = 0   # descriptor-based, brute force matching with knn 
    DES_FLANN = 1   # descriptor-based, FLANN-based matching 


class FeatureTrackingResult(object): 
    def __init__(self):
        self.kps_ref = None          # all reference keypoints (numpy array Nx2)
        self.kps_cur = None          # all current keypoints   (numpy array Nx2)
        self.des_cur = None          # all current descriptors (numpy array NxD)
        self.idxs_ref = None         # indexes of matches in kps_ref
        self.idxs_cur = None         # indexes of matches in kps_cur 
        self.kps_ref_matched = None  # reference matched keypoints
        self.kps_cur_matched = None  # current matched keypoints

# Base class 
""" Extract features by using ORB, match keypoints by using desired matcher on computed descriptors """

class FeatureTracker(object): 
    def __init__(self, num_features=MinNumFeatureDefault, 
                       num_levels = 8,  
                       scale_factor = 1.2,   
                       match_ratio_test = RatioTest, 
                       tracker_type = FeatureTrackerTypes.DES_BF):

        if tracker_type == FeatureTrackerTypes.DES_FLANN:
            self.matching_algo = FeatureMatcherTypes.FLANN
        elif tracker_type == FeatureTrackerTypes.DES_BF:
            self.matching_algo = FeatureMatcherTypes.BF
        else:
            raise ValueError()

        # init orb detector
        self.detector = OrbFeature2D(num_features=num_features, scale_factor=scale_factor, num_levels=num_levels)
	# init matcher 
        self.matcher = feature_matcher_factory(norm_type=cv2.NORM_HAMMING, ratio_test=match_ratio_test, type=self.matching_algo)        


    # out: keypoints and descriptors 
    def detectAndCompute(self, frame):
        return self.detector.detectAndCompute(frame) 


    # out: FeatureTrackingResult()
    def track(self, image_ref, image_cur, kps_ref, des_ref):
        kps_cur, des_cur = self.detectAndCompute(image_cur)
        # convert from list of keypoints to an array of points
        #kps_ref = np.array([x.pt for x in kps_ref], dtype=np.float32)
        kps_cur = np.array([x.pt for x in kps_cur], dtype=np.float32) 
        idxs_ref, idxs_cur = self.matcher.match(des_ref, des_cur)
        res = FeatureTrackingResult()
        res.kps_ref = kps_ref  # all the reference keypoints  
        res.kps_cur = kps_cur  # all the current keypoints       
        res.des_cur = des_cur  # all the current descriptors         
        res.kps_ref_matched = np.int32([kps_ref[pts] for pts in idxs_ref])
        res.idxs_ref = np.asarray(idxs_ref)                  
        res.kps_cur_matched = np.int32([kps_cur[pts] for pts in idxs_cur])
        res.idxs_cur = np.asarray(idxs_cur)
        return res                


