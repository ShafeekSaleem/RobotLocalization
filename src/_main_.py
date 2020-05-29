#!/usr/bin/env python

import numpy as np
import cv2
from feature_tracker import FeatureTrackerTypes, FeatureTracker, FeatureTrackingResult
from feature_orb2D import OrbFeature2D
from feature_matcher import feature_matcher_factory, FeatureMatcherTypes
from parameters import Parameters 


kMinNumFeatureDefault = Parameters.kMinNumFeatureDefault
kRatioTest = Parameters.kFeatureMatchRatioTest

image_ref = cv2.imread("/home/dexter/git/my_git/RobotLocalization/src/Images/image1.jpg")
image_cur = cv2.imread("/home/dexter/git/my_git/RobotLocalization/src/Images/image1.jpg")

detector = OrbFeature2D(num_features=kMinNumFeatureDefault, scale_factor=1.2, num_levels=8)

kps_ref, des_ref = detector.detectAndCompute( image_ref )

featureTracker = FeatureTracker(num_features=kMinNumFeatureDefault, 
                       num_levels = 8,  
                       scale_factor = 1.2,   
                       match_ratio_test = kRatioTest, 
                       tracker_type = FeatureTrackerTypes.DES_BF)

res = featureTracker.track(image_ref, image_cur, kps_ref, des_ref)
print(res.kps_cur)
print(res.des_cur)






