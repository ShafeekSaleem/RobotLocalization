#!/usr/bin/env python

import numpy as np
import cv2
from feature_tracker import FeatureTrackerTypes, FeatureTracker, FeatureTrackingResult
from feature_orb2D import OrbFeature2D
from feature_matcher import feature_matcher_factory, FeatureMatcherTypes
from parameters import Parameters 
import random
from dataset import DatasetType, VideoDataset
kMinNumFeatureDefault = Parameters.kMinNumFeatureDefault
kRatioTest = Parameters.kFeatureMatchRatioTest

#draw matched points in respective image
def draw_points(image, points, color , is_save = True, path = "Images/result.jpg"):
	thickness = 2
	for point in points:
		image = cv2.circle(image, tuple(point), 1, color, thickness)
	if is_save:
		cv2.imwrite( path, image)
filename = "Videos/video.mp4"
data = VideoDataset(filename, DatasetType.VIDEO)
image= data.getImage(302)
print(data.getNextTimestamp())
print(data.Ts)
"""0
image_ref = cv2.imread("Images/image1.jpg")
image_cur = cv2.imread("Images/image2.jpg")

if image_ref is None:
        print("Image is none")

detector = OrbFeature2D(num_features=kMinNumFeatureDefault, scale_factor=1.2, num_levels=8)

kps_ref, des_ref = detector.detectAndCompute( image_ref )
featureTracker = FeatureTracker(num_features=kMinNumFeatureDefault, 
                       num_levels = 8,  
                       scale_factor = 1.2,   
                       match_ratio_test = kRatioTest, 
                       tracker_type = FeatureTrackerTypes.DES_BF)

res = featureTracker.track(image_ref, image_cur, kps_ref, des_ref)
#draw_points(image_ref, res.kps_ref_matched, (255, 0, 0))
#draw_points(image_cur, res.kps_cur_matched, (0, 255, 0))
print(res.kps_ref_matched)
"""






