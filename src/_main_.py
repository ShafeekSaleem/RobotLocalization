#!/usr/bin/env python

import numpy as np
import cv2
import time
from feature_tracker import FeatureTrackerTypes, FeatureTracker, FeatureTrackingResult
from feature_orb2D import OrbFeature2D
from feature_matcher import feature_matcher_factory, FeatureMatcherTypes
from parameters import Parameters 
import random
#import tracemalloc
import linecache
#from visual_odometry import ImageRecievedState, VisualOdometry
#from camera import KinectCamera
#from utils import compute_euler_angle

import rospy
from dataset import DatasetType, VideoDataset, LiveStream


rgb_topic = "/camera/rgb/image_raw"
depth_topic = "/camera/depth/image_raw"
odom_topic = "/odom"
Live = LiveStream(rgb_topic, depth_topic , odom_topic,  type=DatasetType.LIVE)
translations = []
rotationsE = []
rotationsM = []
depth = []


def init_node():
    global translations, rotations
    rospy.init_node('read_image', anonymous=True)
    Live.synchronize()
    time.sleep(5)
    print("start moving you bitch!")
    for i in range(50):
	translations.append(Live.translation)
	rotationsE.append(Live.rotationEuler)
	rotationsM.append(Live.rotationMatrix)
	if Live.rgb_image is not None:
	   cv2.imwrite("/home/dexter/Pictures/101/image_"+str(i)+".jpg",Live.rgb_image)
	print(i)
	time.sleep(0.4)
    np.save("/home/dexter/Pictures/101/0/translations.npy", translations)
    np.save("/home/dexter/Pictures/101/0/rotationsE.npy", rotationsE)
    np.save("/home/dexter/Pictures/101/0/rotationsM.npy", rotationsM)
    while not rospy.is_shutdown():
    	rospy.spin()


init_node()










"""
MinNumFeatureDefault = Parameters.MinNumFeatureDefault
RatioTest = Parameters.FeatureMatchRatioTest


image_ref = cv2.imread("Images/image1.jpg")
image_cur = cv2.imread("Images/image2.jpg")

width  = 640
height = 480
fx     = 554.254691191187
fy     = 554.254691191187
cx     = 320.5
cy     = 240.5
D      = [0.0, 0.0, 0.0, 0.0, 0.0] 	
cam = KinectCamera(width, height, fx, fy, cx, cy, D)

feature_tracker = FeatureTracker(num_features=MinNumFeatureDefault, 
                       num_levels = 8,  
                       scale_factor = 1.2,   
                       match_ratio_test = RatioTest, 
                       tracker_type = FeatureTrackerTypes.DES_BF)
detector = OrbFeature2D(num_features=MinNumFeatureDefault, scale_factor=1.2, num_levels=8)
kps_ref, des_ref = detector.detectAndCompute( image_ref )

res = feature_tracker.track(image_ref, image_cur, kps_ref, des_ref)


vo = VisualOdometry(cam, feature_tracker)

r,t = vo.estimatePose(res.kps_ref_matched, res.kps_cur_matched)
r_e = compute_euler_angle(r)
print("R: ",r)
print("Euler angles: ",r_e)
print("T: ",t)






#draw matched points in respective image

def draw_points(image, points, color , is_save = True, path = "Images/result.jpg"):
	thickness = 2
	for point in points:
		image = cv2.circle(image, tuple(point), 1, color, thickness)
	if is_save:
		cv2.imwrite( path, image)

tracemalloc.start()

current, peak = tracemalloc.get_traced_memory()

print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
tracemalloc.stop()

def display_top(snapshot, key_type='lineno', limit=10):
    snapshot = snapshot.filter_traces((
        tracemalloc.Filter(False, "<frozen importlib._bootstrap>"),
        tracemalloc.Filter(False, "<unknown>"),
    ))
    top_stats = snapshot.statistics(key_type)

    print("Top %s lines" % limit)
    for index, stat in enumerate(top_stats[:limit], 1):
        frame = stat.traceback[0]
        print("#%s: %s:%s: %.1f KiB"
              % (index, frame.filename, frame.lineno, stat.size / 1024))
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print('    %s' % line)

    other = top_stats[limit:]
    if other:
        size = sum(stat.size for stat in other)
        print("%s other: %.1f KiB" % (len(other), size / 1024))
    total = sum(stat.size for stat in top_stats)
    print("Total allocated size: %.1f KiB" % (total / 1024))

tracemalloc.start()
# ... run your application ...
snapshot = tracemalloc.take_snapshot()
display_top(snapshot)
tracemalloc.stop()
"""


