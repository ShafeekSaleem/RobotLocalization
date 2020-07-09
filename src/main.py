import numpy as np
import cv2
import time
from feature_tracker import FeatureTrackerTypes, FeatureTracker, FeatureTrackingResult
from feature_orb2D import OrbFeature2D
from feature_matcher import feature_matcher_factory, FeatureMatcherTypes
from parameters import Parameters 
import random
import tracemalloc
import linecache
from visual_odometry import ImageRecievedState, VisualOdometry
from camera import KinectCamera
from utils import compute_euler_angle
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from itertools import count
from matplotlib.animation import FuncAnimation

MinNumFeatureDefault = Parameters.MinNumFeatureDefault
RatioTest = Parameters.FeatureMatchRatioTest
gt_t = np.load("D:/Others/Projects/VO-SLAM/101/0/translations.npy")
gt_r = np.load("D:/Others/Projects/VO-SLAM/101/0/rotationsM.npy")
depth = np.load("D:/Others/Projects/VO-SLAM/101/0/depth.npy")

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

vo = VisualOdometry(cam, feature_tracker, True, True)

"""
Live plot of the Data using Matplotlib animation
"""

plt.style.use('seaborn')
x = []
y = []

x_t = []
y_t = []

fig, (ax0,ax1) = plt.subplots(1,2,gridspec_kw={'width_ratios': [2, 1]})

image0 = cv2.imread("D:/Others/Projects/VO-SLAM/101/image_0.jpg")
#create image axes
im1 = ax1.imshow(image0)

def animate(i):
    path = "D:/Others/Projects/VO-SLAM/101/image_"+str(i)+".jpg"
    img = cv2.imread(path)
    im1.set_data(img)
    vo.trackImage(img, depth[i], gt_t[i], gt_r[i],i)
    if i>=1:
        x.append(vo.poses[-1][1])
        y.append(vo.poses[-1][0])

        x_t.append(vo.groundtruth_t[-1][1])
        y_t.append(vo.groundtruth_t[-1][0])
        ax0.cla()

        ax0.plot(x, y ,label='VO+MO Est')
        ax0.plot(x_t, y_t, '--',label='Ground Truth - MO')
        ax0.set_title('Pose Estimation')
        ax0.set_xlabel('X-axis')
        ax0.set_ylabel('Y-axis')
        ax0.legend(loc='upper left')

        ax1.set_title('Frames')
        ax1.grid(False)
        plt.tight_layout()

anim = FuncAnimation( plt.gcf(), animate, frames = 500, interval=50, repeat = False )

plt.tight_layout()
plt.show()
 

