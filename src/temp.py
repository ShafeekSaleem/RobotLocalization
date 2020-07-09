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
import matplotlib.animation as animation

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


def animate(i):
    path = "D:/Others/Projects/VO-SLAM/101/image_"+str(i)+".jpg"
    img = cv2.imread(path)

    vo.trackImage(img, depth[i], gt_t[i], gt_r[i],i)
    if i>=1:
        x.append(vo.poses[-1][1])
        y.append(vo.poses[-1][0])

        x_t.append(vo.groundtruth_t[-1][1])
        y_t.append(vo.groundtruth_t[-1][0])
        plt.cla()

        plt.plot(x, y ,color='g',linewidth=2,label='MVO-SLAM Est')
        plt.plot(x_t, y_t,'--',color='b',linewidth=2,label='Ground Truth Est')
        plt.title('Pose Estimation ( With Dynamic alpha + Error threshold )')
        plt.xlabel('X-axis ( meter )')
        plt.ylabel('Z-axis ( meter )')
        plt.legend(loc='upper left')


anim = animation.FuncAnimation( plt.gcf(), animate, frames = 200, interval=10, repeat = False)

plt.tight_layout()
plt.show()


