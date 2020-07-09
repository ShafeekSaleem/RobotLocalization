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

errors = []
frames = []

fig, axs = plt.subplots(ncols=2, nrows=2,gridspec_kw={
                           'width_ratios': [1, 2],
                           'height_ratios': [2, 1]})
gs = axs[0, 0].get_gridspec()
# remove the underlying axes
for ax in axs[0:, -1]:
    ax.remove()
axbig = fig.add_subplot(gs[0:, -1])


image0 = cv2.imread("D:/Others/Projects/VO-SLAM/101/image_0.jpg")
#create image axes
im1 = axs[0][0].imshow(image0)

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
        
        errors.append(vo.errors[-1])
        frames.append(i)
        
        axbig.cla()

        axbig.plot(x, y ,color='g',linewidth=2,label='MVO-SLAM Est')
        axbig.plot(x_t, y_t,'--',color='b',linewidth=2,label='Ground Truth Est')
        axbig.set_title('Pose Estimation ( With Dynamic alpha + Error threshold )')
        axbig.set_xlabel('X-axis ( meter )')
        axbig.set_ylabel('Z-axis ( meter )')
        axbig.legend(loc='upper left')

        axs[1][0].plot(frames, errors,color='r',label='Error')
        axs[1][0].set_xlabel('Frame')
        axs[1][0].set_ylabel('Error ( meter )')
        axs[1][0].set_ylim([0,0.1])


        axs[0][0].set_title('Frames')
        axs[0][0].grid(False)
        axs[0][0].set_yticklabels([])
        axs[0][0].set_xticklabels([])
        plt.tight_layout()

anim = FuncAnimation( plt.gcf(), animate, frames = 500, interval=50,repeat = False )

plt.tight_layout()
plt.show()
 
