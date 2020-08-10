# MVO-SLAM | Robot Localization 
MVO-SLAM contains a python implementation of a monocular SLAM algorithm using Visual Odometry(VO) + Motion Odometry pipeline. This MVO-SLAM package was developed to run in as a [ros](http://wiki.ros.org/) package. And also it can be run in other environments without ROS.

## Abstract idea: 
![Abstract Design](https://github.com/ShafeekSaleem/RobotLocalization/blob/master/src/Images/abstract.jpg)

### Visual Odometry Pipeline
![VO pipeline](https://github.com/ShafeekSaleem/RobotLocalization/blob/master/src/Images/vo.jpg)

For each frame, key-points and descriptors are detected using [ORB](https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html) and matched with the previous frame using [FLANN](https://docs.opencv.org/2.4/modules/flann/doc/flann_fast_approximate_nearest_neighbor_search.html) feature matcher. Using these keypoints, and the 3d points from the depth image, 6 DoF pose is estimated using Perspective-n-Point algorithm with RANSAC.

![Image of Matched Keypoints using FLANN feature matcher](https://github.com/ShafeekSaleem/RobotLocalization/blob/master/src/Images/flann.png)

At each step **t**, main_vo.py estimates the current camera pose **C_t** with respect to the previous one **C_t**. For this, The inter-frame pose estimation returns **R_{k-1,k},t_{k-1,k}** with **||t_{k-1,k}||=1**. With this very basic approach, a ground truth is used in order to recover a correct inter-frame scale **s** and estimate a valid trajectory by composing **C_k = C_{k-1} * [R_{k-1,k}, s t_{k-1,k}]**. This script is a first start to understand the basics of inter-frame feature tracking and camera pose estimation.

## Error refinement
At each frame, initial pose will be set to the estimation from motion odometry. Then, visual odometry estimation will be calculated and the error between these two estimations will be measured. Then the error will be added to the initial estimation scaled by the alpha parameter. The alpha parameter decides how much of the information from visual odometry will be added to the initial estimation.
![Error refinement](https://github.com/ShafeekSaleem/RobotLocalization/blob/master/src/Images/illus.jpg)

## feature_orb2D.py:
Given an image, detects features and returns the detected key points and respective descriptors.

## feature_matcher.py
Does inter frame feature matching using either FLANN or Brute Force ([BF](https://docs.opencv.org/3.4/d3/da1/classcv_1_1BFMatcher.html)) algorithm. 

## feature_tracker.py
Using both feature_orb2D.py and feature_matcher.py, implements inter frame feature tracking and keeps track of the parameters needed for the next phase.
