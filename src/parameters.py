#!/usr/bin/env python

'''
List of shared parameters 
''' 
class Parameters(object):   

    # Number of desired keypoints per frame 
    NumFeatures=2000
    MinNumFeatureDefault = 2000
    # ratio test used by Initializer      
    FeatureMatchRatioTest = 0.7
    
    RansacThresholdNormalized = 0.0003  # metric threshold used for normalized image coordinates 
    RansacThresholdPixels = 0.1         # pixel threshold used for image coordinates 
    AbsoluteScaleThreshold = 0.1        # absolute translation scale; it is also the minimum translation norm for an accepted motion 
    RansacProb = 0.999

    # Alpha and error threshold
    Alpha = 0.5
    ErrorThreshold = 0.3
   


