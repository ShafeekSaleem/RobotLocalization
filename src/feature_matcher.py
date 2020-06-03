#!/usr/bin/env python
import numpy as np 
import cv2
from parameters import Parameters  
from enum import Enum
from collections import defaultdict

RatioTest = Parameters.FeatureMatchRatioTest

class FeatureMatcherTypes(Enum):
    NONE = 0
    BF = 1     
    FLANN = 2        
          
def feature_matcher_factory(norm_type=cv2.NORM_HAMMING, cross_check=False, 		ratio_test=RatioTest, type=FeatureMatcherTypes.BF):
    if type == FeatureMatcherTypes.BF:
        return BfFeatureMatcher(norm_type=norm_type, cross_check=cross_check, ratio_test=ratio_test, type=type)
    if type == FeatureMatcherTypes.FLANN:
        return FlannFeatureMatcher(norm_type=norm_type, cross_check=cross_check, ratio_test=ratio_test, type=type)
    return None 

"""
N.B.: 
The result of matches = matcher.knnMatch() is a list of cv2.DMatch objects. 
A DMatch object has the following attributes:
    DMatch.distance - Distance between descriptors. The lower, the better it is.
    DMatch.trainIdx - Index of the descriptor in train descriptors
    DMatch.queryIdx - Index of the descriptor in query descriptors
    DMatch.imgIdx - Index of the train image.
""" 
# base class 
class FeatureMatcher(object): 
    def __init__(self, norm_type=cv2.NORM_HAMMING, cross_check = False, ratio_test=RatioTest, type = FeatureMatcherTypes.BF):
        self.type = type 
        self.norm_type = norm_type 
        self.cross_check = cross_check   # apply cross check 
        self.matches = []
        self.ratio_test = ratio_test 
        self.matcher = None 
        self.matcher_name = ''
        
        
    # input: des1 = queryDescriptors, des2= trainDescriptors
    # output: idx1, idx2  (vectors of corresponding indexes in des1 and des2, respectively)
    
    def match(self, des1, des2, ratio_test=None):                 
        matches = self.matcher.knnMatch(des1, des2, k=2)  
        self.matches = matches
        return self.goodMatches(matches, des1, des2, ratio_test) 
        
          
    # input: des1 = query-descriptors, des2 = train-descriptors
    # output: idx1, idx2  (vectors of corresponding indexes in des1 and des2, respectively)
   
    def goodMatches(self, matches, des1, des2, ratio_test=None):
        len_des2 = len(des2)
        idx1, idx2 = [], []  
        # good_matches = []           
        if ratio_test is None: 
            ratio_test = self.ratio_test
        if matches is not None:         
            float_inf = float('inf')
            dist_match = defaultdict(lambda: float_inf)   
            index_match = dict()  
            for m, n in matches:
                if m.distance > ratio_test * n.distance:
                    continue     
                dist = dist_match[m.trainIdx]
                if dist == float_inf: 
                    # trainIdx has not been matched yet
                    dist_match[m.trainIdx] = m.distance
                    idx1.append(m.queryIdx)
                    idx2.append(m.trainIdx)
                    index_match[m.trainIdx] = len(idx2)-1
                else:
                    if m.distance < dist: 
                        # we have already a match for trainIdx
                        index = index_match[m.trainIdx]
                        assert(idx2[index] == m.trainIdx) 
                        idx1[index]=m.queryIdx
                        idx2[index]=m.trainIdx                        
        return idx1, idx2


# Brute-Force Matcher 
class BfFeatureMatcher(FeatureMatcher): 
    def __init__(self, norm_type=cv2.NORM_HAMMING, cross_check = False, ratio_test=RatioTest, type = FeatureMatcherTypes.BF):
        super(BfFeatureMatcher, self).__init__(norm_type=norm_type, cross_check=cross_check, ratio_test=ratio_test, type=type)
        self.matcher = cv2.BFMatcher(norm_type, cross_check)     
        self.matcher_name = 'BfFeatureMatcher'   


# Flann Matcher 
class FlannFeatureMatcher(FeatureMatcher): 
    def __init__(self, norm_type=cv2.NORM_HAMMING, cross_check = False, ratio_test=RatioTest, type = FeatureMatcherTypes.FLANN):
        super(FlannFeatureMatcher, self).__init__(norm_type=norm_type, cross_check=cross_check, ratio_test=ratio_test, type=type)
        if norm_type == cv2.NORM_HAMMING:
            # FLANN parameters for binary descriptors 
            FLANN_INDEX_LSH = 6
            self.index_params= dict(algorithm = FLANN_INDEX_LSH,   # Multi-Probe LSH: Efficient Indexing for High-Dimensional Similarity Search
                        table_number = 6,      # 12
                        key_size = 12,         # 20
                        multi_probe_level = 1) # 2            
        if norm_type == cv2.NORM_L2: 
            # FLANN parameters for float descriptors 
            FLANN_INDEX_KDTREE = 1
            self.index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 4)  
        self.search_params = dict(checks=32)   # or pass empty dictionary                 
        self.matcher = cv2.FlannBasedMatcher(self.index_params, self.search_params)  
        self.matcher_name = 'FlannFeatureMatcher'

