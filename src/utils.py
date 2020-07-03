import numpy as np
import math
import random

# [[x,y]] -> [[x,y,1]]
def add_ones(x):
    if len(x.shape) == 1:
        return np.array([x[0], x[1], 1])
    else:
        return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)

# turn rotation matrix R (3x3) into euler angles [a,b,c]
"""
R:
    ⎜r11 r12 r13
     r21 r22 r23
     r31 r32 r33⎞⎟
"""
def compute_euler_angle(R):
    if R.shape[0] != 3 or R.shape[1] != 3:
        print("Invalid shape for matrix R")
        return None
    a = math.atan2(-1*R[1,2], R[2,2])
    b = math.atan2(R[0,2], math.sqrt(R[1,2]**2+R[2,2]**2))
    c = math.atan2(-1*R[0,1], R[0,0])
    return np.array([a,b,c])

# [4x4] homogeneous T from [3x3] R and [3x1] t             
def poseRt(R, t):
    ret = np.eye(4)
    ret[:3, :3] = R
    ret[:3, 3] = t
    return ret  
# [N, x,y] -> [N, y, x]
def switch_xy(xy):
    yx = []
    for i in xy:
        yx.append([i[1],i[0]])
    return np.array(yx)

# remove rows  in an array given indexes
def removeByIndexes( arr, indexes ):
    return np.array(np.delete(arr,indexes, axis=0),np.float32)
