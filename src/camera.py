import numpy as np 
import cv2
from utils import add_ones
import math

"""
fx, fy - focal length of the camera in pixel coordinates
cx, cy - principle point
D      - distortion coefficeints , D = [k1, k2, p1, p2, k3]

by subscribing to the camera/depth/camera_info ros-topic, following parameters are obtained:
	width  -> 640
	height -> 480
	K      -> [554.254691191187, 0.0,              320.5, 
		   0.0,              554.254691191187, 240.5,
		   0.0,              0.0,              1.0]
	fx     -> 554.254691191187
	fy     -> 554.254691191187
	cx     -> 320.5
	cy     -> 240.5
	D      -> [0.0, 0.0, 0.0, 0.0, 0.0] 	

distortion_model: "plumb_bob"
simple model of radial and tangential distortion
"""

class Camera: 
    def __init__(self, width, height, fx, fy, cx, cy, D, fps = 1):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.D = np.array(D,dtype=np.float32) # np.array([k1, k2, p1, p2, k3])  distortion coefficients 
        self.fps = fps 
        
        self.is_distorted = np.linalg.norm(self.D) > 1e-10



class KinectCamera(Camera):
    def __init__(self, width, height, fx, fy, cx, cy, D, fps = 1):
        super(KinectCamera, self).__init__(width, height, fx, fy, cx, cy, D, fps)
        # K : intrinsic matrix of the camera
        self.K = np.array([[fx, 0,cx],
                           [ 0,fy,cy],
                           [ 0, 0, 1]])
        self.K_inverted = np.array([[1/fx,    0,-cx/fx],
                              [   0, 1/fy,-cy/fy],
                              [   0,    0,    1]])             
        
        self.u_min, self.u_max = 0, self.width 
        self.v_min, self.v_max = 0, self.height       

    # turn [u,v] -> [x,y,1] -> [x,y] : where x,y are in normalized coordinate system and u,v in pixel coordinate system    
    def unproject_points(self, uvs):
        return np.dot(self.K_inverted, add_ones(uvs).T).T[:, 0:2]

    # turn [u,v] -> [x,y,z] : Z contains the depth values for each pixels
    def unproject_points_z(self, uvs, Z):
          xyzs = []
          for idx in range(uvs.shape[0]):
            z = Z[uvs[idx, 0],uvs[idx, 1]]
            if math.isnan(z):
                continue
            if z==0:print("z is zero")
            x = ((uvs[idx, 0] - self.cx) / self.fx) * z
            y = ((uvs[idx, 1] - self.cy) / self.fy) * z

            xyzs.append([x, y, z])
          return xyzs

    # turn uvs of Nx2 array into uvs_undistorted of Nx2 
    def undistort_points(self, uvs):
        if self.is_distorted:
            uvs_undistorted = cv2.undistortPoints(np.expand_dims(uvs, axis=1), self.K, self.D, None, self.K)      
            return uvs_undistorted.ravel().reshape(uvs_undistorted.shape[0], 2)
        else:
            return uvs 
        

    
