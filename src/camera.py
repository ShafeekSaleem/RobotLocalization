import numpy as np 
import cv2

"""
fx, fy - focal length of the camera in pixel coordinates
cx, cy - principle point
D      - distortion coefficeints , D = [k1, k2, p1, p2, k3]
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

    def add_ones(self, x):
        if len(x.shape) == 1:
            return np.array([x[0], x[1], 1])
        else:
            return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)

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
            z = Z[0] if len(Z) == 1 else Z[idx]
            x = ((uvs[idx, 0] - self.c_x) / self.f_x) * z
            y = ((uvs[idx, 1] - self.c_y) / self.f_y) * z
            xyzs.append([x, y, z])
          return xyzs

    # turn uvs of Nx2 array into uvs_undistorted of Nx2 
    def undistort_points(self, uvs):
        if self.is_distorted:
            uvs_undistorted = cv2.undistortPoints(np.expand_dims(uvs, axis=1), self.K, self.D, None, self.K)      
            return uvs_undistorted.ravel().reshape(uvs_undistorted.shape[0], 2)
        else:
            return uvs 
        

    
