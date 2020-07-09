import numpy as np 
import cv2
from enum import Enum
from feature_tracker import FeatureTrackerTypes, FeatureTrackingResult
from parameters import Parameters
from utils import *

class ImageRecievedState(Enum):
    NO_IMAGES_YET   = 0    
    GOT_FIRST_IMAGE = 1

   
MinNumFeature = Parameters.MinNumFeatureDefault
RansacThresholdPixels = Parameters.RansacThresholdPixels
RansacProb = Parameters.RansacProb
RansacThresholdNormalized = Parameters.RansacThresholdNormalized
AbsoluteScaleThreshold = Parameters.AbsoluteScaleThreshold
Alpha = Parameters.Alpha
MinAlpha = Parameters.MinAlpha
ErrorThreshold = Parameters.ErrorThreshold

class VisualOdometry(object):
    def __init__(self, cam, feature_tracker, dynamic_alpha = False, set_error_threshold = False ):
        self.state = ImageRecievedState.NO_IMAGES_YET
        self.cam = cam #camera object
        self.cur_image = None   # current image
        self.prev_image = None  # previous/reference image
        self.prev_depth = None
        self.cur_depth = None
        self.groundtruth_t = []
        self.groundtruth_R = []
        
        self.kps_ref = None  # reference keypoints 
        self.des_ref = None # refeference descriptors 
        self.kps_cur = None  # current keypoints 
        self.des_cur = None # current descriptors 

        self.cur_R = np.zeros((3,1)) # current rotation 
        self.cur_t = np.zeros((3,1)) # current translation
 
     
        self.feature_tracker = feature_tracker
        self.track_result = None 

        self.poses = []              # track list of poses
        self.trans_est = []           # history of  estimated translation
        self.rotation_est = []           # history of  estimated rotation
        self.trans_est_ref = []         # history of estimated translations centered w.r.t. first one
        self.rotation_est_ref = []         # history of estimated rotation centered w.r.t. first one
        self.errors = []    # history of errors

        self.num_matched_kps = None    # current number of matched keypoints  
        self.num_inliers = None        # current number of inliers
        self.mask   = None # mask of matched keypoints

        self.alpha = Alpha
        self.alpha0 = Alpha
        self.error_threshold = ErrorThreshold
        self.dynamic_alpha = dynamic_alpha
        self.set_error_threshold = set_error_threshold
        self.scale = 1

    def computeFundamentalMatrix(self, kps_ref, kps_cur):
        F, mask = cv2.findFundamentalMat(kps_ref, kps_cur, cv2.FM_RANSAC, param1=RansacThresholdPixels, param2=RansacProb)
        if F is None or F.shape == (1, 1):
            # no fundamental matrix found
            raise Exception('No fundamental matrix found')
        elif F.shape[0] > 3:
            # more than one matrix found, pick the first
            F = F[0:3, 0:3]
        return np.matrix(F), mask 	

    def removeOutliers(self, mask): 
        if mask is not None:       
            mask_index = [ i for i,v in enumerate(mask) if v > 0]    
            self.kps_cur = self.kps_cur[mask_index]           
            self.kps_ref = self.kps_ref[mask_index]           
            if self.des_cur is not None: 
                self.des_cur = self.des_cur[mask_index]        
            if self.des_ref is not None: 
                self.des_ref = self.des_ref[mask_index]  

    def estimatePose(self, kps_ref_matched, kps_cur_matched, frame_id):
        # undistorting points 
        kp_ref_u = self.cam.undistort_points(kps_ref_matched)	
        kp_cur_u = self.cam.undistort_points(kps_cur_matched)

        # turn [u,v] -> [x,y] : pixel coordinates -> normalized coordinates
        self.kpn_ref = self.cam.unproject_points(kp_ref_u)
        self.kpn_cur = self.cam.unproject_points(kp_cur_u)
        if self.kpn_ref.shape[0]<5:
            self.alpha = True
            print("Not enough key-points are matched. Using ground truth...")
            return np.eye(3,3), self.ground_truth_t[frame_id]
        else:
            self.alpha = False
            E, self.mask = cv2.findEssentialMat(self.kpn_cur, self.kpn_ref, focal=1, pp=(0., 0.), method=cv2.RANSAC, prob=RansacProb, threshold=RansacThresholdNormalized)    
            _, R, t, mask = cv2.recoverPose(E, self.kpn_cur, self.kpn_ref, focal=1, pp=(0., 0.))   
            return R,t  # Rotation and Translation (with respect to 'ref' frame)

    def estimatePosePNP(self, xyz_ref, kps_cur_matched, frame_id):
        _, R, t, inliers = cv2.solvePnPRansac( xyz_ref, kps_cur_matched, self.cam.K, self.cam.D)
        return R, t, inliers

    def computeError( self, vo_translation, mo_translation ):
        error   = [a_i - b_i for a_i, b_i in zip(vo_translation, mo_translation)]
        err_sum = sum([abs(i) for i in error])
        return error, err_sum

    def processFirstFrame(self):
        print("processing first frame")
        self.kps_ref, self.des_ref = self.feature_tracker.detectAndCompute(self.cur_image)
        # convert from list of keypoints to an array of points 
        self.kps_ref = np.array([x.pt for x in self.kps_ref], dtype=np.float32) 
        # self.draw_img = self.drawFeatureTracks(self.cur_image)

    def processInterFrames(self, frame_id):
        print("processing "+str(frame_id)+" frame")
        # track features 
        self.track_result = self.feature_tracker.track(self.prev_image, self.cur_image, self.kps_ref, self.des_ref)
        
        # switch x,y dimension to y,x
        kps_ref_yx = switch_xy(self.track_result.kps_ref_matched)

        # unproject u,v points to x,y,z coordinates
        xyz_ref, indexes = self.cam.unproject_points_z(kps_ref_yx, self.prev_depth)

        # remove keypoints with depth=0
        kps_cur_matched_z = removeByIndexes( self.track_result.kps_cur_matched, indexes )
        
        # estimate pose
        R, t, inliers = self.estimatePosePNP(xyz_ref, kps_cur_matched_z, frame_id)
        t = t[::-1]*-1
        # update keypoints history  
        self.kps_ref = self.track_result.kps_ref
        self.kps_cur = self.track_result.kps_cur
        self.des_cur = self.track_result.des_cur
        self.num_inliers = inliers.shape[0]
        # self.mask = inliers

        # update estimations
        self.trans_est.append(t)
        self.rotation_est.append(R)
        
        self.num_matched_kps = self.kps_ref.shape[0] 
        print('# matched points: ', self.num_matched_kps)
        print('Inliers after Ransac : ' + str(self.num_inliers))
        
        # compute absolute rotation and translation with reference to world frame
        """
        N.B:
            We represent relative rotation and translation from one frame A to another frame B by Rab and tab.
            
            Compose absolute motion [Rwa,twa] with estimated relative motion [Rab,s*tab] to compute [Rwb,twb].(s is the translation scale)
            [Rwb,twb] = [Rwa,twa]*[Rab,tab] = [Rwa*Rab, twa + Rwa*tab]
        """

        self.cur_t = self.cur_t + self.scale*t
        self.cur_R = R # todo
            
        # draw image         
        # self.draw_img = self.drawFeatureTracks(self.cur_image) 
                  
        self.kps_ref = self.kps_cur
        self.des_ref = self.des_cur
        self.updateHistory(frame_id)
            

    def trackImage(self, img, depth_image, gt_t, gt_R, frame_id):
        # convert image to gray if needed    
        if img.ndim>2:
            img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        # check coherence of image size with camera settings 
        assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"

        self.cur_image = img
        self.cur_depth = depth_image
        self.groundtruth_t.append(gt_t)
        self.groundtruth_R.append(gt_R)
        
        if(self.state == ImageRecievedState.GOT_FIRST_IMAGE):
            self.processInterFrames(frame_id)
        elif(self.state == ImageRecievedState.NO_IMAGES_YET):
            self.processFirstFrame()
            self.state = ImageRecievedState.GOT_FIRST_IMAGE            
        self.prev_image = self.cur_image
        self.prev_depth = self.cur_depth

    def drawFeatureTracks(self, img, reinit = False):
        draw_img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB) 
        if(self.state == ImageRecievedState.GOT_FIRST_IMAGE):            
            if reinit:
                for p1 in self.kps_cur:
                    a,b = p1.ravel()
                    cv2.circle(draw_img,(a,b),1, (0,255,0),-1)                    
            else:    
                for i,pts in enumerate(zip(self.track_result.kps_ref_matched, self.track_result.kps_cur_matched)):        
                    p1, p2 = pts 
                    a,b = p1.ravel()
                    c,d = p2.ravel()
                    cv2.line(draw_img, (a,b),(c,d), (0,255,0), 1)
                    cv2.circle(draw_img,(a,b),1, (0,0,255),-1)   
        return draw_img   

    def updateHistory(self, frame_id):
        self.trans_est_ref.append(self.cur_t)
        self.rotation_est_ref.append(self.cur_R)

        error, err_sum = self.computeError(self.cur_t, self.groundtruth_t[frame_id])
        #self.errors.append(err_sum)
        if self.set_error_threshold:
            if err_sum> self.error_threshold:
                self.alpha = self.alpha/(err_sum/self.error_threshold)

        # re-estimate pose as error*vo+mo
        pose = [e*self.alpha + t for e, t in zip(error, self.groundtruth_t[frame_id])]
        self.cur_t = pose
        error, err_sum = self.computeError(self.cur_t, self.groundtruth_t[frame_id])
        self.errors.append(err_sum)
        if self.dynamic_alpha:
            self.alpha = max(MinAlpha, self.num_inliers/MinNumFeature)
        else:
            self.alpha = self.alpha0
        self.poses.append(pose)   
        
