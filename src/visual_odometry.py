import numpy as np 
import cv2
from enum import Enum
from feature_tracker import FeatureTrackerTypes, FeatureTrackingResult
from parameters import Parameters
from utils import poseRt

class ImageRecievedState(Enum):
    NO_IMAGES_YET   = 0    
    GOT_FIRST_IMAGE = 1
   
MinNumFeature = Parameters.MinNumFeatureDefault
RansacThresholdPixels = Parameters.RansacThresholdPixels
RansacProb = Parameters.RansacProb
RansacThresholdNormalized = Parameters.RansacThresholdNormalized
AbsoluteScaleThreshold = Parameters.AbsoluteScaleThreshold

class VisualOdometry(object):
    def __init__(self, cam, feature_tracker, ground_truth_t,ground_truth_r):
        self.state = ImageRecievedState.NO_IMAGES_YET
        self.cam = cam #camera object
        self.cur_image = None   # current image
        self.prev_image = None  # previous/reference image

        self.kps_ref = None  # reference keypoints 
        self.des_ref = None # refeference descriptors 
        self.kps_cur = None  # current keypoints 
        self.des_cur = None # current descriptors 

        self.cur_R = np.eye(3,3) # current rotation 
        self.cur_t = np.zeros((3,1)) # current translation
        self.cur_t2 = np.zeros((3,1)) # current translation
     
        self.feature_tracker = feature_tracker
        self.track_result = None 

        self.init_history = True 
        self.poses = []              # track list of poses
        self.trans_est = None           # history of starting estimated translation      
        self.trans_est2 = None 
        self.trans_gt_ref = None            # history of estimated ground truth translations centered w.r.t. world frame   ( if available )
        self.rotation_gt_ref = None            # history of estimated ground truth rotations centered w.r.t. world frame ( if available )
        self.trans_est_ref = []         # history of estimated translations centered w.r.t. first one
        self.trans_est_ref2 = []         # history of estimated translations centered w.r.t. first one

        self.num_matched_kps = None    # current number of matched keypoints  
        self.num_inliers = None        # current number of inliers
        self.mask   = None # mask of matched keypoints
        self.trueX, self.trueY, self.trueZ, self.scale = 0, 0, 0, 1

        self.alpha = False

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

    def processFirstFrame(self):
        print("processing first frame")
        self.kps_ref, self.des_ref = self.feature_tracker.detectAndCompute(self.cur_image)
        # convert from list of keypoints to an array of points 
        self.kps_ref = np.array([x.pt for x in self.kps_ref], dtype=np.float32) 
        self.draw_img = self.drawFeatureTracks(self.cur_image)

    def processInterFrames(self, frame_id):
        print("processing "+str(frame_id)+" frame")
        # track features 
        self.track_result = self.feature_tracker.track(self.prev_image, self.cur_image, self.kps_ref, self.des_ref)
        # estimate pose
        R, t = self.estimatePose(self.track_result.kps_ref_matched, self.track_result.kps_cur_matched, frame_id)
        # update keypoints history  
        self.kps_ref = self.track_result.kps_ref
        self.kps_cur = self.track_result.kps_cur
        self.des_cur = self.track_result.des_cur

        if self.alpha:
                self.cur_t = t
                self.cur_R = R
        if not self.alpha:
            self.num_matched_kps = self.kpn_ref.shape[0] 
            self.num_inliers =  np.sum(self.mask)
            print('# matched points: ', self.num_matched_kps, ', # inliers: ', self.num_inliers)

            # compute absolute rotation and translation with reference to world frame
            """
            N.B:
                We represent relative rotation and translation from one frame A to another frame B by Rab and tab.
                
                Compose absolute motion [Rwa,twa] with estimated relative motion [Rab,s*tab] to compute [Rwb,twb].(s is the translation scale)
                [Rwb,twb] = [Rwa,twa]*[Rab,tab] = [Rwa*Rab, twa + Rwa*tab]
            """
            absolute_scale = self.scale
            if(absolute_scale > AbsoluteScaleThreshold):
                self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t)
                self.cur_t2 = self.cur_t + t
                self.cur_R = self.cur_R.dot(R)
            
        # draw image         
        self.draw_img = self.drawFeatureTracks(self.cur_image) 
                  
        self.kps_ref = self.kps_cur
        self.des_ref = self.des_cur
        self.updateHistory()
            

    def trackImage(self, img, frame_id):
        # convert image to gray if needed    
        if img.ndim>2:
            img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        # check coherence of image size with camera settings 
        assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"

        self.cur_image = img 
        if(self.state == ImageRecievedState.GOT_FIRST_IMAGE):
            self.processInterFrames(frame_id)
        elif(self.state == ImageRecievedState.NO_IMAGES_YET):
            self.processFirstFrame()
            self.state = ImageRecievedState.GOT_FIRST_IMAGE            
        self.prev_image = self.cur_image

    def drawFeatureTracks(self, img, reinit = False):
        draw_img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB) 
        if(self.state == ImageRecievedState.GOT_FIRST_IMAGE):            
            if reinit:
                for p1 in self.kps_cur:
                    a,b = p1.ravel()
                    cv2.circle(draw_img,(a,b),1, (0,255,0),-1)                    
            else:    
                for i,pts in enumerate(zip(self.track_result.kps_ref_matched, self.track_result.kps_cur_matched)):
                    if self.mask[i]:
                        p1, p2 = pts 
                        a,b = p1.ravel()
                        c,d = p2.ravel()
                        cv2.line(draw_img, (a,b),(c,d), (0,255,0), 1)
                        cv2.circle(draw_img,(a,b),1, (0,0,255),-1)   
        return draw_img   

    def updateHistory(self):
        if (self.init_history is True) and (self.trueX is not None):
            self.trans_est = np.array([self.cur_t[0], self.cur_t[1], self.cur_t[2]])  # setting up the translation of the first frame as starting translation 
            self.trans_est2 = np.array([self.cur_t2[0], self.cur_t2[1], self.cur_t2[2]]) 
            self.init_history = False 
        if (self.trans_est is not None):
            # translation of the current frame with respect to the first frame
            pose = [self.cur_t[0]-self.trans_est[0], self.cur_t[1]-self.trans_est[1], self.cur_t[2]-self.trans_est[2]]
            pose2 = [self.cur_t2[0]-self.trans_est2[0], self.cur_t2[1]-self.trans_est2[1], self.cur_t2[2]-self.trans_est2[2]]
            self.trans_est_ref.append(pose)
            self.trans_est_ref2.append(pose2) 
            self.poses.append(poseRt(self.cur_R, pose))   
        
