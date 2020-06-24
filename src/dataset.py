#!/usr/bin/env python

import numpy as np 
from enum import Enum
import cv2
import time 
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import message_filters
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix

class DatasetType(Enum):
    NONE = 0
    VIDEO = 1
    LIVE = 2 # subscribing to a ros-topic

class Dataset(object):
    def __init__(self, path, fps=None, type=DatasetType.NONE):
        self.path=path 
        self.type=type    
        self.is_ok = True
        self.fps = fps   
        if fps is not None:       
            self.Ts = 1./fps 
        else: 
            self.Ts = None 
          
        self.timestamps = None 
        self._timestamp = None       # current timestamp if available [s]
        self._next_timestamp = None  # next timestamp if available otherwise an estimate [s]
        
    def isOk(self):
        return self.is_ok

    def getImage(self):
        return None 

    def getDepth(self):
        return None        

    def getTimestamp(self):
        return self._timestamp
    
    def getNextTimestamp(self):
        return self._next_timestamp

class VideoDataset(Dataset): 
    def __init__(self, path, type=DatasetType.VIDEO): 
        super(VideoDataset, self).__init__(path, None , type)    
        self.filename = path
        self.cap = cv2.VideoCapture(self.filename)
        if not self.cap.isOpened():
            raise IOError('Cannot open video file: ', self.filename)
        else: 
            print('Processing Video Input')
            self.num_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) 
            self.fps = float(self.cap.get(cv2.CAP_PROP_FPS))
            self.Ts = 1./self.fps 
            print('num frames: ', self.num_frames)  
            print('fps: ', self.fps)              
        self.is_init = False   
            
    def getImage(self, frame_id):
        # retrieve the first image if its id is > 0 
        if self.is_init is False and frame_id > 0:
            self.is_init = True 
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        self.is_init = True
        ret, image = self.cap.read()
        #self._timestamp = time.time()  # rough timestamp if nothing else is available 
        self._timestamp = float(self.cap.get(cv2.CAP_PROP_POS_MSEC)*1000)
        self._next_timestamp = self._timestamp + self.Ts 
        if ret is False:
            print('ERROR while reading from file: ', self.filename)
        return image

"""
N.B: for live streams, set the rgb_topic and depth_topic arguments to the corrosponding ros-topics that you are subscribing into.
ex: camera/rgb/image_raw
"""

class LiveStream(Dataset): 
    def __init__(self, rgb_topic, depth_topic , odom_topic,  type=DatasetType.LIVE): 
        super(LiveStream, self).__init__(None,  None, type)    
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
	self.odom_topic = odom_topic
        self.rgb_image = None
        self.depth_image = None
	self.translation = None
	self.rotationEuler = None 
	self.rotationMatrix = None
        self.bridge = CvBridge()
	    
    def synchronize(self):
	
        """Method that listens the topic /camera/rgb/image_raw and /odom with time synchronization"""

        def callback(image, image_d, odom):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
                image = np.array(cv_image, dtype=np.uint8)
                self.rgb_image =  image

		cv_depth = self.bridge.imgmsg_to_cv2(image_d, "passthrough")
                depth_image = np.array(cv_depth, dtype=np.float32)
                self.depth_image =  depth_image

		x = round(odom.pose.pose.position.x,3)
		y = round(odom.pose.pose.position.y,3)
		z = round(odom.pose.pose.position.z,3)
		self.translation = [x,y,z]
		quaternion = odom.pose.pose.orientation
		quaternion_list = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
		(roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
		self.rotationEuler = (round(roll,3), round(pitch,3), round(yaw,3))
		self.rotationMatrix = quaternion_matrix(quaternion_list)
	
	
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {0}".format(e))
            
        def listener(self):
                sub_image = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    		sub_odom  = message_filters.Subscriber("/odom", Odometry)
		sub_depth = message_filters.Subscriber("/camera/depth/image_raw", Image)
    		ts = message_filters.TimeSynchronizer([sub_image,sub_depth, sub_odom], 10)
    		ts.registerCallback(callback)

        listener(self)
	


    def Rgb_Image(self):
	
        """Method that listens the topic /camera/rgb/image_raw"""

        def callback(data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
                image = np.array(cv_image, dtype=np.uint8)
                self.rgb_image =  image
	
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {0}".format(e))
            
        def listener(self):
            rospy.Subscriber(self.rgb_topic, Image, callback)

        listener(self)

    def Depth_Image(self):

        """Method that listens the topic /camera/depth/image_raw"""

        def callback(data):
            try:
                cv_depth = self.bridge.imgmsg_to_cv2(data, "passthrough")
                depth_image = np.array(cv_depth, dtype=np.float32)
                self.depth_image =  depth_image
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {0}".format(e))
            
        def listener(self):
            rospy.Subscriber(self.depth_topic, Image, callback)

        listener(self)
            
    def getPose(self):
	return self.rotation, self.translation
 
    def getImage(self):
        if self.rgb_image is None:
            print("Error while reading from topic: ", self.rgb_topic)
            return None
        return self.rgb_image

    def getDepth(self):
        if self.depth_image is None:
            print("Error while reading from topic: ", self.depth_topic)
            return None
        return self.depth_image
