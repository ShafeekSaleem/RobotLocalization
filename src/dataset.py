import numpy as np 
from enum import Enum
import cv2
import time 

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

    def getImage(self, frame_id):
        return None 

    def getDepth(self, frame_id):
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
