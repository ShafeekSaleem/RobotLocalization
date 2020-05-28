#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

rospy.init_node('read_depth', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

# Define a callback for the Image message
def depth_callback(depth_msg):
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
	depth_array = np.array(depth_image, dtype=np.float32)
	
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    rospy.loginfo(depth_array[460])
    time.sleep(100)

sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)


while not rospy.is_shutdown():
    rospy.spin()

