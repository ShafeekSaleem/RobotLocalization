#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

rospy.init_node('read_image', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

index = 0

# Define a callback for the Image message
def image_callback(img_msg):
    global index
    index += 1
     # log some info about the image topic
    rospy.loginfo(img_msg.header)

     # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	frame = np.array(cv_image, dtype=np.uint8)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

     # Show the converted image
    cv2.imwrite("/home/dexter/Pictures/image"+str(index)+".jpg", frame )
    rospy.loginfo(frame)

    time.sleep(15) #change this
    

sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)


while not rospy.is_shutdown():
    rospy.spin()

