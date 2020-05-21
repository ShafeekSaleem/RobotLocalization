#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

rospy.init_node('cv_bridge_demo', anonymous=True)

# Print "Hello ROS!" to the Terminal and ROSLOG
rospy.loginfo("Hello ROS!")


 # Initialize the CvBridge class
bridge = CvBridge()

 # Define a function to show the image in an OpenCV Window
def show_image(img):
    
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

 # Define a callback for the Image message
def image_callback(img_msg):
     # log some info about the image topic
    rospy.loginfo(img_msg.header)

     # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	frame = np.array(cv_image, dtype=np.uint8)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

     # Show the converted image
    #cv2.imwrite("/home/dexter/Pictures/image.jpg", frame )
    #show_image(cv_image)
    rospy.loginfo(frame)
    
    cv2.imwrite("/home/dexter/Pictures/image.jpg", frame )
    time.sleep(10)
    

sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

#cv2.namedWindow("Image Window", 1)

while not rospy.is_shutdown():
    rospy.spin()

