#!/usr/bin/env python
import numpy as np 
import cv2 
import time
import random
 
image1 = cv2.imread('/home/dexter/Pictures/image2.jpg') 
image2 = cv2.imread('/home/dexter/Pictures/image9.jpg') 

def draw_points(image, points, color, is_save = True):
	thickness = 2
	for point in points:
		image = cv2.circle(image, tuple(point), 1, color, thickness)
	if is_save:
		cv2.imwrite("/home/dexter/Pictures/image_"+str(random.randint(1,101))+".jpg", image)
	
def feature_matching( image1, image2, is_save = True ):	   
	# Initialize the ORB detector algorithm 
	orb = cv2.ORB_create() 
	   
	# detect the keypoints and compute 
	# the descriptors for the both images
	img1KP, img1DP = orb.detectAndCompute(image1,None) 
	img2KP, img2DP = orb.detectAndCompute(image2,None) 
	  
	# Initialize the Matcher with crossCheck = True
	matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) 
	matches = matcher.match(img1DP,img2DP) 

	matches = sorted(matches, key = lambda x:x.distance)
	 
	# draw the matches to the final image 
	final_img = cv2.drawMatches(image1, img1KP,  
	image2, img2KP, matches[:50] ,None, flags=2) 
	
	#getting pixel coordinates of the matched feature keypoints
	pts1 = np.int32([img1KP[mat.queryIdx].pt for mat in matches[:50]])
	pts2 = np.int32([img2KP[mat.trainIdx].pt for mat in matches[:50]])

	#calculate fundemental matrix
	F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_RANSAC)

	# select only inlier points
	pts1 = pts1[mask.ravel()==1]
	pts2 = pts2[mask.ravel()==1]
	if is_save:
		# save the final image 
		final_img = cv2.resize(final_img, (640,480)) 
		cv2.imwrite("/home/dexter/Pictures/result.jpg", final_img)
	
	return pts1, pts2, F

"""
pts1, pts2, F = feature_matching(image1, image2)
draw_points(image1, pts1, (255, 0, 0))
draw_points(image2, pts2, (0, 255, 0))
print(F)
"""
