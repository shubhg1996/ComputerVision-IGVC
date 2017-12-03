#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image

flag = 0
meang = 0
meanb = 0
n = 0
globmean=0
globvar =0
boxs = 20
frame = 0
dmean2 = dvar2 = 0

def image_callback(msg):
	global frame, dmean2, dvar2, globvar, globmean,n
	frame = bridge.imgmsg_to_cv2(msg, "bgr8")
	height,width,_ = frame.shape
	frame1 = b2gext(frame)
	# cv2.imshow('image',b2gext(frame,t1))
	n2 = 0
	frame2 = cv2.inRange(frame1,gausslmin(0.7),gausslmax(0.7))
	frame1 = cv2.bitwise_and(frame1,frame1,mask=frame2)
	frame = cv2.bitwise_and(frame,frame,mask=255-frame2)
	dmean2 = np.mean(frame1)
	dvar2 = np.var(frame1)
	globvar = globvar*globvar + globmean*globmean
	globvar = (globvar*n+dvar2)/(n+1)
	globmean = (globmean*n+dmean2)/(n+1)
	# globvar = (globvar*n + dmean2*dmean2)/(n+1)
	globvar = np.sqrt(globvar - globmean*globmean)
	# print globvar
	n = n+1
	msg = bridge.cv2_to_imgmsg(255-frame2, encoding="mono8") 
	msg.header.stamp = rospy.get_rostime()
	pub.publish(msg)
    
    
    # Save your OpenCV2 image as a jpeg 
    #cv2.imwrite('camera_image.jpeg', cv2_img)

def gausslmax(x):
	global globvar,globmean
	temp = np.log(1/x)
	temp = np.sqrt(2*temp)
	temp = temp*globvar

	return globmean+temp

def gausslmin(x):
	global globvar,globmean
	temp = np.log(1/x)
	temp = np.sqrt(2*temp)
	temp = temp*globvar

	return globmean-temp

def b2gext(frame):
	global flag,meang,meanb,n,globmean,globvar
	height,width,_ = frame.shape
	# cv2.imshow('frame2',window)
	if flag == 0:
		window = frame[height-200:height,width/2-100:width/2+100,:]
		meanb = np.mean(window[:,:,0])
		meang = np.mean(window[:,:,1])
		cnvwind = (window[:,:,0]*float(meang)/float(meanb)/255.0).astype("float") - (window[:,:,1]/255.0).astype("float")
		cnvwind[cnvwind<0] = 0
		globmean = np.mean(cnvwind)
		globvar = 0.8#np.sqrt(np.var(cnvwind))
		n = 4*4
		flag = 1
	imret = (frame[:,:,0]*float(meang)/float(meanb)/255.0).astype("float") - (frame[:,:,1]/255.0).astype("float")
	imret[imret<0] = 0
	return imret
	
# globmean = np.array([135.691362831,135.4321424,137.666994954],dtype="float32")
# globvar = np.array([9.9112034735,11.9904986783,9.48740966622],dtype="float32")

if __name__== '__main__':
	rospy.init_node('grass_classifier')

	bridge = cv_bridge.CvBridge()
	rospy.Subscriber("/top_view", Image, image_callback)
	pub = rospy.Publisher("/lane_image", Image, queue_size=10)

	#vidFile = cv2.VideoCapture("/home/swati/catkin_ws/src/CV_algo/src/lolwa.mp4")
	#ret, frame = vidFile.read()
	#rospy.spin()
	
	# dmean2 = np.array([0,0,0],dtype="float32")
	
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		#ret, frame = vidFile.read()
		
		# cv2.imshow('image',255-frame2)
		
		rospy.spin()
		rate.sleep()
		key = cv2.waitKey(10)
		if key==27 or key==1048603:
			break
	exit()
