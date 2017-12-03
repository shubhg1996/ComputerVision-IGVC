#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from numpy import linalg
import cv_bridge
from sensor_msgs.msg import Image

def nothing(x):
    pass

count = flag=meang=meanb=n=globmean=globvar=0
h = np.zeros((3,3),dtype="uint8")
dist = 0

def topview(image):
    src = np.array([[405,411],[1152,411],[1634,972],[160,972]],dtype='float32')

    dest = np.array([[405/2+100,411],[1152/2+100,411],[1152/2+100,972],[405/2+100,972]],dtype='float32')
    h, status = cv2.findHomography(src, dest)
    if len(image.shape)==2:
        height,width = image.shape
        imgd = np.zeros((height,width),dtype='uint8')
        imgd = cv2.warpPerspective(image, h, (width,height))
    else:
        height,width,col = image.shape
        imgd = np.zeros((height,width,col),dtype='uint8')
        imgd[:,:,0] = cv2.warpPerspective(image[:,:,0], h, (width,height))
        imgd[:,:,1] = cv2.warpPerspective(image[:,:,1], h, (width,height))
        imgd[:,:,2] = cv2.warpPerspective(image[:,:,2], h, (width,height))
    return imgd

def hsvfilter(frame):
    # img = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    minbound = np.array([150,150,150],dtype='uint8')
    maxbound = np.array([255,255,255],dtype='uint8')
    imfinal1 = cv2.inRange(frame,minbound,maxbound)

    return imfinal1

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

def left_update(lpts, olmean, ovlmax):
    lmean = [np.mean(lpts[:,0]),np.mean(lpts[:,1])]

    if(olmean[0]+olmean[1] == 0):
        pass
    else:    #weight of old frame
        lmean = (lmean+olmean*2)/3
    lcov = np.cov(lpts.T)

    lval,lvec = linalg.eigh(lcov)
    _,evlmax,_,ilmax = cv2.minMaxLoc(lval)
    vlmax = lvec[:,ilmax[1]]
    if(ovlmax.any() is not 0):
        vlmax = (vlmax+ovlmax*2)/3
    return lmean, vlmax

def right_update(rpts, ormean, ovrmax):
    rmean = [np.mean(rpts[:,0]),np.mean(rpts[:,1])]
    if(ormean[0]+ormean[1] == 0):
        pass
    else:    #weight of old frame
        rmean = (rmean+ormean*2)/3
    rcov = np.cov(rpts.T)

    rval,rvec = linalg.eigh(rcov)
    _,evrmax,_,irmax = cv2.minMaxLoc(rval)
    vrmax = rvec[:,irmax[1]]
    if(ovrmax.any() is not 0):
        vrmax = (vrmax+ovrmax*2)/3
    return rmean, vrmax

def mask_maker(lmean, vlmax, rmean, vrmax):
    global height,width, maskframel
    maskframel = (np.zeros_like(maskframel))
    maskframer = (np.zeros_like(maskframel))
    maskframec = (np.zeros_like(maskframel))
    if(lmean[1]==rmean[1]):
        rmean[1] = lmean[1]+1
    cv2.line(maskframec,(0,int(lmean[0]+(rmean[0]-lmean[0])/(rmean[1]-lmean[1])*(0-lmean[1]))),(width,int(lmean[0]+(rmean[0]-lmean[0])/(rmean[1]-lmean[1])*(width-lmean[1]))),(255,0,0),2)

    cv2.line(maskframel,(int(lmean[1]+vlmax[1]/vlmax[0]*(height-lmean[0])),height),(int(lmean[1]+vlmax[1]/vlmax[0]*(0-lmean[0])),0),(255,0,0),2)
    cv2.line(maskframer,(int(rmean[1]+vrmax[1]/vrmax[0]*(height-rmean[0])),height),(int(rmean[1]+vrmax[1]/vrmax[0]*(0-rmean[0])),0),(255,0,0),2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(150,150))
    # print maskframel.dtype
    maskframeln = (cv2.dilate(maskframel,kernel)).astype("uint8")
    # print maskframel.dtype
    maskframern = (cv2.dilate(maskframer,kernel)).astype("uint8")
    maskframecn = (cv2.dilate(maskframec,kernel)).astype("uint8")
    return maskframeln, maskframern, maskframecn

def distance(lmean, vlmax, rmean, vrmax):
    diff = np.asarray(rmean) - np.asarray(lmean)
    diffcross = np.cross(diff,vrmax)
    tdist = np.abs(diffcross)
    diffcross = np.cross(diff,vlmax)
    tdist = (tdist + np.abs(diffcross))/2
    return tdist

def push_the_lane(lmean, vlmax, rmean, vrmax):
    global height, center
    lcent = lmean[1]+vlmax[1]/vlmax[0]*(height/2-lmean[0])
    rcent = rmean[1]+vrmax[1]/vrmax[0]*(height/2-rmean[0])
    if lcent<0:
        lmean[1] = lmean[1]+200
        rmean[1] = rmean[1]+200
    if rcent>2*center:
        rmean[1] = rmean[1]-200
        lmean[1] = lmean[1]-200
    return lmean, rmean


center = 0
maskframel = np.zeros((10,10))
maskframer = np.zeros((10,10))
maskframec = np.zeros((10,10))
lmean = rmean = np.asarray([0,0])
vlmax = vrmax = np.asarray([0,0])
omaskframel = maskframel
omaskframer = maskframer
omaskframec = maskframec
olmean = lmean
ovlmax = vlmax
ormean = rmean
ovrmax = vrmax
# cap = cv2.VideoCapture('/home/shubh/Documents/opencv_vids/lolwa.mp4')
# ret,frame = cap.read()

def pca(lhalf,rhalf,maskframel,maskframer,maskframec, olmean,ovlmax,ormean,ovrmax):
    global height,dist,center
    lpts = lhalf.nonzero()
    rpts = rhalf.nonzero()
    lpts = np.asmatrix(lpts).T
    rpts = np.asmatrix(rpts).T
    olmean = np.asarray(olmean)
    ormean = np.asarray(ormean)
    lmean = olmean
    rmean = ormean
    vrmax = ovrmax
    vlmax = ovlmax
    maskframeln = maskframel
    maskframern = maskframer
    maskframecn = maskframec

    # rpts[:,1] = rpts[:,1] + center
    # print lpts.shape
    if lpts.shape[0] >200 and rpts.shape[0] >200:
        
        lmean, vlmax = left_update(lpts, olmean, ovlmax)
        rmean, vrmax = right_update(rpts, ormean, ovrmax)
        tdist = distance(lmean, vlmax, rmean, vrmax)
        if tdist<300:        #lanes too close
            lmean[1] = center-200
            rmean[1] = center+200
        else:
            # dist = (dist*10+tdist)/11
            dist = 400
        adiv = np.dot(vlmax,vrmax.T)
        print adiv,dist
        if adiv<0.85:       #angle not parallel
            # print vlmax,vrmax
            if vlmax[0]>-0.82:
                vlmax[0] = -1
                vlmax[1] = 0
            if vrmax[0]>-0.82:
                vrmax[0] = -1
                vrmax[1] = 0
        lmean, rmean = push_the_lane(lmean, vlmax, rmean, vrmax)
        maskframeln, maskframern, maskframecn = mask_maker(lmean, vlmax, rmean, vrmax)
    elif lpts.shape[0] > 200:
        
        lmean, vlmax = left_update(lpts, olmean, ovlmax)
        rmean = lmean + dist
        vrmax = vlmax
        lmean, rmean = push_the_lane(lmean, vlmax, rmean, vrmax)

        maskframeln, maskframern, maskframecn = mask_maker(lmean, vlmax, rmean, vrmax)

    elif rpts.shape[0] > 200:
        
        rmean, vrmax = right_update(rpts, ormean, ovrmax)
        lmean = rmean - dist
        vlmax = vrmax
        lmean, rmean = push_the_lane(lmean, vlmax, rmean, vrmax)

        maskframeln, maskframern, maskframecn = mask_maker(lmean, vlmax, rmean, vrmax)
    # cv2.imshow('lhalf',maskframel)
    # cv2.imshow('rhalf',maskframer)    
    return maskframeln,maskframern,maskframecn,lmean,vlmax,rmean,vrmax



# cv2.imshow('frame',thresframe)
# cv2.imshow('mask',maskframe)
# cv2.waitKey(0)
# exit()

def image_callback(msg):
    global maskframel,maskframer,maskframec,frame, dmean2, dvar2, globvar, globmean,n,height,width,center,omaskframel,omaskframer,omaskframec,olmean,ormean,ovlmax,ovrmax
        
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    thresframe = hsvfilter(frame)
    topframe = topview(thresframe)
    # topact = topview(frame)
    iout = np.zeros_like(topframe)
    if center>0:
        lhalf = cv2.bitwise_and(topframe,topframe,mask=maskframel)
        rhalf = cv2.bitwise_and(topframe,topframe,mask=maskframer)
        chalf = cv2.bitwise_and(topframe,topframe,mask=maskframec)


        maskframel,maskframer,maskframec, lmean,vlmax,rmean,vrmax = pca(lhalf,rhalf,omaskframel,omaskframer,omaskframec, olmean,ovlmax,ormean,ovrmax)
        omaskframel = maskframel
        omaskframer = maskframer
        omaskframec = maskframec
        olmean = lmean
        ovlmax = vlmax
        ormean = rmean
        ovrmax = vrmax

        lcount = np.count_nonzero(lhalf)
        rcount = np.count_nonzero(rhalf)
        ccount = np.count_nonzero(chalf)
        if(max(lcount, rcount, ccount)==ccount):
            pass# cv2.line(topact,(int(lmean[1]), int(lmean[0])),(int(rmean[1]), int(rmean[0])),(0,0,255),2)
        else:
            cv2.line(iout,(int(lmean[1]+vlmax[1]/vlmax[0]*(height-lmean[0])),height),(int(lmean[1]+vlmax[1]/vlmax[0]*(0-lmean[0])),0),255,2)
            cv2.line(iout,(int(rmean[1]+vrmax[1]/vrmax[0]*(height-rmean[0])),height),(int(rmean[1]+vrmax[1]/vrmax[0]*(0-rmean[0])),0),255,2)
        msg2 = bridge.cv2_to_imgmsg(iout, encoding="mono8") 
        msg2.header.stamp = rospy.get_rostime()
        pub.publish(msg2)
    else:
        height,width,_ = frame.shape
        center = width/2-250
        # print "center ",center
        lhalf = np.zeros_like(topframe)
        rhalf = np.zeros_like(topframe)
        maskframel = np.zeros_like(lhalf)
        maskframer = np.zeros_like(lhalf)
        maskframec = np.zeros_like(lhalf)
        lhalf[:,0:center] = topframe[:,0:center]
        rhalf[:,center:2*center] = topframe[:,center:2*center]
        lmean = rmean = np.asarray([0,0])
        vlmax = vrmax = np.asarray([0,0])
        maskframel,maskframer,maskframec, lmean,vlmax,rmean,vrmax = pca(lhalf,rhalf,maskframel, maskframer,maskframec, lmean,vlmax,rmean,vrmax)
        omaskframel = maskframel
        omaskframer = maskframer
        omaskframec = maskframec
        olmean = lmean
        ovlmax = vlmax
        ormean = rmean
        ovrmax = vrmax

if __name__== '__main__':
    rospy.init_node('ltracking')

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
    exit()
