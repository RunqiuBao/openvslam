#!/usr/bin/env python
import rospy
import numpy as np

import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import time
import torch 
import torch.nn as nn
from torch.autograd import Variable
from util import *
import argparse
import os 
import os.path as osp
from darknet import Darknet
from preprocess import prep_image2, inp_to_image
import pandas as pd
import random 
import pickle as pkl
import itertools


def myCallBack(imageleft_ori, imageright, args):
    pub_leftimage = args[0]
    pub_rightimage = args[1]
    pub_maskimage = args[2]
    model = args[3]
    num_classes = args[4]
    classes = args[5]
    cvbridge = CvBridge()
    imageleft = cvbridge.imgmsg_to_cv2(imageleft_ori, "rgb8")
    lefttimestamp = imageleft_ori.header.stamp

    # predict
    inp_dim = int(model.net_info["height"])
    height, width = imageleft.shape[:2]
    confidence = 0.5
    nms_thesh = 0.4
    imgleft_tensor, _, _ = prep_image2(imageleft, inp_dim)
    CUDA = torch.cuda.is_available()
    if CUDA:
        imgleft_tensor = imgleft_tensor.cuda()
    with torch.no_grad():
        start = time.time()
        prediction = model(Variable(imgleft_tensor), CUDA)
        # print(time.time()-start)
    prediction = write_results(prediction, confidence, num_classes, nms = True, nms_conf = nms_thesh)

    # generate mask image, person, vehicle, truck, bus
    maskimg = np.ones((height, width))*255 #in openvslam, 255 means no mask, 0 means mask.
    dynaset = set(['person', 'vehicle', 'truck', 'bus'])
    prediction = prediction.cpu().numpy()
    if prediction.shape[1]!=8:
        #print(prediction)
        return
    for predict in prediction:
        thelabel = classes[int(predict[-1])]
        if set([thelabel]).issubset(dynaset):
            xmin, ymin = predict[[1,3]]
            xmax, ymax = predict[[2,4]]
            if height>width:
                print("weird! image height larger than width")
            else:
                scale = width/int(inp_dim)
                # print([xmin, ymin, xmax, ymax])
                mylist = np.array([xmin, ymin, xmax, ymax])
                mylist = scale*mylist
                # print(scale)
                # print("what is mylist?")
                # print(mylist)
                xmin, ymin, xmax, ymax = mylist
                # print([xmin, ymin, xmax, ymax])
            cv2.rectangle(maskimg, (xmin, ymin), (xmax, ymax), (0,0,0), thickness=-1)
    maskimg = maskimg.astype(np.uint8)

    imageleft_ori.header.stamp = lefttimestamp
    pub_leftimage.publish(imageleft_ori)

    imageright.header.stamp = lefttimestamp
    pub_rightimage.publish(imageright)
    
    maskimg = cvbridge.cv2_to_imgmsg(maskimg, "mono8")
    maskimg.header.stamp = lefttimestamp
    pub_maskimage.publish(maskimg)
    
    rospy.loginfo("mask generated")

def yolov3():
    rospy.loginfo("start yolov3 rospy!")
    print("start yolov3 rospy!")
    rospy.init_node('yolov3', anonymous=True)
    pub_leftimage = rospy.Publisher('imagel_seman', Image, queue_size=1)
    pub_rightimage = rospy.Publisher('imager_seman', Image, queue_size=1)
    pub_maskimage = rospy.Publisher('maskimage', Image, queue_size=1)

    CUDA = torch.cuda.is_available()
    print("cuda condition: {}".format(CUDA))
    num_classes = 80
    classes = load_classes('src/yolov3/src/coco.names')
    model = Darknet('src/yolov3/src/yolov3.cfg')
    model.load_weights('src/yolov3/src/yolov3.weights')
    model.net_info["height"] = "416"
    if CUDA:
        model.cuda()
    model.eval()#evaluation mode

    sub_leftimage = message_filters.Subscriber('imagel', Image)
    sub_rightimage = message_filters.Subscriber('imager', Image)
    ts = message_filters.ApproximateTimeSynchronizer([sub_leftimage, sub_rightimage], 1, 0.01, allow_headerless=True)#queue length, max time diff
    ts.registerCallback(myCallBack, (pub_leftimage, pub_rightimage, pub_maskimage, model, num_classes, classes))
    rospy.spin()

if __name__=='__main__':
    yolov3()
