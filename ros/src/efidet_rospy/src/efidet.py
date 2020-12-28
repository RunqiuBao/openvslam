###!/usr/local/bin/python3.6
## efficidentDet in ros package
import torch
import rospy
from std_msgs.msg import Header
import numpy as np

import message_filters
from sensor_msgs.msg import Image
import cv2
import efficientnet_pytorch
from torchvision import transforms
from dataset import CocoDataset, Resizer, Normalizer
from config import COCO_CLASSES, colors

def myCallBack(imageleft, imageright, args):
    pub_leftimage = args[0]
    pub_rightimage = args[1]
    pub_maskimage = args[2]
    model = args[3]

    # predict
    image_size = 512
    height, width = imageleft.shape[:2]
    image = imageleft.copy()
    image = image.astype(np.float32)/255
    image[:,:,0] = (image[:,:,0]-0.485)/0.229 #suppose image channel order is RGB
    image[:,:,1] = (image[:,:,1]-0.456)/0.224
    image[:,:,2] = (image[:,:,2]-0.406)/0.225
    if height>width:
        scale = image_size/height
        resized_height = image_size
        resized_width = int(width*scale)
    else:
        scale = image_size/width
        resized_height = int(height*scale)
        resized_width = image_size
    image = cv2.resize(image, (resized_width, resized_height))
    imgtensor = np.zeros((image_size,image_size,3))
    imgtensor[0:resized_height, 0:resized_width] = image
    imgtensor = np.transpose(imgtensor, (2,0,1))#switch axis order
    imgtensor = imgtensor[None,:,:,:]
    imgtensor = torch.Tensor(imgtensor)
    if torch.cuda.is_available():
        imgtensor = imgtensor.cuda()
    with torch.no_grad():#eliminate noise in torch.gradient
        scores, labels, boxes = model(imgtensor)
        boxes /= scale
    
    # generate mask image, person, vehicle, truck, bus
    maskimg = np.zeros((height, width))
    dynaset = set(['person', 'vehicle', 'truck', 'bus'])
    for box_id in range(boxes.shape[0]):
        if scores[box_id].item()>=0.3 and set([COCO_CLASSES[int(labels[box_id])]]).issubset(dynaset):
            xmin, ymin, xmax, ymax = boxes[box_id, :]
            cv2.rectangle(maskimg, (xmin, ymin), (xmax, ymax), (255,255,255), thickness=-1)
    maskimg.astype(np.uint8)

    pub_leftimage.publish(imageleft.astype(np.uint8))
    pub_rightimage.publish(imageright.astype(np.uint8))
    pub_maskimage.publish(maskimg)
    rospy.loginfo("mask generated")


def efidet():
    rospy.loginfo("start efidet rospy!")
    rospy.init_node('efidet', anonymous=True)
    pub_leftimage = rospy.Publisher('imagel_seman', Image, queue_size=1)
    pub_rightimage = rospy.Publisher('imager_seman', Image, queue_size=1)
    pub_maskimage = rospy.Publisher('maskimage', Image, queue_size=1)
    model = torch.load("/home/jiang/Desktop/bao/efficientDet/efficientdet/trained_models/signatrix_efficientdet_coco.pth").module
    sub_leftimage = message_filters.Subscriber('imagel', Image)
    sub_rightimage = message_filters.Subscriber('imager', Image)
    ts = message_filters.ApproximateTimeSynchronizer([sub_leftimage, sub_rightimage], 1, 0.01, allow_headerless=True)#queue length, max time diff
    ts.registerCallback(myCallBack, (pub_leftimage, pub_rightimage, pub_maskimage, model))
    rospy.spin()

if __name__=='__main__':
    efidet()
