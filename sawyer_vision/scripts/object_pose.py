#!/usr/bin/env python

import rospy
import sys
import cv2
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time
# save numpy array as npy file
from numpy import asarray
from numpy import save
# load numpy array from npy file
from numpy import load

def BGR_depth_point_callback(ros_image, depth_image, point_cloud):
    # Convert the ROS image to OpenCV format
    bridge = CvBridge()
    try:
      frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
      frame_depth = bridge.imgmsg_to_cv2(depth_image, "32FC1")
    except CvBridgeError, e:
      print e
     
    frame = np.array(frame, dtype=np.uint8)
    print('bgr imgs is:', frame)
    frame_depth = np.array(frame_depth, dtype=np.float32)
    # save to npy file
    save('/home/nalindas9/Desktop/data.npy', frame_depth)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    thresh, thresh_img = cv2.threshold(gray, 100,255,cv2.THRESH_BINARY)
    cv2.imshow('Segmented Image', thresh_img)
    # load array
    data = load('/home/nalindas9/Desktop/data.npy')
    # print the array
    print('The depth data is:', data)
    #cv2.normalize(frame_depth, frame_depth, 0, 1, cv2.NORM_MINMAX)
    
    img, x, y = process_image(frame)
    gen = point_cloud2.read_points(point_cloud, field_names=("x","y","z"), skip_nans=False, uvs = [(241,142)])
    time.sleep(1)
    #print ('The Z value at (x,y) is:', list(gen))
    #lent = 0
    #for p in gen:
      #lent = lent+1
      #print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    cv2.imshow('RGB Image', frame)
    #frame_depth = cv2.circle(frame_depth, (x,y), 5, (255,0,0), 2)
    cv2.imshow('Depth Image', frame_depth)
    #print('Depth:', frame_depth[x][y])
    cv2.imshow('RGB Image processsed', img)
    #print('Depth imgs is:', frame_depth)
    cv2.waitKey(0)

def process_image(frame):
  areas = {}
  grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  
  blur = cv2.blur(grey, (7,7))
    
  edges = cv2.Canny(grey, 15.0, 30.0)
  
  cv2.imshow('Edges', edges)
  ret,thresh = cv2.threshold(edges,127,255,0)
  image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  image_color = np.dstack((image,image,image))*255
  for c in contours:
    # find bounding box coordinates
    x,y,w,h = cv2.boundingRect(c)
    if 15>=abs(w-h)>=0:
      areas[(x,y,w,h)] = w*h
      #cv2.rectangle(image_color, (x,y), (x+w, y+h), (0, 255, 0), 2)
  
  print (areas)
  size = max(areas, key=areas.get)
  print(size)
  cv2.rectangle(image_color, (size[0],size[1]), (size[0]+size[2], size[1]+size[3]), (0, 255, 0), 2)
  x_center, y_center = size[0]+(size[2])/2, size[1]+(size[3])/2
  print('(X,Y) coordinates', (x_center, y_center))
  return image_color, x_center, y_center
    
def main():
  rospy.init_node('cv_bridge')
  #image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
  image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
  depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
  point_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)
  sync = message_filters.TimeSynchronizer([image_sub, depth_sub, point_sub], 1)
  sync.registerCallback(BGR_depth_point_callback)
  rospy.loginfo("Waiting for the image topics ...")
  rospy.spin()
  
if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    print "Shutting down vision node"
    cv2.destroyAllWindows()
  
