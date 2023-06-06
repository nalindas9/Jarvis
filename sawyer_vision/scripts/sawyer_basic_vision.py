#!/usr/bin/env python

import rospy
import sys
import cv2
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Class for CvBridge Functions

class cvBridgeDemo():
  def __init__(self):
    self.node_name = "cv_bridge"
    
    #Initializing the ROS node
    rospy.init_node(self.node_name)
    
    #During shutdown
    rospy.on_shutdown(self.cleanup)
    
    # Creating the OpenCV display window for the RGB Image
    self.cv_window_name = self.node_name
    #cv2.imshow('RGB image:', self.cv_window_name)
    #cv.MoveWindow(self.cv_window_name, 25, 75)
    
    # Creating the OpenCV display window for the depth image
    #cv.imshow("Depth Image", cv.CV_WINDOW_NORMAL)
    #cv.MoveWindow("Depth Image", 25, 350)
    
    # Create the cv_bridge object
    self.bridge = CvBridge()
    
    # Subscribe to the camera image and depth topic and set the callback
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
    #self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
    
    rospy.loginfo("Waiting for the image topics ...")
    
  def image_callback(self, ros_image):
    # Convert the ROS image to OpenCV format
    
    try:
      frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError, e:
      print e
     
    frame = np.array(frame, dtype=np.uint8)
    
    img = self.process_image(frame)
  
    
    cv2.imshow(self.node_name, img)
    
    cv2.waitKey(0)
  
  def depth_callback(self, ros_image):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
    except CvBridgeError, e:
      print e
      
    frame1 = np.array(depth_image, dtype=np.float32)
    
    cv2.normalize(frame1, frame1, 0, 1, cv2.NORM_MINMAX)
    
    img1 = self.process_depth_image(frame1)
    
    cv2.imshow("Depth image", img1)

    cv2.waitKey(0)
  
  def process_image(self, frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    blur = cv2.blur(grey, (7,7))
    
    edges = cv2.Canny(grey, 15.0, 30.0)
    
    return edges
    
  def process_depth_image(self, frame):
    return frame
    
  def cleanup(self):
    print "Shutting down vision node"
    cv2.destroyAllWindows()
    

def main(args):
  try:
    cvBridgeDemo()
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down vision node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
  main(sys.argv)
  

