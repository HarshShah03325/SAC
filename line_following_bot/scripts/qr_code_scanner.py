#! /usr/bin/env python

from __future__ import print_function
import rospy
import cv2
import cv_bridge
import numpy as np
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image,CameraInfo
import pyzbar.pyzbar as pyzbar

bridge = cv_bridge.CvBridge()
img_sub = 0
cmd_vel_pub = 0
img=0

def image_callback(msg):
  global bridge, img, cx
  img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
  gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
  _,thresh = cv2.threshold(gray,60,255,cv2.THRESH_BINARY)
  decodedObjects = decode(thresh)
  #display(img, decodedObjects)

# Display barcode and QR code location  
def display(im, decodedObjects):

  # Loop over all decoded objects
  for decodedObject in decodedObjects: 
    points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else : 
      hull = points
    
    # Number of points in the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

  # Display results 
  cv2.imshow("Results", im)
  cv2.waitKey(0)


def decode(im) : 
  # Find barcodes and QR codes
  decodedObjects = pyzbar.decode(im)
  print("check")
  print(decodedObjects)
  # Print results
  for obj in decodedObjects:
    print('Type : ', obj.type)
    print('Data : ', obj.data,'\n')
    
  return decodedObjects


def main():
  global sensors
  rospy.init_node('wall_follower',anonymous=True)
  cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
  img_sub = rospy.Subscriber('/mybot/camera/image_raw',Image,image_callback)
  #laser_sub = rospy.Subscriber("scan", LaserScan , LaserScanProcess)
  rospy.sleep(3)
    
  while not rospy.is_shutdown():
    if False:
      print("check")

if __name__=='__main__':
  main()