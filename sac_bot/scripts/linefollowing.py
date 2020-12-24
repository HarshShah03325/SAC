#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image,CameraInfo

bridge = cv_bridge.CvBridge()
img_sub = 0
cmd_vel_pub = 0
img=0
cx=0

def image_callback(msg):
    global bridge, img, cx
    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    crop_img = img[200:320,160:480]

    gray = cv2.cvtColor(crop_img,cv2.COLOR_RGB2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)

    ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY)

    _,contours,_ = cv2.findContours(thresh.copy(),1,cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:

        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
        cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
        
        linefollower()

    cv2.imshow("real img",crop_img)
    # cv2.imshow("gray",gray)
    # cv2.imshow("blur",blur)
    # cv2.imshow("thresh",thresh)

    cv2.waitKey(3)

def linefollower():
    global cx, cmd_vel_pub
    print(cx)
    msg = Twist()

    # turn left
    if cx <= 140:
        msg.linear.x=0.05
        msg.angular.z=0.6
    # Straight
    elif 140<cx<175:
        msg.linear.x=0.2
        msg.angular.z=0
    # turn right
    elif cx>=175:
        msg.linear.x=0.05
        msg.angular.z=-0.6

    cmd_vel_pub.publish(msg)



def main():
    rospy.init_node('line_follower')
    global bridge,img_sub,cmd_vel_pub
    bridge = cv_bridge.CvBridge()
    img_sub = rospy.Subscriber('/mybot/camera/image_raw',Image,image_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    while not rospy.is_shutdown():
        linefollower()

if __name__=='__main__':
    main()