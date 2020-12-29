#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image,CameraInfo


img=0
bridge = cv_bridge.CvBridge()

def image_callback(msg):
    global bridge, img, cx,isRight,isLeft
    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

def video_reader():
    global img
    # cam = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    while True:
        data, bbox, _ = detector.detectAndDecode(img)
        if data:
            print("QR Code detected-->", data)
            break
        cv2.imshow("img", img)    
    # if cv2.waitKey(1) == ord("Q"):
    #     break
    # cam.release()
    cv2.destroyAllWindows()
    



def main():
    global img
    rospy.init_node('line_follower')
    img_sub = rospy.Subscriber('/mybot/camera/image_raw',Image,image_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    #rospy.sleep(3)

    while not rospy.is_shutdown():
        video_reader()

if __name__=='__main__':
    main()