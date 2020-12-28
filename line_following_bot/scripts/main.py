#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image,CameraInfo

from functions import*

bridge = cv_bridge.CvBridge()
img_sub = 0
cmd_vel_pub = 0
img=0
cx=0
isLeft = False
isRight = False

dist_to_be_maint=0.2

cmd_vel_pub = None
sensors= {
    'right':0.5,
    'front_right': 0,
    'front': 0,
    'front_left':0,
    'left':0.5,
}

def image_callback(msg):
    global bridge, img, cx,isRight,isLeft
    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    crop_img = img[290:320,160:480]

    gray = cv2.cvtColor(crop_img,cv2.COLOR_RGB2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)

    _,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY)

    _,thresh_turns = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)

    _,contours,_ = cv2.findContours(thresh.copy(),1,cv2.CHAIN_APPROX_NONE)

    _,contours_turn,_ = cv2.findContours(thresh_turns.copy(),1,cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        isLeft = False
        isRight = False
        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
        cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)

        center=[]
        linefollower()
    # else:
    #     global cmd_vel_pub
    #     msg = Twist()
    #     msg.linear.x=0
    #     msg.angular.z = 0.6
    #     cmd_vel_pub.publish(msg)


    cv2.imshow("real img",crop_img)
    cv2.waitKey(3)

def LaserScanProcess(msg):
    global sensors

    sensors = {
       "right" : msg.ranges[0],
        "front_right" : min(min(msg.ranges[144:287]),10),
        "front" : min(min(msg.ranges[288:431]),10),
        "front_left" : min(min(msg.ranges[432:575]),10),
        "left" :msg.ranges[719],
    }

def linefollower():
    global cx, cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    print(cx)
    msg = Twist()

    # turn left
    if cx <= 140:
        msg.linear.x=1
        msg.angular.z=5
    # Straight
    elif 140<cx<175:
        msg.linear.x=1
        msg.angular.z=0
    # turn right
    elif cx>=175:
        msg.linear.x=1
        msg.angular.z=-5

    cmd_vel_pub.publish(msg)


def wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    right=sensors['right']
    left=sensors['left']
    msg=Twist()
    msg.linear.x=0.5
    angle=((right+left)**2-(0.41**2))
    if angle>0:
        angle=angle**0.5
        angle=math.atan(angle/0.41)
        if right>left:
            msg.angular.z=-angle
        elif left>right:
            msg.angular.z=angle
    print("left: {}, right: {}, angular: {}".format(sensors['left'],sensors['right'],angle))
    cmd_vel_pub.publish(msg)

def left_wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    left=sensors['left']
    msg=Twist()
    msg.linear.x=1
    if left<0.2:
        angle=-1
    elif left>0.2:
        angle=1
    print("left: {},angle :{}".format(left,angle))
    msg.angular.z=angle
    cmd_vel_pub.publish(msg)

def right_wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    right=sensors['right']
    msg=Twist()
    msg.linear.x=1
    if right<0.2:
        angle=1
    elif right>0.2:
        angle=-1
    print("right: {},angle :{}".format(right,angle))
    msg.angular.z=angle
    cmd_vel_pub.publish(msg)




def main():
    global sensors
    rospy.init_node('wall_follower',anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    img_sub = rospy.Subscriber('/mybot/camera/image_raw',Image,image_callback)
    laser_sub = rospy.Subscriber("scan", LaserScan , LaserScanProcess)
    rospy.sleep(3)
    
    while not rospy.is_shutdown():
        if sensors['right']<0.4:
            right_wall_follower()
        elif sensors['left']<0.4:
            left_wall_follower()
        else:
            linefollower()


if __name__=='__main__':
    main()