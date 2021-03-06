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
    crop_img = img[280:320,160:480]

    gray = cv2.cvtColor(crop_img,cv2.COLOR_RGB2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)

    _,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY)

    _,thresh_invert = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)

   # _,contours_turn,_ = cv2.findContours(thresh_invert.copy(),1,cv2.CHAIN_APPROX_NONE)

    img_left = thresh[:,:106]
    img_middle = thresh[:,106:213]
    img_right = thresh[:,213:320]

    if(cv2.countNonZero(img_left)> 2400 and cv2.countNonZero(img_middle)<1800 and cv2.countNonZero(img_right)>2400):
        #print("black line")
        thresh = thresh_invert

    _,contours,_ = cv2.findContours(thresh.copy(),1,cv2.CHAIN_APPROX_NONE)
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

        
    else:
        cx=-1
        cy=-1
    
    stop_img = img[160:320,40:300]
    gray_stop = cv2.cvtColor(stop_img,cv2.COLOR_RGB2GRAY)
    blur_stop = cv2.GaussianBlur(gray_stop,(5,5),0)
    _,thresh_stop = cv2.threshold(blur_stop,60,255,cv2.THRESH_BINARY)
    count = cv2.countNonZero(thresh_stop)
    print(count)
    if 13500<count<14300 and len(contours)==1 and 3000<cv2.countNonZero(thresh_stop[:,120:180])<3400:
        print(count)
        #print("middle",cv2.countNonZero(thresh_stop[:,120:180]))
        print("stop")


    cv2.imshow("real img",crop_img)

    cv2.waitKey(3)


def LaserScanProcess(msg):
    global sensors

    sensors = {
       "right" : min(min(msg.ranges[0:150]),10),
        "front_right" : min(min(msg.ranges[144:287]),10),
        "front" : min(min(msg.ranges[288:431]),10),
        "front_left" : min(min(msg.ranges[432:575]),10),
        "left" :min(min(msg.ranges[580:720]),10),
    }

def linefollower():
    global cx, cmd_vel_pub
    cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg = Twist()

    # turn left
    if cx <= 20:
        msg.linear.x=0.2
        msg.angular.z=5
    elif cx <=50:
        msg.linear.x=0.2
        msg.angular.z=4.5
    elif cx <=70:
        msg.linear.x=0.2
        msg.angular.z=4
    elif cx<=90:
        msg.linear.x=0.2
        msg.angular.z=3
    elif cx<=110:
        msg.linear.x=0.3
        msg.angular.z=2.5
    elif cx<=130:
        msg.linear.x=0.4
        msg.angular.z=2
    elif cx<=150:
        msg.linear.x=0.6
        msg.angular.z=1.5
    # Straight
    elif 150<cx<170:
        msg.linear.x=0.8
        msg.angular.z=0
    # turn right
    elif 170<=cx<180:
        #print("cond1")
        msg.linear.x=0.6
        msg.angular.z=-1.5
    elif 180<=cx<200:
        #print("cond2")
        msg.linear.x=0.4
        msg.angular.z=-2
    elif 200<=cx<220:
        #print("cond3")
        msg.linear.x=0.3
        msg.angular.z=-2.5
    elif 220<=cx<250:
        #print("cond4")
        msg.linear.x=0.2
        msg.angular.z=-3
    elif 250<=cx<270:
        #print("cond5")
        msg.linear.x=0.2
        msg.angular.z=-4
    elif 270<=cx<300:
        #print("cond6")
        msg.linear.x=0.2
        msg.angular.z=-4.5
    elif cx>=300:
        #print("cond7")
        msg.linear.x=0.2
        msg.angular.z=-5

    cmd_vel_pub.publish(msg)


def recovery():
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    msg = Twist()
    msg.linear.x=0
    msg.angular.z = 3
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
    #print("left: {}, right: {}, angular: {}".format(sensors['left'],sensors['right'],angle))
    cmd_vel_pub.publish(msg)

def left_wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    left=sensors['left']
    msg=Twist()
    msg.linear.x=1
    if left<0.2:
        angle=-3
    elif left>0.2:
        angle=3
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
        angle=3
    elif right>0.2:
        angle=-3
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
        # if sensors['right']<0.4:
        #     right_wall_follower()
        # if sensors['left']<0.4:
        #     left_wall_follower()
        # elif cx>0:
        #     linefollower()
        # else :
        #     go_straight(0.3)
        # if False:
        #     print("true")
        if cx>0:
            linefollower()
        else:
            if sensors["left"]<0.4:
                left_wall_follower()
            elif sensors["right"]<0.4:
                right_wall_follower()
            else:
                recovery()

if __name__=='__main__':
    main()