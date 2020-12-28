#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
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
wrongTurn = 0

def image_callback(msg):
    global bridge, img, cx,isRight,isLeft,wrongTurn
    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    crop_img = img[280:320,160:480]

    gray = cv2.cvtColor(crop_img,cv2.COLOR_RGB2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)

    _,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY)

    _,thresh_turns = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)

    _,contours,_ = cv2.findContours(thresh.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    _,contours_turn,_ = cv2.findContours(thresh_turns.copy(),1,cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        isLeft = False
        isRight = False
        wrongTurn=0
        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
        cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)

        center=[]

    #     if len(contours)==2:
    #         moments = cv2.moments(contours[1])
    #         if moments['m00']>900:
    #             print("contour area: ",moments['m00'])
    #             wrongx = int(moments['m10']/moments['m00'])
    #             wrongy = int(moments['m01']/moments['m00'])
    #             cv2.circle(crop_img,(wrongx,wrongy),5,(72,125,255),-1)
    #             if wrongx<cx:
    #                 wrongTurn=1

        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            try:
                turnx = int(moments['m10']/moments['m00'])
                turny = int(moments['m01']/moments['m00'])
                center.append((turnx,turny))
                cv2.circle(crop_img,center[-1],5,(0,0,255),-1)
                # if(turny<55 and 20<=turnx<=140):
                #     # print("turnx:",turnx)
                #     # print("turny:",turny)
                #     isLeft = True
                # if(turny<55 and 180<=turnx<=300):
                #     # print("turnx:",turnx)
                #     # print("turny:",turny)
                #     isRight = True
            except ZeroDivisionError:
                pass
        
    #     if wrongTurn ==1:
    #         most_left_centroid_index = 0
    #         index = 0

        most_left_centroid_index = 0
        index=0
        min_x_value = 320

        for y in center:
            cx = y[0]
            if cx <= min_x_value:
                min_x_value = cx
                most_left_centroid_index = index
            index+=1

        try:
            cx = center[most_left_centroid_index][0]
            cy = center[most_left_centroid_index][1]

            cv2.circle(crop_img,(int(cx),int(cy)),5,(0,255,0),-1)

        except ZeroDivisionError:
            pass
        
       # linefollower()
    # else:
    #     global cmd_vel_pub
    #     msg = Twist()
    #     msg.linear.x=0
    #     msg.angular.z = 0.6
    #     cmd_vel_pub.publish(msg)


    cv2.imshow("real img",crop_img)
    #cv2.imshow("gray",gray)
    #cv2.imshow("blur",blur)
    #cv2.imshow("thresh",thresh)

    cv2.waitKey(3)

def linefollower():
    global cx, cmd_vel_pub
    msg = Twist()

    # turn left
    if cx <= 140:
        msg.linear.x=0.2
        msg.angular.z=2
    # Straight
    elif 140<cx<175:
        msg.linear.x=0.3
        msg.angular.z=0
    # turn right
    elif cx>=175:
        msg.linear.x=0.2
        msg.angular.z=-2

    cmd_vel_pub.publish(msg)

# def wrongTurn(code):
#     global cx,cmd_vel_pub
#     msg = Twist()
#     if(code<cx):
#         msg.angular.z=1
#     cmd_vel_pub.publish(msg)
    

        


def main():
    rospy.init_node('line_follower')
    global bridge,img_sub,cmd_vel_pub,isLeft,isRight
    bridge = cv_bridge.CvBridge()
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.sleep(3)
    #img_sub = rospy.Subscriber('/mybot/camera/image_raw',Image,image_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

   
    while not rospy.is_shutdown():
        go_straight(1)
    

if __name__=='__main__':
    main()