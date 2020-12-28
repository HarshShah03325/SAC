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

dist_to_be_maint=0.2

cmd_vel_pub = None
sensors= {
    'right':0.18,
    'front_right': 0,
    'front': 0,
    'front_left':0,
    'left':0.22,
}
# sensors = {
#     'right_1': 0.18,
#     'right_2': 0.2,
# }


def wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    # d1=sensors['right_1']
    # d2=sensors['right_2']

    # angle=d1-d2

    # dist_from_wall=(d1+d2)/2

    # angular_z=dist_from_wall+angle

    # msg.angular.z=-angular_z

    # msg.linear.x=0.2

    # cmd_vel_pub.publish(msg)

    # print("angle: {}".format(angle))
    print("right: {}, left: {}".format(sensors['right'],sensors['left']))
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
        # else:
        #     msg.angular.z=0
    print("left: {}, right: {}, angular: {}".format(sensors['left'],sensors['right'],angle))
    cmd_vel_pub.publish(msg)

def left_wall_follower():
    global sensors,cmd_vel_pub
    cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg=Twist()
    left=sensors['left']
    msg=Twist()
    msg.linear.x=0.5
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


def LaserScanProcess(msg):
    global sensors

    sensors = {
       "right" : msg.ranges[0],
        "front_right" : min(min(msg.ranges[144:287]),10),
        "front" : min(min(msg.ranges[288:431]),10),
        "front_left" : min(min(msg.ranges[432:575]),10),
        "left" :msg.ranges[719],
    }
    # print("right: {}, left: {}".format(sensors['right'],sensors['left']))
    # sensors = {
    #     'right_1': msg.ranges[20],
    #     'right_2': msg.ranges[60],
    # }


def main():
    rospy.init_node('wall_follower')
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    laser_sub = rospy.Subscriber("scan", LaserScan , LaserScanProcess)
    # rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        # rospy.sleep(3)
        right_wall_follower()
        # rate.sleep()

if __name__=='__main__':
    # while not rospy.is_shutdown():
    #     main()
    main()