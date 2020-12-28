#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import math


pub=None
yaw_ = 0
current_yaw_=0


def clbk_odom(msg):
    global yaw_,current_yaw_
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    current_yaw_ = euler[2]


def yaw_error(target_yaw):
    global current_yaw_ ,yaw_
    yaw_ = target_yaw
    if(-(math.pi)<target_yaw<-3.1 or target_yaw<-(math.pi)):
        yaw_ = yaw_ + 2*math.pi
    if(yaw_>math.pi):
        yaw_ = yaw_ - 2*math.pi
    #print('current yaw: {} yaw: {}'.format(current_yaw_,yaw_))
    return (yaw_ - current_yaw_)

def rotate(degree,linear_velocity,angular_velocity):
    global yaw_,current_yaw_,pub
    angular_z = angular_velocity if degree>0 else -angular_velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    msg = Twist()
    turn_angle=degree*math.pi/180
    target_yaw = current_yaw_+turn_angle
    print("target_yaw: {} current_yaw:{}".format(target_yaw,current_yaw_))
    rospy.loginfo("turning by angle")
    yaw_error(target_yaw)
    while yaw_error(target_yaw)>0.02 or yaw_error(target_yaw)<-0.02:
        msg.angular.z= angular_z
        msg.linear.x=linear_velocity
        pub.publish(msg)
    msg.angular.z=0
    msg.linear.x=0
    pub.publish(msg)
    rospy.loginfo("turning successful")

def go_straight(linear_velocity):
    global pub
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    msg = Twist()
    msg.linear.x=linear_velocity
    pub.publish(msg)
