#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

g_range_ahead = 1 #anything to start
scan = rospy.Subscriber('scan',LaserScan,scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        if (g_range_ahead < 0.3 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now()+rospy.Duration(1)
    else:
        if rospy.Time.now() > state_change_time:
            driving_forward = True # stop spinning ,time to go forward
            state_change_time = rospy.Time.now() + rospy.Duration(30)
    twist = Twist()
    if driving_forward:
        twist.linear.x = -0.1
    else:
        twist.angular.z = 0.2


    cmd_vel_pub.publish(twist)

    rate.sleep()
