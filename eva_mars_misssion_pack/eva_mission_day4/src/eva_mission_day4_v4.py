#!/usr/bin/env python3

# EVA MARS MISSION ROBOT - MISSON DAY 4 
# EXPLORING TO THE ROCKY LAKE

#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None

def main():
    global pub

    rospy.init_node('eva_mission_day4')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('laser_scan', LaserScan, clbk_laser)

    rospy.spin()

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:71]), 10),
        'fright': min(min(msg.ranges[72:143]), 10),
        'front':  min(min(msg.ranges[144:215]), 10),
        'fleft':  min(min(msg.ranges[216:287]), 10),
        'left':   min(min(msg.ranges[288:359]), 10),
    }

    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > 1.0 and regions['fleft'] > 1.0 and regions['fright'] > 1.0:
        state_description = 'case 1 - nothing'
        linear_x = 0.05
        angular_z = 0
    elif regions['front'] < 1.0 and regions['fleft'] > 1.0 and regions['fright'] > 1.0:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] > 1.0 and regions['fleft'] > 1.0 and regions['fright'] < 1.0:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] > 1.0 and regions['fleft'] < 1.0 and regions['fright'] > 1.0:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] < 1.0 and regions['fleft'] > 1.0 and regions['fright'] < 1.0:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] < 1.0 and regions['fleft'] < 1.0 and regions['fright'] > 1.0:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] < 1.0 and regions['fleft'] < 1.0 and regions['fright'] < 1.0:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] > 1.0 and regions['fleft'] < 1.0 and regions['fright'] < 1.0:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0
        angular_z = -0.1
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

if __name__ == '__main__':
    main()