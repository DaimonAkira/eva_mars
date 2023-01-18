#!/usr/bin/env python3

# EVA MARS MISSION ROBOT - MISSON DAY 1 
# LANDING AND FIRST EXPLORE 

import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion  
import math

#Initial Parameters

X_POS_NOW = 11.0
Y_POS_NOW = -11.0
ANGLE_NOW = 0.0
X_LAST = 0.0
Y_LAST = 0.0
ANGLE_LAST = 0.0

go_count = 0
back_count = 0

B_ODOM = True

i_state = 0


def odom_callback(odom):
    
    global ANGLE_NOW, X_POS_NOW, Y_POS_NOW, B_ODOM

    # Yaw calculations
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    ANGLE_NOW = yaw
    X_POS_NOW = odom.pose.pose.position.x
    Y_POS_NOW = odom.pose.pose.position.y
    B_ODOM = False

    
def main():
    
    global ANGLE_LAST, X_LAST, Y_LAST,i_state
    global Y_POS_NOW, X_POS_NOW, ANGLE_NOW

    rospy.init_node("eva_mission_day1")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber("odom", Odometry, odom_callback)

    r = rospy.Rate(10)

    while not B_ODOM:
        rospy.spin()
        r.sleep()

    X_LAST = X_POS_NOW
    Y_LAST = Y_POS_NOW
    ANGLE_LAST = ANGLE_NOW

    msg = Twist()

    while not rospy.is_shutdown():
        
        if i_state == 0:
            msg = go_command()
        elif i_state == 1:
            msg = back_command()
            if go_count == 4 and back_count == 4:
                i_state = 2
        elif i_state == 2:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            rospy.loginfo("End of the first exploration sequence!")
            exit()
        else:
            rospy.loginfo("Error: Wrong state msg!")
    
        pub.publish(msg)
    rospy.spin()
    r.sleep()

def go_command():
    
    global X_LAST, Y_LAST, ANGLE_LAST,i_state, go_count

    X_CHANGE = X_POS_NOW - X_LAST
    Y_CHANGE = Y_POS_NOW - Y_LAST
    POS_CHANGE = math.sqrt( (X_CHANGE*X_CHANGE) + (Y_CHANGE*Y_CHANGE) )
    rospy.loginfo("Robot is going %.2f meter", POS_CHANGE)

    m = Twist()

    if(POS_CHANGE >= 2):
        i_state = 1
        X_LAST = X_POS_NOW
        Y_LAST = Y_POS_NOW
        ANGLE_LAST = ANGLE_NOW
        go_count+=1
        return m
    else:
        m.linear.x = 0.2        
        return m

def back_command():

    global X_LAST, Y_LAST, ANGLE_LAST,i_state,back_count
    
    TURN_CHANGE = math.fabs(ANGLE_NOW - ANGLE_LAST)
    rospy.loginfo("Robot is turning %f radian", TURN_CHANGE)

    m = Twist()

    
    if TURN_CHANGE >= 1.57:
        
        i_state = 0
        X_LAST = X_POS_NOW
        Y_LAST = Y_POS_NOW
        ANGLE_LAST = ANGLE_NOW
        back_count+=1
        return m
    else:
        
        m.angular.z = -0.2
        return m

main()
