#!/usr/bin/env python3

# EVA MARS MISSION ROBOT - MISSON DAY II 
# EXPANDED FIELD EXPLORING AND MANEUVERABILITY TEST 


import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion  
import math
import random 

#Initial Parameters

X_POS_NOW = -1.0
Y_POS_NOW = -5.0
ANGLE_NOW = 0.0
X_LAST = 0.0
Y_LAST = 0.0
ANGLE_LAST = 0.0

go_count = 0
turn_count = 0

B_ODOM = True

i_state = 0


def odom_callback(odom):
    
    global ANGLE_NOW, X_POS_NOW, Y_POS_NOW, B_ODOM

    # yaw calculations
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

    rospy.init_node("eva_mission_day2")
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
            msg = turn_command()
            x = random.randint(3,10)
            #Random progression numbers from 3-10 are assigned.
            if go_count == x and turn_count == x:
                i_state = 2
        elif i_state == 2:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            rospy.loginfo("End of the expended exploration sequence! You can go back to initial position.")
            break
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
    rospy.loginfo("Route %d: Robot is going %.2f meter",go_count, POS_CHANGE)

    m = Twist()

    go = random.randint(2, 4)
    if(POS_CHANGE >= go):
        i_state = 1
        X_LAST = X_POS_NOW
        Y_LAST = Y_POS_NOW
        ANGLE_LAST = ANGLE_NOW
        go_count+=1
        return m
    else:
        
        m.linear.x = 0.5      
        return m

def turn_command():

    global X_LAST, Y_LAST, ANGLE_LAST,i_state,turn_count
    
    TURN_CHANGE = math.fabs(ANGLE_NOW - ANGLE_LAST)
    rospy.loginfo("Turn %d: Robot is turning %f radian", turn_count, TURN_CHANGE)

    m = Twist()

    turn = random.uniform(0.5,1.5)

    if TURN_CHANGE >= turn:
        
        i_state = 0
        X_LAST = X_POS_NOW
        Y_LAST = Y_POS_NOW
        ANGLE_LAST = ANGLE_NOW
        turn_count+=1
        return m
    else:
        
        
        m.angular.z = 0.5
        return m

main()

