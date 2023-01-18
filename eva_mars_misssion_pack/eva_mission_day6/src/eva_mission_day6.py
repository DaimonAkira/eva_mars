#!/usr/bin/env python3

# EVA MARS MISSION ROBOT - MISSON DAY 6 
# SILK ROAD EXPLORATION MISSION


import rospy
from geometry_msgs.msg import Twist, Vector3, Point
from operator import itemgetter
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion  
import math

# Initial Parameters

GOAL_X = 0.0
GOAL_Y = 0.0
ROBOT_POSE_X = 0.0
ROBOT_POSE_Y = 0.0
ROBOT_POSE_ORIENTATION = 0.0

GOAL_LIST = []
GOAL_AMOUNT = 3

def odom_callback(odom):
    
    global ROBOT_POSE_ORIENTATION, ROBOT_POSE_X, ROBOT_POSE_Y

    # Yaw calculations
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    ROBOT_POSE_ORIENTATION = yaw
    ROBOT_POSE_X = odom.pose.pose.position.x
    ROBOT_POSE_Y = odom.pose.pose.position.y

def appVel(g_x,g_y):

    msg = Twist()
    goal = Point()
    goal.x = g_x
    goal.y = g_y
        
    inc_x = goal.x - ROBOT_POSE_X
    inc_y = goal.y - ROBOT_POSE_Y
    angle_to_goal = math.atan2(inc_y, inc_x)

    if abs(angle_to_goal - ROBOT_POSE_ORIENTATION) > 0.1:
        msg.linear.x = 0.0
        msg.angular.z = 0.5
    else:
        msg.linear.x = 0.1
        msg.angular.z = 0.0
    return msg

def routeCreator():
    
    global GOAL_LIST, GOAL_AMOUNT

    new_routes = GOAL_LIST
    
    new_routes = sorted(new_routes, key= itemgetter(2))
    

    return new_routes


def main():

    global GOAL_X, GOAL_Y, GOAL_AMOUNT, GOAL_LIST

    rospy.init_node("eva_mission_day6")
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)

    r = rospy.Rate(4)
    msg = Twist()

    for i in range(GOAL_AMOUNT):
    
        GOAL_X = float(input("Enter goal pos x: "))
        GOAL_Y = float(input("Enter goal pos y: "))
        distance_from_goal = int(math.sqrt(math.pow(GOAL_X - ROBOT_POSE_X,2) + math.pow(GOAL_Y - ROBOT_POSE_Y,2)))
        GOAL_LIST.append((GOAL_X, GOAL_Y, distance_from_goal))    
        i += 1

    GOAL_LIST = routeCreator()

    while not rospy.is_shutdown():
    
        for i in range(GOAL_AMOUNT):

            (GOAL_X,GOAL_Y,dist) = GOAL_LIST[i]
            msg = appVel(GOAL_X,GOAL_Y)
            cmd_vel_pub.publish(msg)
            rospy.loginfo("Linear: %.2f, Angular: %.2f\n X: %.2f Y: %.2f th:%.2f", msg.linear.x, msg.angular.z, ROBOT_POSE_X, ROBOT_POSE_Y, ROBOT_POSE_ORIENTATION)
            print(i)
            if GOAL_X == ROBOT_POSE_X and GOAL_Y == ROBOT_POSE_Y:
                i +=1
            
    rospy.spin()
    r.sleep()
main()

