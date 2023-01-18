#!/usr/bin/env python3


# EVA MARS MISSION ROBOT - MISSON DAY VII 
# TIME TO DRILLING


import rospy
import time
from std_msgs.msg import Float64

control_param = True
def sondaj_control():

    global control_param
    pub_sondaj = rospy.Publisher('eva_mars/sondaj_joint_position_controller/command', Float64, queue_size=10)
    pub_sondaj2 = rospy.Publisher('eva_mars/sondaj2_joint_position_controller/command', Float64, queue_size=10)
    pub_sondaj3 = rospy.Publisher('eva_mars/sondaj3_joint_position_controller/command', Float64, queue_size=10)
    
    rospy.init_node('eva_mission_day7')
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        control_str = input("For Drilling, click 'd' button. For end drilling, click 'e'. For exit, click 'x' ")
        if control_param:
            if control_str == "d":
                rospy.loginfo("Drilling...")
                pub_sondaj.publish(float(-0.04))
                rate.sleep()
                pub_sondaj2.publish(float(-0.10))
                rate.sleep()
                pub_sondaj3.publish(float(-0.24))
                control_param = False
            elif control_str == "x":
                rospy.loginfo("Terminated..")
                break
            else:
                rospy.loginfo("You cannot end before starting!")
            rate.sleep()
        else:
            if control_str == "e":
                rospy.loginfo("Finishing...")
                pub_sondaj3.publish(float(0.0))
                rate.sleep()
                pub_sondaj2.publish(float(0.0))
                rate.sleep()
                pub_sondaj.publish(float(0.0))
                control_param = True
            elif control_str == "x":
                rospy.loginfo("Terminated..")
                break
            else:
                rospy.loginfo("You cannot start, before ending!")
            rate.sleep()


if __name__ == '__main__':
    try:
        sondaj_control()
    except rospy.ROSInterruptException:
        pass
