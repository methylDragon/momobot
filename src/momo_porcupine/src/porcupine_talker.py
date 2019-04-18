#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys

sys.path.append('/Porcupine/demo/python')
from porcupine_demo import PorcupineDemo

def talker():
    pub = rospy.Publisher('momo_emotion',String)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(0.1) #0.1Hz

    while not rospy.is_shutdown():
        test_str = "test"
        rospy.loginfo(test_str)
        pub.publish(test_str)
        rate.sleep()


if __name__ == '__main__' :
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
