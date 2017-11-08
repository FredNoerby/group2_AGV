#!/usr/bin/env python

import rospy
from random import randint
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('robot_inventory', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.3) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        hello_str = (str(randint(0, 6)) + " " + str(randint(0, 6)))
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
