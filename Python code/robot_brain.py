#!/usr/bin/env python
import roslib; roslib.load_manifest('group2_agv')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

    left_speed = msg.linear.x * 1250 + msg.angular.z * -1250
    right_speed = msg.linear.x * 1250 + msg.angular.z * 1250

    pub = rospy.Publisher('motor_control', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    data = Float32MultiArray()
    data.data = [left_speed, right_speed]
    rospy.loginfo(data)
    pub.publish(data)
    rate.sleep()


def listener():
    rospy.init_node('robot_brain', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
