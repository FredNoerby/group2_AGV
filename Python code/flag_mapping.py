#!/usr/bin/env python 
import tf
import rospy


if __name__ == '__main__':
    rospy.init_node('flag_mapping')
    # Creates a transform listener used for getting flag location
    listener = tf.TransformListener()
    trans = []
    rot = []

    # Enters loop that will run until script or ROS is shutdown 
    while not rospy.is_shutdown():

        # Gets the transform and rotation from the maps zero position to the flag
        try:
            (trans, rot) = listener.lookupTransform('/map', '/vicon/Flag/Flag', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if trans and rot :
            print("Trans: ")
            print(trans) 
            print("Rot: ")
            print(rot)
        rospy.sleep(10)
