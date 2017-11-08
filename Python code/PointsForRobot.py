#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('learning_tf')
import rospy
import tf



if __name__ == '__main__':
    rospy.init_node('PointsForRobot')
    
	# Points for the for robot
    point1 = tf.TransformBroadcaster()
    point2 = tf.TransformBroadcaster()
    point3 = tf.TransformBroadcaster() 
    point4 = tf.TransformBroadcaster() 

    rate = rospy.Rate(10.0)



 
    while not rospy.is_shutdown():
        point1.sendTransform((1.5, -1.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/point1",
                         "/world")

        point2.sendTransform((1.5, -0.5, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/point2",
                         "/world")	

        point3.sendTransform((1.5, 0.5, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/point3",
                         "/world")	

        point4.sendTransform((1.5, 1.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/point4",
                         "/world")	

        rate.sleep()

