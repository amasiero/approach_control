#!/usr/bin/env python

import rospy
import tf
import numpy as np
from std_msgs.msg import Float64

def pub_distance():
	
	rospy.init_node('pub_distance', anonymous=True)
	
	pub = rospy.Publisher('/torso_distance', Float64, queue_size=10)
	transformer = tf.TransformListener()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		if transformer.frameExists('/torso_1'):
			try:
				(trans, rot) = transformer.lookupTransform('/openni_depth_frame', '/torso_1', rospy.Time(0))
				distance = np.linalg.norm(trans)
				pub.publish(distance)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				rospy.logerr(e)
				continue

		rate.sleep()


if __name__ == '__main__':
	try:
		pub_distance()
	except rospy.RosInterruptException:
		rospy.logwarn('Shutting down pub_distance')
		pass
