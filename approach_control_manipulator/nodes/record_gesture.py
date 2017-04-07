#!/usr/bin/env python

import rospy
import yaml
import os.path
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable


def main(args):
    pass


if __name__ == '__main__':
    rospy.init_node('record_geture', anonymous = True)
    main(sys.argv)
    try:
        rospy.spin()
    except KeboardInterrupt:
        rospy.loginfo('Shutting down node record_gesture')