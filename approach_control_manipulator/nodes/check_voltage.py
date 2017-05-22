#!/usr/bin/env python
import rospy
from dynamixel_msgs.msg import MotorState, MotorStateList

class CheckVoltage(object):
    def __init__(self):
        rospy.Subscriber('/motor_states/pan_tilt_port', MotorStateList, self.motor_states_callback)

    def motor_states_callback(self, states):
        if (states.motor_states[0].voltage < 16.6):
            rospy.logerr('Low Battery: %.1f' % states.motor_states[0].voltage)


if __name__ == '__main__':
    rospy.init_node('checking_voltage')
    CheckVoltage()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")