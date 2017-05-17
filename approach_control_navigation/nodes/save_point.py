#!/usr/bin/env python
import rospy
import yaml
import os.path as path
from geometry_msgs.msg import PoseWithCovarianceStamped
from judith_utils.Gamepad import Gamepad
from sensor_msgs.msg import Joy

class SavePoint(object):

    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.processGamepad, queue_size = 1)
        self.fname = path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals_saved.yaml'
        stream = open(self.fname, 'r')
        self.data = yaml.load(stream)
        if self.data is not None:
            self.keys = self.data.keys()
        else:
            self.data = []
        self.newKey = 0


    def processGamepad(self, joyMsg):
        self.controller = Gamepad(joyMsg)

        if self.controller.x:
            self.newKey += 1
            if self.data:
                self.data[self.newKey] = [['funfou', 'mesmo']]
            else:
                self.data = dict([(self.newKey, [['funfou', 'mesmo']])])
            with open(self.fname, 'w') as yaml_file:
                yaml_file.write(yaml.dump(self.data, default_flow_style = False))
            rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node('save_point')
    SavePoint()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")