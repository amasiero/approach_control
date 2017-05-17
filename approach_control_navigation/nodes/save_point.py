#!/usr/bin/env python
import rospy
import yaml
import os.path as path
from geometry_msgs.msg import PoseWithCovarianceStamped
from judith_utils.Gamepad import Gamepad
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

class SavePoint(object):

    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.processGamepad, queue_size = 1)
        rospy.Subscriber('/pose', Odometry, self.getPose)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.getAmclPose)
        self.fname = path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals_saved.yaml'
        stream = open(self.fname, 'r')
        self.data = yaml.load(stream)
        if self.data is not None:
            self.keys = self.data.keys()
            self.newKey = self.keys[-1]
        else:
            self.data = {}
            self.newKey = 0

    def getPose(self, poseMsg):
        self.pose = poseMsg.pose.pose

    def getAmclPose(self, amclPoseMsg):
        self.amcl_pose = amclPoseMsg.pose.pose

    def processGamepad(self, joyMsg):
        self.controller = Gamepad(joyMsg)

        if self.controller.x:
            self.newKey += 1
            print(self.amcl_pose)
            print(self.pose)
            if self.data:
                self.data[self.newKey] = [[self.amcl_pose.position.x, self.amcl_pose.position.y], [0.0, 0.0, self.pose.orientation.z, self.pose.orientation.w]]
            else:
                self.data = dict([(self.newKey, [[self.amcl_pose.position.x, self.amcl_pose.position.y], [0.0, 0.0, self.pose.orientation.z, self.pose.orientation.w]])])
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