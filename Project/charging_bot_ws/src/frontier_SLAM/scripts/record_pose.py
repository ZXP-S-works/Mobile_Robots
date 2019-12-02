#!/usr/bin/env python

"""
Record position and orientation, write into route.yaml file
Author: ZXP
Time: 1st, Dec, 2019
"""

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class RecordPose:

    def __init__(self):
        rospy.init_node('record_pose', anonymous=True)
        self.pose_subscriber = rospy.Subscriber\
            ('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)
        self.rate = rospy.Rate(5)
        self.pose = Pose()
        rospy.sleep(1)

    def update_pose(self, data):
        self.pose = data.pose.pose

    def record(self, filename):
        with open("../config/observe_pose.yaml", 'r') as stream:
            data_map = yaml.load(stream)
        if not data_map:
            data_map = []
        # {filename: '.png', position: { x:, y:}, quaternion: {r1:, r2:, r3:, r4:}}
        print(self.pose)
        data_map.append({'filename': filename,
                         'position': {'x': self.pose.position.x,
                                      'y': self.pose.position.y,
                                      'z': self.pose.position.z},
                         'quaternion': {'r1': self.pose.orientation.x,
                                        'r2': self.pose.orientation.y,
                                        'r3': self.pose.orientation.z,
                                        'r4': self.pose.orientation.w}})

        with open("../config/observe_pose.yaml", 'w') as stream:
            yaml.dump(data_map, stream)


if __name__ == '__main__':
    try:
        r = RecordPose()
        r.record('PoseNow')
    except rospy.ROSInterruptException:
        pass
