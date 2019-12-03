#!/usr/bin/env python

"""
The script for the whole process of charging, which include:
- Autonomous SLAM to built the world map
- Car detection for locating a car in the img by SSD
- Record way-points
- User sends way-points command, the robot navigate to theses points
Author: ZXP
Date: 2th, Dec, 2019
"""

import rospy
import roslaunch
import yaml
from take_photo import TakePhoto
from record_pose import RecordPose
from go_to_specific_point_on_map import GoToPose
from sensor.msg import Image


class ChargingBotLaunch():

    def __init__(self):
        rospy.init_node('car_detection', anonymouse=True)
        self.car_detection_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.car_detection)
        self.rate_car_detection = rospy.Rate(1/120)     #1/120 Hz, very slow for debugging
        self.rate_command_navigation = rospy.Rate(1/2)
        self.record_pose = RecordPose
        self.navigator = GoToPose()
        self.camera = TakePhoto()

    def auto_SLAM(self):
        success = False
        rospy.init_node('auto_SLAM', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent\
            (uuid, ["/opt/ros/kinetic/share/explore_lite/launch/explore.launch"])
        launch.start()
        rospy.loginfo("started")
        rospy.sleep(15*60)  # wait for 15 min for auto SLAM
        launch.shutdown()

    def car_detection(self, img):
        # convert img to cv2 img
        # $rosmsg show sensor_msgs/Image
        now = rospy.get_rostime()
        self.record_pose.record(str(now.secs))
        self.rate_car_detection.sleep()

    def command_navigation(self):
        # Read information from yaml file
        with open("../config/observe_pose.yaml", 'r') as stream:
            dataMap = yaml.load(stream)

        # Print recorded cars
        print_poses = raw_input("Print present cars? [Y/N] ")
        if print_poses == 'Y' or print_poses == 'y':
            for obj in dataMap:
                print(obj['filename'] + '\t')

        # Chose a car
        command_num = raw_input("Please chose a car to charge: ")
        if command_num > len(dataMap):
            return
        command_pose = dataMap[command_num]

        # Navigate to the car
        success = self.navigator.goto\
            (command_pose['position'], command_pose['quaternion'])
        if not success:
            rospy.loginfo("Failed to reach %s pose", command_pose['filename'])
        else:
            rospy.loginfo("Reached %s pose", command_pose['filename'])

        # Take a photo
        if self.take_picture(command_pose['filename']):
            rospy.loginfo("Saved image " + command_pose['filename'])
        else:
            rospy.loginfo("No images received")

        self.rate_command_navigation.sleep()

    def back_base(self):
        # Define the origin
        position = {'x': 0, 'y': 0}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}

        # Navigate to the origin
        success = self.navigator.goto \
            (position, quaternion)
        if not success:
            rospy.loginfo("Failed to reach the origin")
        else:
            rospy.loginfo("Reached the origin")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    def shutdown(self):
        self.back_base()
        # add more to shutdown safely


if __name__ == '__main__':
    try:
        x = ChargingBotLaunch()
        x.car_detection()
        x.auto_SLAM()
        # if auto_SLAM_success:
        #     rospy.loginfo("Auto-SLAM succeed!")
        # else:
        #     rospy.loginfo("Please build map manually.")
        #     '''
        #     add code for finishing manual SLAM
        #     '''
        x.command_navigation()
    except rospy.ROSInterruptException:
        pass
