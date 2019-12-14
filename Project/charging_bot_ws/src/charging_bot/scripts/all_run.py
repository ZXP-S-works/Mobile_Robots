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
# from record_pose import RecordPose
from go_to_specific_point_on_map import GoToPose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class ChargingBotLaunch:

    def __init__(self):
        rospy.init_node('car_detection', anonymous=True)
        self.navigator = GoToPose()
        self.camera = TakePhoto()
        self.car_detection_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.car_detection)
        self.rate_car_detection = rospy.Rate(1./120.)     #1/120 Hz, very slow for debugging
        self.rate_command_navigation = rospy.Rate(1./2.)
        self.pose_subscriber = rospy.Subscriber\
            ('/amcl_pose', PoseWithCovarianceStamped, self.__update_pose__)
        self.rate = rospy.Rate(5)
        self.pose = Pose()

        # Allow up to one second to connection
        rospy.sleep(1)

    def __update_pose__(self, data):
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

    def auto_SLAM(self):
        # rospy.init_node('auto_SLAM', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent\
            (uuid, ["/opt/ros/kinetic/share/explore_lite/launch/explore.launch"])
        launch.start()
        rospy.loginfo("-------------------Auto-SLAM started-----------------")
        rospy.sleep(20*60)  # wait for 20 min for auto SLAM
        rospy.loginfo('Please save the map: $rosrun map_server map_saver -f '
                      '/home/zxp-s-works/Desktop/Mobile_Rob/Project/map/garage_map.yaml')
        rospy.sleep(5*60)
        launch.shutdown()
        self.back_base()

    def car_detection(self, img):
        img =img
        # convert img to cv2 img
        # $rosmsg show sensor_msgs/Image
        # presently record pos every 5 min to simulate SSD detecting cars
        now = rospy.get_rostime()
        self.record(str(now.secs))
        self.rate_car_detection.sleep()

    def init_navigation(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent\
            (uuid, ["/home/zxp-s-works/Desktop/Mobile_Rob/Project/charging_bot_ws/"
                    "src/charging_bot/launch/init_navigation.launch"])
        launch.start()
        rospy.loginfo("---------------AMCL navigation initialized------------")

    def command_navigation(self):
        # Read information from yaml file
        with open("../config/recorded_pose.yaml", 'r') as stream:
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
        if self.camera(command_pose['filename']):
            rospy.loginfo("Saved image " + command_pose['filename'])
        else:
            rospy.loginfo("No images received")

        # Move back to base
        command_move = raw_input("Move back to the base [B] or move to the next car [N]? ")
        if command_move == 'B':
            self.back_base()
        else:
            pass

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
        x.auto_SLAM()
        # if auto_SLAM_success:
        #     rospy.loginfo("Auto-SLAM succeed!")
        # else:
        #     rospy.loginfo("Please build map manually.")
        #     '''
        #     add code for finishing manual SLAM
        #     '''
        x.init_navigation()
        x.back_base()
        while True:
            x.command_navigation()

    except rospy.ROSInterruptException:
        pass
