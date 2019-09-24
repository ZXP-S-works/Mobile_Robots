#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi, cos, sin
import numpy as np


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.desire_traj = rospy.get_param('trajectory_description')
        self.timestep = self.desire_traj['timestep']
        self.rate = rospy.Rate(1 / self.timestep)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def euclidean_distance(self, pose1, pose2):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((pose1.x - pose2.x), 2) +
                    pow((pose1.y - pose2.y), 2))

    def move2goal(self):
        """Moves the turtle to the goal."""
        desire_pose = Pose()
        desire_pose_next = Pose()
        desire_pose_next_next = Pose()
        epsilon = 1.2
        a = 5
        vel_msg = Twist()
        rospy.loginfo('timestep is: ' + str(self.timestep))
        rospy.loginfo('rate is: ' + str(1 / self.timestep))

        for loop in np.arange(0, 2):
            rospy.loginfo("Now is the loop " + str(loop) + " .")

            for t in np.arange(0, len(self.desire_traj['list_of_x']) - 2):
                # calculate pose, speed for control
                rospy.loginfo(t)
                desire_pose.x = self.desire_traj['list_of_x'][t]
                desire_pose.y = self.desire_traj['list_of_y'][t]
                desire_pose_next.x = self.desire_traj['list_of_x'][t+1]
                desire_pose_next.y = self.desire_traj['list_of_y'][t+1]
                desire_pose_next_next.x = self.desire_traj['list_of_x'][t+2]
                desire_pose_next_next.y = self.desire_traj['list_of_y'][t+2]
                dy = (desire_pose_next.y - desire_pose.y)
                dx = (desire_pose_next.x - desire_pose.x)
                dy_next = (desire_pose_next_next.y - desire_pose_next.y)
                dx_next = (desire_pose_next_next.x - desire_pose_next.x)
                desire_pose.theta = atan2(dy, dx)
                if desire_pose.theta < 0:
                    desire_pose.theta += 2 * pi
                desire_pose_next.theta = atan2(dy_next, dx_next)
                if desire_pose_next.theta < 0:
                    desire_pose_next.theta += 2 * pi
                rospy.loginfo('[x, y] = ' + str(desire_pose.x) + ' ' + str(desire_pose.y))
                rospy.loginfo('desire theta ' + str(desire_pose.theta))
                desire_velocity = self.euclidean_distance(desire_pose, desire_pose_next) \
                     / self.timestep
                desire_omega = (desire_pose_next.theta - desire_pose.theta) \
                          / self.timestep

                # control parameters
                k1 = 2 * epsilon * a
                k2 = (a ** 2 - desire_omega ** 2) \
                     / (desire_velocity + 1e-5)
                k3 = k1

                # calculate velocity and omega
                theta = desire_pose.theta
                r_theta = np.array([[cos(theta),   sin(theta),  0],
                                    [-sin(theta),  cos(theta),  0],
                                    [0,            0,           1]])
                dq = np.array([[desire_pose.x - self.pose.x],
                               [desire_pose.y - self.pose.y],
                               [desire_pose.theta - self.pose.theta]])
                error = r_theta.dot(dq)
                u1 = -k1 * error[0]
                u2 = -k2 * error[1] - k3 * error[2]
                velocity = desire_velocity * cos(error[2]) - u1
                omega = desire_omega - u2

                # Linear velocity in the x-axis.
                vel_msg.linear.x = velocity
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = omega

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass