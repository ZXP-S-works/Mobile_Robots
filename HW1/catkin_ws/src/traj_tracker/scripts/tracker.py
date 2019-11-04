#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from math import pow, atan2, sqrt, pi, cos, sin


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=2)

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

    def set_error_pen(self, desire_pose):
        rospy.wait_for_service('/turtle1/set_pen')
        error_norm = self.euclidean_distance(desire_pose, self.pose)
        # rospy.loginfo("error: " + str(error_norm))
        error_norm = min(error_norm * 500, 255)
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        try:
            set_pen(255, 255 - error_norm, 255 - error_norm, 5, 0)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def set_no_pen(self):
        rospy.wait_for_service('/turtle1/set_pen')
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        try:
            set_pen(0, 0, 0, 5, 1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def set_init_pose(self, x, y, theta):
        self.set_no_pen()
        rospy.wait_for_service('/turtle1/teleport_absolute')
        set_pose = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        try:
            set_pose(x, y, theta)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def plot_config_hist(self, config_hist):
        plt.plot(np.arange(0, len(config_hist[0])), config_hist[0])
        plt.plot(np.arange(0, len(config_hist[0])), config_hist[1])
        plt.plot(np.arange(0, len(config_hist[0])), config_hist[2])
        plt.legend(["x", "y", "theta"])
        plt.xlabel("time step")
        plt.ylabel("1 or rad")
        plt.title("Robot Configuration")
        plt.show()

    def stop(self):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal(self):
        """Tracker main func"""
        # Set the initial position of the turtle
        # for _ in np.arange(0, 30):
        #     self.rate.sleep()
        # self.set_init_pose(5.5, 5.5, 0)    # center
        # self.set_init_pose(0, 11, 0)    # top-left
        # self.set_init_pose(11, 11, 0)    # top-right
        # self.set_init_pose(0, 0, 0)    # bottom-left
        # self.set_init_pose(11, 0, 0)    # bottom-right
        self.set_init_pose(1, 1, pi/2.2)    # for 'M R'

        # Moves the turtle to follow the trajectory in traj_*.yaml
        desire_pose = Pose()
        desire_pose_next = Pose()
        desire_pose_next_next = Pose()
        vel_msg = Twist()
        config_hist = [[], [], []]
        rospy.loginfo('timestep is: ' + str(self.timestep))
        rospy.loginfo('rate is: ' + str(1 / self.timestep))
        tic = rospy.get_time()

        for loop in np.arange(0, 1):
            rospy.loginfo("********** Now is the loop " + str(loop) + ". **********")

            for t in np.arange(0, len(self.desire_traj['list_of_x']) - 2):
                # calculate pose, speed for control
                # rospy.loginfo(t)
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
                if t % 20 == 0:
                    rospy.loginfo('[x, y] = ' + str(desire_pose.x)
                                  + ', ' + str(desire_pose.y))
                # rospy.loginfo('desire theta ' + str(desire_pose.theta))
                desire_velocity = self.euclidean_distance(desire_pose, desire_pose_next) \
                     / self.timestep
                d_desire_theta = (desire_pose_next.theta - desire_pose.theta)
                if d_desire_theta < -pi:
                    d_desire_theta += 2 * pi
                if d_desire_theta > pi:
                    d_desire_theta -= 2 * pi
                desire_omega = d_desire_theta \
                          / self.timestep

                # control parameters
                # method1
                epsilon = 2.5    # performs good
                a = 5.5    # performs good
                # epsilon = 5
                # a = 5.5
                # epsilon = 5
                # a = 2.75
                # epsilon = 1.25
                # a = 5.5
                k1 = 2 * epsilon * a
                k2 = (a ** 2 - desire_omega ** 2) \
                     / (desire_velocity + 1e-5)
                k3 = k1
                # method2
                # k1 = 1    # performs good
                # k2 = 0.5    # performs good
                # k1 = 6.5
                # k2 = 2
                # k3 = k1

                # calculate velocity and omega
                theta = desire_pose.theta
                r_theta = np.array([[cos(theta),   sin(theta),  0],
                                    [-sin(theta),  cos(theta),  0],
                                    [0,            0,           1]])
                d_desire_pose_theta = desire_pose.theta - self.pose.theta
                if d_desire_pose_theta < -pi:
                    d_desire_pose_theta += 2 * pi
                if d_desire_pose_theta > pi:
                    d_desire_pose_theta -= 2 * pi
                dq = np.array([[desire_pose.x - self.pose.x],
                               [desire_pose.y - self.pose.y],
                               [d_desire_pose_theta]])
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

                # Record the robot configuration (x, y, theta)
                config_hist[0].append([self.pose.x])
                config_hist[1].append([self.pose.y])
                config_hist[2].append([self.pose.theta])

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Call set_pen service
                # Uncomment for 'MR' trajactory
                if (t >= 72 * 4 - 1) and (t <= 72 * 4 + 22):
                    self.set_no_pen()
                else:
                    self.set_error_pen(desire_pose)

                # Publish at the desired rate.
                self.rate.sleep()

        toc = rospy.get_time()
        rospy.loginfo('Time consumption: ' + str(toc - tic) + 's')

        # Stopping our robot after the movement is over.
        self.stop()

        # Plot the robot configuration along time
        self.plot_config_hist(config_hist)

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
