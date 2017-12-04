#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.x = -3.5
        self.y = 12.0
        self.yaw = 0.0
        self.current_angular_velocity = 0.0
        self.last_cmd_stamp = None
        self.current_cmd_stamp = None
        self.v = 0.0

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback)  # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, self.cmd_callback)

    def cmd_callback(self, data):
        self.current_cmd_stamp = rospy.Time.now()

        if self.last_cmd_stamp is None:
            self.last_cmd_stamp = self.current_cmd_stamp

        dt = self.current_cmd_stamp.to_sec() - self.last_cmd_stamp.to_sec()
        delta_x = data.drive.speed * math.cos(self.yaw) * dt * 0.5 #adjustable
	#delta_x = data.drive.speed * math.cos(self.yaw) * dt * 1.0 
        delta_y = data.drive.speed * math.sin(self.yaw) * dt * 0.5 #adjustable
	#delta_y = data.drive.speed * math.sin(self.yaw) * dt * 1.0

        self.current_angular_velocity = data.drive.speed * math.tan(data.drive.steering_angle) / 1.1 #adjustable
	#self.current_angular_velocity = data.drive.speed * math.tan(data.drive.steering_angle) / 1.0

        self.x += delta_x
        self.y += delta_y
        self.yaw += self.current_angular_velocity * dt
        self.v = data.drive.speed * 0.5 #adjustable
	#self.v = data.drive.speed * 1.0

        self.last_cmd_stamp = self.current_cmd_stamp

    def timer_callback(self, event):
        if self.last_cmd_stamp is None:
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.current_angular_velocity

        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = odom.child_frame_id
        t.transform.translation = odom.pose.pose.position
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_pub.sendTransform(t)


# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node_prj4")
    node = OdometryNode()
    rospy.spin()
