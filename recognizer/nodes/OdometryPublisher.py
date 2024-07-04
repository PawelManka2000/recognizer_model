from collections import deque

import rclpy
import math

import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from recognizer.drivers.MotorsDriver import MotorsDriver

from recognizer.enums.EMotorID import EMotorID

wheel_radius = 0.03
w = 0.135  # [m]
l = 0.14  # [m]
alpha = np.arctan(w / l)


class OdometryPublisher(Node):

    def __init__(self, motors_driver: MotorsDriver):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.odom_trans_broadcaster = TransformBroadcaster(self)
        self.dt = 0.5
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.motors_driver = motors_driver
        self.previous_motors_position = self.motors_driver.motors_position_dict

    def update_pose(self):
        angular_displacements = {}
        linear_displacements = {}
        linear_vel = {}
        vx = 0

        for e_motor_id in EMotorID:
            angular_displacements[e_motor_id] = self.motors_driver.motors_position_dict[
                                                    e_motor_id] - self.previous_motors_position[e_motor_id]
            # linear_displacements[e_motor_id] = angular_displacements[e_motor_id] * wheel_radius
            #
            if angular_displacements[e_motor_id]  > 0:
                linear_vel[e_motor_id] = self.motors_driver.motors_velocity_dict[e_motor_id]*wheel_radius/2
            else:
                linear_vel[e_motor_id] = -self.motors_driver.motors_velocity_dict[e_motor_id] * wheel_radius /2

            vx += (linear_vel[e_motor_id] / MotorsDriver.NO_OF_MOTORS)

        omega_z = ((linear_vel[EMotorID.RB] + linear_vel[EMotorID.RF]) - (
                linear_vel[EMotorID.LB] + linear_vel[EMotorID.LF])) * np.sin(alpha)

        # omega_z = ((linear_vel[EMotorID.RB] + linear_vel[EMotorID.RF]) - (
        #         linear_vel[EMotorID.LB] + linear_vel[EMotorID.LF])) / w

        self.x += vx * self.dt * np.cos(self.theta)
        self.y += vx * self.dt * np.sin(self.theta)
        self.theta += (omega_z * self.dt)
        self.previous_motors_position = self.motors_driver.motors_position_dict

    def timer_callback(self):

        self.update_pose()
        odom_trans_msg = TransformStamped()
        odom_trans_msg.header.stamp = self.get_clock().now().to_msg()
        odom_trans_msg.header.frame_id = 'odom'
        odom_trans_msg.child_frame_id = 'base_link'
        odom_trans_msg.transform.translation.x = self.x
        odom_trans_msg.transform.translation.y = self.y
        odom_trans_msg.transform.translation.z = 0.0
        odom_quat = self.quaternion_from_euler(0, 0, self.theta)

        odom_trans_msg.transform.rotation.x = odom_quat[0]
        odom_trans_msg.transform.rotation.y = odom_quat[1]
        odom_trans_msg.transform.rotation.z = odom_quat[2]
        odom_trans_msg.transform.rotation.w = odom_quat[3]


        self.odom_trans_broadcaster.sendTransform(odom_trans_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        self.publisher_.publish(odom_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
            pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
            pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
            pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
            pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]
