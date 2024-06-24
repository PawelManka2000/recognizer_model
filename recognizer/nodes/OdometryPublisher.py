import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.odom_trans_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        # Update robot's position
        self.x += 0.1
        self.y += 0.1
        self.theta += 0.1

        # Create the odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_quat = self.quaternion_from_euler(0, 0, self.theta)

        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        # Send the transform
        self.odom_trans_broadcaster.sendTransform(odom_trans)

        # Create the odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]

        self.publisher_.publish(msg)
        print('Publishing: "%s"' % msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]
