import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

class RobotStatePublisherNode(Node):

    def __init__(self, xacro_file):


        state_publisher_params = {'robot_description': robot_description_config}

        super().__init__(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[state_publisher_params]
        )

        robot_description_config = Command(['xacro ', xacro_file])
        self.declare_parameter('description', value=robot_description_config)

        # Tworzenie publishera na temat /motor_status, publikujący std_msgs/String
        self.pub = self.create_publisher(String, '/motor_status', 10)

        # Częstotliwość publikacji danych (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_motor_status)

    def publish_motor_state(self):
        motor_state = random.choice(['running', 'stopped', 'idle', 'error'])

        # Publikowanie wiadomości ROS
        msg = String()
        msg.data = motor_state
        self.pub.publish(msg)

        self.get_logger().info(f"Published motor state: {motor_state}")




# import rospy
# from std_msgs.msg import String
# import random
#
# class MotorStatusPublisherNode(rospy.Node):
#     def __init__(self):
#         super().__init__('motor_status_publisher')  # Inicjalizacja klasy bazowej (rospy.Node)
#
#         # Tworzenie publishera na temat /motor_status, publikujący std_msgs/String
#         self.pub = self.create_publisher(String, '/motor_status', 10)
#
#         # Częstotliwość publikacji danych (1 Hz)
#         self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_motor_status)
#
#     def publish_motor_status(self, event):
#         # Generowanie losowego stanu silnika (np. "running", "stopped" itp.)
#         motor_state = random.choice(['running', 'stopped', 'idle', 'error'])
#
#         # Publikowanie wiadomości ROS
#         msg = String()
#         msg.data = motor_state
#         self.pub.publish(msg)
#
#         rospy.loginfo(f"Published motor state: {motor_state}")