import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command

import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import time

import rclpy
from recognizer.drivers.MotorsDriver import MotorsDriver
from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EOmniDirModeId import EOmniDirModeId
from recognizer.manager.MotorsControllerManager import MotorsControllerManager
from recognizer.nodes.RobotStatePublisher import EncoderJointStatePublisher
from recognizer.recognizer_gui import main_gui


# def create_robot_description_node():
#
#     pkg_path = os.path.join(get_package_share_directory('recognizer'))
#     xacro_file = os.path.join(pkg_path, 'description', 'robot_core.xacro')
#
#     robot_description_config = Command(['xacro ', xacro_file])
#
#     state_publisher_params = {'robot_description': robot_description_config}
#
#     recognizer_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[state_publisher_params]
#     )
#     # return LaunchDescription([
#     #
#     #     recognizer_state_publisher
#     #
#     # ])
#
#     return recognizer_state_publisher


def main(args=None):
    # TODO this should be in the configuration file
    serial_port = "/dev/stmG4"
    baud_rate = 115200
    serial_timeout = 1

    rclpy.init(args=args)
    motors_driver = MotorsDriver()

    serial_drv = SerialDrv(serial_port, baud_rate, serial_timeout)
    serial_drv.start_communication()

    motors_controller_manager = MotorsControllerManager(serial_drv, motors_driver)

    # motors_controller_manager.send_ctrl_velo_motor_command(EOmniDirModeId.FORWARD, 3.5)

    motor_joint_state_publisher = EncoderJointStatePublisher(motors_controller_manager)

    try:
        rclpy.spin(motor_joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        motor_joint_state_publisher.destroy_node()
        serial_drv.close_conn()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
