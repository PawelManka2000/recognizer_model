import os


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command

import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import time

import rclpy
from recognizer.drivers.MotorDriver import MotorDriver
from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EOmniDirModeId import EOmniDirModeId
from recognizer.manager.MotorsControllerManager import MotorsControllerManager
from recognizer.nodes.RobotStatePublisher import RobotStatePublisherNode
from recognizer.recognizer_gui import main_gui


def create_robot_description_node():

    pkg_path = os.path.join(get_package_share_directory('recognizer'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot_core.xacro')

    robot_description_config = Command(['xacro ', xacro_file])

    state_publisher_params = {'robot_description': robot_description_config}

    recognizer_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[state_publisher_params]
    )
    # return LaunchDescription([
    #
    #     recognizer_state_publisher
    #
    # ])

    return recognizer_state_publisher


def main(args=None):
    # TODO this should be in the configuration file
    serial_port = "/dev/stmG4"
    baud_rate = 115200
    serial_timeout = 1

    rclpy.init(args=args)
    motor_driver = MotorDriver()

    serial_drv = SerialDrv(serial_port, baud_rate, serial_timeout)
    serial_drv.start_communication()

    motors_controller_manager = MotorsControllerManager(serial_drv)

    rate = motor_driver.create_rate(2)
    motors_controller_manager.send_ctrl_velo_motor_command(EOmniDirModeId.FORWARD, 3.5)

    pkg_path = os.path.join(get_package_share_directory('recognizer'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot_core.xacro')

    robot_desc_node = RobotStatePublisherNode(xacro_file)
    rclpy.spin(robot_desc_node)

    while rclpy.ok():

        rclpy.spin_once(motor_driver)
        motors_controller_manager.send_encoder_read_command()
        time.sleep(0.5)

    serial_drv.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
