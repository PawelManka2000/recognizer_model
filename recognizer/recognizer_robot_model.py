import os
import threading

import rclpy

from recognizer.control_panel.recognizer_gui import SpeedControlApp
from recognizer.drivers.MotorsDriver import MotorsDriver
from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EOmniDirModeId import EOmniDirModeId
from recognizer.manager.MotorsControllerManager import MotorsControllerManager
from recognizer.nodes.OdometryPublisher import OdometryPublisher
from recognizer.nodes.RobotStatePublisher import JointStatePublisher

serial_port = "/dev/stmG4"
baud_rate = 115200
serial_timeout = 1


motors_driver = MotorsDriver()
serial_drv = SerialDrv(serial_port, baud_rate, serial_timeout)
serial_drv.start_communication()
motors_controller_manager = MotorsControllerManager(serial_drv, motors_driver)


def ros2_main(args=None):

    # TODO this should be in the configuration file

    rclpy.init(args=args)
    # motors_controller_manager.send_ctrl_velo_motor_command(EOmniDirModeId.FORWARD, 3.5)
    executor = rclpy.executors.MultiThreadedExecutor()
    motor_joint_state_publisher = JointStatePublisher(motors_controller_manager)
    odometry_publisher_node = OdometryPublisher(motors_driver)

    try:

        executor.add_node(motor_joint_state_publisher)
        executor.add_node(odometry_publisher_node)
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        motor_joint_state_publisher.destroy_node()
        odometry_publisher_node.destroy_node()
        serial_drv.close_conn()
        executor.shutdown()
        rclpy.shutdown()

def main():
    ros2_thread = threading.Thread(target=ros2_main)
    ros2_thread.start()

    app = SpeedControlApp(motors_controller_manager)
    app.mainloop()

    ros2_thread.join()

if __name__ == '__main__':
    main()


