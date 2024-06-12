import rclpy 
from recognizer.drivers.MotorDriver import MotorDriver
from recognizer.enums.EDrivingModeId import EDrivingModeId
from recognizer.recognizer_gui import main_gui


def main(args=None):


    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()
        # motor_driver.send_ctrl_velo_motor_command(EDrivingModeId.FORWARD, 3.5)


    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
