import rclpy 
from recognizer.drivers.MotorDriver import MotorDriver

def main(args=None):


    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()


    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()


# def main():
#     print("HIHIHI ")

if __name__ == '__main__':

    main()
