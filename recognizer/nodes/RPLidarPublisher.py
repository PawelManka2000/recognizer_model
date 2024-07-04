import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import time


class RPLidarPublisher(Node):
    def __init__(self):
        super().__init__('rplidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.serial_port = '/dev/serial/by-path/pci-0000:29:00.3-usb-0:1:1.0-port0'
        self.frame_id = 'laser_frame'
        self.angle_compensate = True
        self.scan_mode = 'Standard'

        try:
            self.serial_connection = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info('Serial connection established')
            self.start_motor()
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            self.serial_connection = None

        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info('RPLidar Publisher Node has been started')

    def start_motor(self):
        self.serial_connection.setDTR(False)
        self.serial_connection.write(b'\xA5\xF0')  # Command to start motor

    def stop_motor(self):
        self.serial_connection.setDTR(True)
        self.serial_connection.write(b'\xA5\x25\x00')  # Command to stop motor

    def timer_callback(self):
        if self.serial_connection:
            scan_data = self.read_scan_data()
            if scan_data:
                laser_scan_msg = LaserScan()
                laser_scan_msg.header.stamp = self.get_clock().now().to_msg()
                laser_scan_msg.header.frame_id = self.frame_id

                angle_increment = 2.0 * 3.1415926535897 / len(scan_data)
                laser_scan_msg.angle_min = 0.0
                laser_scan_msg.angle_max = 2.0 * 3.1415926535897
                laser_scan_msg.angle_increment = angle_increment
                laser_scan_msg.range_min = 0.15
                laser_scan_msg.range_max = 6.0

                laser_scan_msg.ranges = scan_data

                self.publisher_.publish(laser_scan_msg)
                self.get_logger().info(f'Published scan data {scan_data}')

    def read_scan_data(self):
        try:
            self.serial_connection.write(b'\xA5\x20')
            time.sleep(0.1)
            raw_data = self.serial_connection.read(360 * 2)
            scan_data = [struct.unpack('<H', raw_data[i:i + 2])[0] / 1000.0 for i in range(0, len(raw_data), 2)]
            return scan_data
        except Exception as e:
            self.get_logger().error(f'Error reading scan data: {e}')
            return None
