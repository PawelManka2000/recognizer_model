import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from recognizer.drivers.SerialDrv import SerialDrvTimeout
from recognizer.manager.MotorsControllerManager import UpdateMotorsError, MotorsControllerManager


class EncoderJointStatePublisher(Node):

    def __init__(self, motors_controller_manager: MotorsControllerManager):
        super().__init__('encoder_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Utwórz obiekt JointState raz
        self.joint_state = JointState()
        self.joint_state.name = ['left_back_wheel_joint', 'left_front_wheel_joint',
                                 'right_back_wheel_joint', 'right_front_wheel_joint']

        self.motors_controller_manager = motors_controller_manager


    def timer_callback(self):

        try:
            self.motors_controller_manager.update_motors_attributes()
        except UpdateMotorsError as e:
            print(e)
            return

        motors_driver = self.motors_controller_manager.motors_driver

        breakpoint()
        self.joint_state.position = [random.uniform(-1, 1) for _ in range(4)]
        
        self.joint_state.position = motors_driver.motors_position
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
