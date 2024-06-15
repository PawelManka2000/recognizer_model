import rclpy
from rclpy.node import Node

from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EMsgId import EMsgId
from recognizer.messages.MsgCmd import MsgCmd
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals

import time
import math
import serial
from threading import Lock
from recognizer.drivers.ICommunicationDrv import ICommunicationDrv
from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EOmniDirModeId import EOmniDirModeId


class MotorDriver(Node):

    def __init__(self):

        super().__init__('motor_driver')

        # Setup parameters
        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")

        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")


        self.declare_parameter('serial_debug', value=True)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value

        if (self.debug_serial_cmds):
            print("Serial debug enabled")

        # Setup topics & services

        # self.subscription = self.create_subscription(
        #     MotorCommand,
        #     'motor_command',
        #     self.motor_command_callback,
        #     10)
        #
        # self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)
        #
        # self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        #

        # Member Variables
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()

    # Raw serial commands


    def parse_motors_state(self, resp):

        motor_states_dict = {}
        if resp:

            resp_splited = resp.split('\n\r')
            motors_states_tab = resp_splited[:-1]

            for motor_state in motors_states_tab:
                motor_state_splited = motor_state.split(',')
                motor_id = EMotorId(int(motor_state_splited[0]))
                motor_states_dict[motor_id] = motor_state_splited[1:]
            print(resp)

    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if motor_command.is_pwm:
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            self.send_ctrl_velo_motor_command(EOmniDirModeId.FORWARD.value, 3.5)
