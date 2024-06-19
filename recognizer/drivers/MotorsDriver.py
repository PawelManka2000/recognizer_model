import rclpy
from rclpy.node import Node

from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EMotorID import EMotorID
from recognizer.enums.EMsgId import EMsgId
from recognizer.messages.MsgCmd import MsgCmd
from recognizer.messages.MsgState import MsgState
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


class MotorsDriver:

    LB_MOTOR_POS = 0
    LF_MOTOR_POS = 1
    RB_MOTOR_POS = 2
    RF_MOTOR_POS = 3

    NO_OF_MOTORS = 4
    MOTORS_INITIAL_VAL = 0
    def __init__(self):

        self.__motors_velocity = [MotorsDriver.MOTORS_INITIAL_VAL] * MotorsDriver.NO_OF_MOTORS
        self.__motor_position = [MotorsDriver.MOTORS_INITIAL_VAL] * MotorsDriver.NO_OF_MOTORS

    @property
    def motors_position(self):
        return self.__motor_position

    @property
    def motors_velocity(self):
        return self.__motors_velocity

    def parse_motors_attributes(self, resp):

        msg_id_pos = 0
        if resp[msg_id_pos] == EMsgId.State.value:

            msg_state = MsgState.create_msg_state_from_raw(resp)
            self.motors_position[self.LB_MOTOR_POS] = msg_state.position_dict[EMotorID.LB]
            self.motors_velocity[self.LB_MOTOR_POS] = msg_state.velocity_dict[EMotorID.LB]

            self.motors_position[self.LF_MOTOR_POS] = msg_state.position_dict[EMotorID.LF]
            self.motors_velocity[self.LF_MOTOR_POS] = msg_state.velocity_dict[EMotorID.LF]

            self.motors_position[self.RB_MOTOR_POS] = msg_state.position_dict[EMotorID.RB]
            self.motors_velocity[self.RB_MOTOR_POS] = msg_state.velocity_dict[EMotorID.RB]

            self.motors_position[self.RF_MOTOR_POS] = msg_state.position_dict[EMotorID.RF]
            self.motors_velocity[self.RF_MOTOR_POS] = msg_state.velocity_dict[EMotorID.RF]

        else:
            raise ParsingError("Tried to parse not state msg")


class ParsingError(Exception):
    pass
