import struct

from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EMotorID import EMotorID
from recognizer.enums.EMsgId import EMsgId


class MsgState:
    # General message offsets
    MSG_ID_LEN = 1
    MSG_ID_OFFSET_BEGIN = 0

    # Single State payload offsets
    MOTOR_ID_OFFSET_BEGIN = 0
    MOTOR_ID_OFFSET_END = 1

    VELOCITY_OFFSET_BEGIN = 1
    VELOCITY_OFFSET_END = 3

    POSITION_OFFSET_BEGIN = 3
    POSITION_OFFSET_END = 7

    SINGLE_STATE_LENGTH = 7
    NO_OF_STATES = 4

    POSITION_DEC_EXPANSION = 2
    VELOCITY_DEV_EXPANSION = 2

    def __init__(self, msg_id: EMsgId, position_dict, velocity_dict):
        self.msg_id = msg_id
        self.__position_dict = position_dict
        self.__velocity_dict = velocity_dict

    @property
    def position_dict(self):
        return self.__position_dict

    @property
    def velocity_dict(self):
        return self.__velocity_dict

    @staticmethod
    def create_msg_state_from_raw(msg_state: bytes):
        msg_id = int(msg_state[MsgState.MSG_ID_OFFSET_BEGIN])
        position_dict = {}
        velocity_dict = {}

        for i in range(MsgState.NO_OF_STATES):
            it = i * MsgState.SINGLE_STATE_LENGTH + MsgState.MSG_ID_LEN
            e_motor_id = int(msg_state[it])
            velo_from_bytes = int.from_bytes(
                (msg_state[it + MsgState.VELOCITY_OFFSET_BEGIN: it + MsgState.VELOCITY_OFFSET_END]), byteorder='big')

            velo_float = velo_from_bytes / 100
            # breakpoint()
            pos_value_bytes = msg_state[it + MsgState.POSITION_OFFSET_BEGIN: it + MsgState.POSITION_OFFSET_END]
            pos_int = int.from_bytes(pos_value_bytes, byteorder='big', signed=True)
            pos_float = pos_int/ 100

            velocity_dict[EMotorID(e_motor_id)] = velo_float
            position_dict[EMotorID(e_motor_id)] = pos_float

        return MsgState(EMsgId(msg_id), position_dict, velocity_dict)
