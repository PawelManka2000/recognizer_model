import struct

from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EMotorID import EMotorID
from recognizer.enums.EMsgId import EMsgId


class MsgState:

    MSG_ID_LEN = 1
    MSG_ID_OFFSET = 0

    MOTOR_ID_OFFSET = 0
    VELOCITY_OFFSET = 1
    POSITION_OFFSET = 3

    SINGLE_STATE_LENGTH = 7
    NO_OF_STATES = 4

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
        msg_id = int(msg_state[MsgState.MSG_ID_OFFSET])
        position_dict = {}
        velocity_dict = {}

        for i in range(MsgState.NO_OF_STATES):

            it = i * MsgState.SINGLE_STATE_LENGTH + MsgState.MSG_ID_LEN
            e_motor_id = int(msg_state[it])
            velo_num = int(msg_state[it + MsgState.VELOCITY_OFFSET])
            velo_fr = int(msg_state[it + MsgState.VELOCITY_OFFSET + 1])
            velo = velo_num + velo_fr / 100

            pos_value_bytes = msg_state[it + MsgState.POSITION_OFFSET: it + MsgState.SINGLE_STATE_LENGTH]
            pos_int_value = struct.unpack('>i', pos_value_bytes)[0]

            velocity_dict[EMotorID(e_motor_id)] = velo
            position_dict[EMotorID(e_motor_id)] = float(pos_int_value)

        return MsgState(EMsgId(msg_id), position_dict, velocity_dict)
