from enum import Enum


class ECmdCode(Enum):

    UNKNOWN = 0x00
    MOTORS_STATE = 0x01
    CTRL_VELO = 0x02
    PWM_DRIVING = 0x03
