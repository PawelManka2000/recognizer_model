from enum import Enum


class ECmdId(Enum):

    UNKNOWN = 0
    STATE_REQ = 1
    CTRL_VELO_REQ = 2
    PWM_DRIVING_REQ = 3
