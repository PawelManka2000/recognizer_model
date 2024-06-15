from enum import Enum

class EMsgId(Enum):

    UNKNOWN = 0x00
    MsgCmdReq = 0x01
    MsgCmdResp = 0x02
    State = 0x03