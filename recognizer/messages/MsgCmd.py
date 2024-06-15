from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EMsgId import EMsgId


END_BYTE = 0xFE
PAD_BYTE = 0x00

class MsgCmd:

    def __init__(self, msg_id: EMsgId, cmd_code: ECmdCode, payload: []):

        self.msg_id = msg_id
        self.cmd_code = cmd_code
        self.payload = payload

    def __bytes__(self):

        target_length = 8
        hex_msg_lst = [self.msg_id, self.cmd_code] + self.payload + [END_BYTE]

        if len(hex_msg_lst) > target_length:
            raise TooLongMsgError(f"Payload length exceed {len(self.payload)}")

        missing_elements = target_length - len(hex_msg_lst)
        if missing_elements > 0:
            hex_msg_lst += [PAD_BYTE] * missing_elements

        return bytes(hex_msg_lst)

class TooLongMsgError(Exception):
    pass





