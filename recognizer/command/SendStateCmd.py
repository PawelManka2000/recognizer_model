from recognizer.command.ICommand import ICommand
from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EMsgId import EMsgId
from recognizer.messages.MsgCmd import MsgCmd


class SendStateCmd(ICommand):

    def __init__(self, comm_driver):
        self.comm_drv = comm_driver

    def execute(self):
        payload = []
        msg_cmd = MsgCmd(EMsgId.MsgCmdReq.value, ECmdCode.MOTORS_STATE.value, payload)
        self.comm_drv.send_raw_msg(bytes(msg_cmd))

        return self.comm_drv.receive_response()
