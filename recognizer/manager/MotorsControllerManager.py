from recognizer.drivers.ICommunicationDrv import ICommunicationDrv
from recognizer.enums.ECmdCode import ECmdCode
from recognizer.enums.EMsgId import EMsgId
from recognizer.enums.EOmniDirModeId import EOmniDirModeId
from recognizer.drivers.SerialDrv import SerialDrv
from recognizer.enums.EMsgId import EMsgId
from recognizer.messages.MsgCmd import MsgCmd



class MotorsControllerManager:

    def __init__(self, comm_driver: ICommunicationDrv, debug_serial_cmds=True):

        self.comm_drv = comm_driver
        self.debug_serial_cmds = debug_serial_cmds

    def send_pwm_motor_command(self, omnidir_mode: EOmniDirModeId, pwm: int):

        payload = [omnidir_mode.value, pwm]
        msg_cmd = MsgCmd(EMsgId.MsgCmdReq.value, ECmdCode.CTRL_VELO.value, payload)
        resp = self.send_raw_command(bytes(msg_cmd))

        return resp

    def send_ctrl_velo_motor_command(self, omnidir_mode: EOmniDirModeId, velocity: float):

        decimal_expansion_multiplier = 100
        velo_whole_num = int(velocity)
        velo_fraction_num = int((velocity - velo_whole_num) * decimal_expansion_multiplier)

        payload = [omnidir_mode.value, velo_whole_num, velo_fraction_num]
        msg_cmd = MsgCmd(EMsgId.MsgCmdReq.value, ECmdCode.CTRL_VELO.value, payload)
        resp = self.send_raw_command(bytes(msg_cmd))

        return resp

    def send_encoder_read_command(self):

        payload = []
        msg_cmd = MsgCmd(EMsgId.MsgCmdReq.value, ECmdCode.MOTORS_STATE.value, payload)
        resp = self.send_raw_command(bytes(msg_cmd))

        return resp

    def send_raw_command(self, msg_cmd_raw: bytes):

        self.comm_drv.send(msg_cmd_raw)
        if (self.debug_serial_cmds):
            print(f"Sent: {msg_cmd_raw}")
        self.comm_drv.receive_response()
