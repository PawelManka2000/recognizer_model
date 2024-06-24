import serial

from recognizer.drivers.ICommunicationDrv import ICommunicationDrv, CommunicationDrvError


class SerialDrv(ICommunicationDrv):

    def __init__(self, serial_port, baud_rate, timeout, debug_serial_cmds=False):

        self.__serial_conn = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.debug_serial_cmds = debug_serial_cmds

    def start_communication(self) -> None:

        if self.__serial_conn is None:

            print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
            self.__serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            print(f"Connected to {self.serial_conn}")

        else:
            raise CommunicationDrvError("Tried to restart already running SerialDrv")

    def close_conn(self):
        self.__serial_conn.close()

    def send_raw_msg(self, raw_msg: bytes):

        self.__serial_conn.write(raw_msg)

    def receive_response(self):

        if self.serial_conn:
            try:
                received_raw_msg = self.serial_conn.read_all()

                if received_raw_msg:
                    return received_raw_msg
                # TODO implement
                else:
                    raise SerialDrvTimeout("SerialDrv: Response was not received")
            except serial.SerialException as e:
                print(f'Error receiving response: {e}')
                return ''
        return None

    @property
    def serial_conn(self):
        if self.__serial_conn is None:
            raise CommunicationDrvError("Tried to get serial connection that was not defined")
        return self.__serial_conn


class SerialDrvTimeout(Exception):
    pass
