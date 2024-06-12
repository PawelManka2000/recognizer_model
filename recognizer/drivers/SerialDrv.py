from recognizer.drivers.ICommunicationDrv import ICommunicationDrv

class SerialDrv(ICommunicationDrv):

    def __init__():
        pass

    def send():

        pass

    def receive():
        pass

    def start_communication(self, conn_parameters : {}) -> None:

        self.declare_parameter('serial_port', value="/dev/ttyACM0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=115200)
        self.baud_rate = self.get_parameter('baud_rate').value


        



