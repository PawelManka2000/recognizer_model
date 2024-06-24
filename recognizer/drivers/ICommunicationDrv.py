from abc import abstractmethod, ABC


class ICommunicationDrv(ABC):

    def send_raw_msg(self, raw_msg: bytes):
        pass 

    def receive_response(self):
        pass 

    def start_communication(self):
        pass 


class CommunicationDrvError(Exception):
    pass