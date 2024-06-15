from abc import abstractmethod, ABC


class ICommunicationDrv(ABC):

    def send(self, raw_msg: bytes):
        pass 

    def receive(self):
        pass 

    def start_communication(self):
        pass 


class CommunicationDrvError(Exception):
    pass