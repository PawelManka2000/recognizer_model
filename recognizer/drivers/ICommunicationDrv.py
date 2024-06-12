from abc import abstractmethod, ABC


class ICommunicationDrv(ABC):

    def send(self):
        pass 

    def receive(self):
        pass 

    def start_communication(self):
        pass 

