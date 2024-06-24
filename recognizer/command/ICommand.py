from abc import ABC


class ICommand(ABC):

    def execute(self):
        pass