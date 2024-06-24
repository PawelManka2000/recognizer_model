import queue
import time

from threading import Thread

from recognizer.command.ICommand import ICommand


class CommandInvokerThread(Thread):
    def __init__(self):
        super().__init__()
        self.command_queue = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:

                command = self.command_queue.get(timeout=0.1)
                self.command_queue.task_done()
                return command.execute()

            except queue.Empty:
                pass
            time.sleep(0.1)

    def add_cmd(self, cmd: ICommand):
        pass

    def stop(self):
        self.running = False