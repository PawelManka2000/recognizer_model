import tkinter as tk
from tkinter import ttk

from recognizer.enums.EOmniDirModeId import EOmniDirModeId
from recognizer.manager.MotorsControllerManager import MotorsControllerManager


class SpeedControlApp(tk.Tk):
    def __init__(self, motor_controller_manager: MotorsControllerManager):
        super().__init__()

        self.title("Robot Speed Control")
        self.geometry("300x400")

        # Speed slider and label
        self.speed_label = ttk.Label(self, text="Speed: 0.0")
        self.speed_label.pack(pady=10)

        self.speed_slider = ttk.Scale(self, from_=0.0, to=15.0, orient='horizontal', command=self.update_velo)
        self.speed_slider.pack(pady=10)

        # PWM slider and label
        self.pwm_label = ttk.Label(self, text="PWM Value: 0")
        self.pwm_label.pack(pady=10)

        self.pwm_slider = ttk.Scale(self, from_=0, to=100, orient='horizontal', command=self.update_pwm)
        self.pwm_slider.pack(pady=10)

        # Control buttons
        self.forward_button = ttk.Button(self, text="Forward", command=self.move_forward)
        self.forward_button.pack(pady=5)

        self.backward_button = ttk.Button(self, text="Backward", command=self.move_backward)
        self.backward_button.pack(pady=5)

        self.left_button = ttk.Button(self, text="Left", command=self.move_left)
        self.left_button.pack(pady=5)

        self.right_button = ttk.Button(self, text="Right", command=self.move_right)
        self.right_button.pack(pady=5)

        self.stop_button = ttk.Button(self, text="Stop", command=self.stop)
        self.stop_button.pack(pady=5)

        self.omnidir_mode = EOmniDirModeId.STOP
        self.motor_controller_manager = motor_controller_manager
        self.speed_ctrl_flag = False
        self.velo = 0.0
        self.pwm = 0

    def send_motor_ctrl_cmd(self):
        if self.speed_ctrl_flag:
            self.motor_controller_manager.send_ctrl_velo_motor_command(self.omnidir_mode, self.velo)
        else:
            self.motor_controller_manager.send_pwm_motor_command(self.omnidir_mode, self.pwm)

    def update_velo(self, event):
        self.velo = float(self.speed_slider.get())
        self.speed_ctrl_flag = True
        self.send_motor_ctrl_cmd()

    def update_pwm(self, event):
        self.pwm_value = int(self.pwm_slider.get())
        self.speed_ctrl_flag = False
        self.send_motor_ctrl_cmd()

    def move_forward(self):
        self.omnidir_mode = EOmniDirModeId.FORWARD
        self.send_motor_ctrl_cmd()

    def move_backward(self):
        self.omnidir_mode = EOmniDirModeId.BACKWARD
        self.send_motor_ctrl_cmd()

    def move_left(self):
        self.omnidir_mode = EOmniDirModeId.LEFT
        self.send_motor_ctrl_cmd()

    def move_right(self):
        self.omnidir_mode = EOmniDirModeId.RIGHT
        self.send_motor_ctrl_cmd()

    def stop(self):
        self.omnidir_mode = EOmniDirModeId.STOP
        self.send_motor_ctrl_cmd()
