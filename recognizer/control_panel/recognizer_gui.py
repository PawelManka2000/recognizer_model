import tkinter as tk
from tkinter import ttk

class SpeedControlApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Robot Speed Control")
        self.geometry("300x300")

        # Create a label to display the speed
        self.speed_label = ttk.Label(self, text="Speed: 0")
        self.speed_label.pack(pady=10)

        # Create a slider to control the speed
        self.speed_slider = ttk.Scale(self, from_=0, to=100, orient='horizontal', command=self.update_speed)
        self.speed_slider.pack(pady=10)

        # Create control buttons
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

    def update_speed(self, event):
        # Get the current value of the slider
        speed = int(self.speed_slider.get())

        # Update the label with the current speed
        self.speed_label.config(text=f"Speed: {speed}")

        # Here you can add the code to send the speed value to your robot's control system
        # For example:
        # robot.set_speed(speed)

    def move_forward(self):
        # Code to move the robot forward
        print("Moving forward")
        # robot.move_forward()

    def move_backward(self):
        # Code to move the robot backward
        print("Moving backward")
        # robot.move_backward()

    def move_left(self):
        # Code to move the robot left
        print("Moving left")
        # robot.move_left()

    def move_right(self):
        # Code to move the robot right
        print("Moving right")
        # robot.move_right()

    def stop(self):
        # Code to stop the robot
        print("Stopping")
        # robot.stop()

