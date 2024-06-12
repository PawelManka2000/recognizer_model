import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals

import time
import math
import serial
from threading import Lock
from recognizer.drivers.ICommunicationDrv import ICommunicationDrv
from recognizer.enums.ECmdId import ECmdId
from recognizer.enums.EDrivingModeId import EDrivingModeId

FINISH_CHAR = "f"
PAD_CHAR = "0"

class MotorDriver(Node):

    def __init__(self):

        super().__init__('motor_driver')


        # Setup parameters
        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")


        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")

        self.declare_parameter('serial_port', value="/dev/ttyACM1")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=115200)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=True)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")



        # Setup topics & services

        # self.subscription = self.create_subscription(
        #     MotorCommand,
        #     'motor_command',
        #     self.motor_command_callback,
        #     10)
        #
        # self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)
        #
        # self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        #

        # Member Variables
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()


        # Open serial comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate,timeout=1.0)
        print(f"Connected to {self.conn}")
        


    # Raw serial commands
    
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_ctrl_velo_motor_command(self, driving_mode: EDrivingModeId, velocity: float):

        formatted_velocity = "{:.2f}".format(velocity)
        velo_cmd = f"{ECmdId.CTRL_VELO_REQ.value}{driving_mode}{formatted_velocity}{FINISH_CHAR}"
        velo_cmd.ljust(8, PAD_CHAR)

        self.send_command(velo_cmd)

    def send_encoder_read_command(self):
        
        encoder_read_cmd = f"{ECmdId.STATE_REQ.value}{EDrivingModeId.UNKNOWN.value}f"
        encoder_read_cmd.ljust(8, PAD_CHAR)
        breakpoint()
        resp = self.send_command(encoder_read_cmd)
        return resp


    def parse_motors_state(self, resp):
        
        motor_states_dict = {}
        if resp:    

            resp_splited = resp.split('\n\r')
            motors_states_tab = resp_splited[:-1]
            
            for motor_state in motors_states_tab:
                motor_state_splited = motor_state.split(',')
                motor_id = EMotorId(int(motor_state_splited[0]))
                motor_states_dict[motor_id] = motor_state_splited[1:]
            print(resp)

        

    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if motor_command.is_pwm:
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            self.send_ctrl_velo_motor_command(EDrivingModeId.FORWARD.value, 3.5)


    def check_encoders(self):
        
        resp = self.send_encoder_read_command()
        parsed_motors_state = self.parse_motors_state(resp)


        if (resp):
            
            pass 
            # new_time = time.time()
            # time_diff = new_time - self.last_enc_read_time
            # self.last_enc_read_time = new_time

            # m1_diff = resp[0] - self.last_m1_enc
            # self.last_m1_enc = resp[0]
            # m2_diff = resp[1] - self.last_m2_enc
            # self.last_m2_enc = resp[1]

            # rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
            # self.m1_spd = m1_diff*rads_per_ct/time_diff
            # self.m2_spd = m2_diff*rads_per_ct/time_diff

            # spd_msg = MotorVels()
            # spd_msg.mot_1_rad_sec = self.m1_spd
            # spd_msg.mot_2_rad_sec = self.m2_spd
            # self.speed_pub.publish(spd_msg)

            # enc_msg = EncoderVals()
            # enc_msg.mot_1_enc_val = self.last_m1_enc
            # enc_msg.mot_2_enc_val = self.last_m2_enc
            # self.encoder_pub.publish(enc_msg)



    # Utility functions

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()


    def close_conn(self):
        self.conn.close()


