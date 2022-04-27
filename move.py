
from ipaddress import ip_address
from threading import Thread
from dobot_api import *

import time
import json
import copy

class RobotProperty:
    def __init__(self) -> None:
        self.filled = False
        self.speeding_scale = None
        self.robot_mode = None
        self.digital_input_bits = None
        self.digital_output_bits = None
        self.q_actual = None
        self.tool_vector_actual = None
    

class Robot():
    def __init__(self, ip='192.168.1.6', dash_port=29999, move_port=30003, feedback_port=30004):
        self.ip = ip
        self.dash_port = dash_port
        self.move_port = move_port
        self.feedback_port = feedback_port
        self.connected = False
        self.feedback_data = False
        
        self.client_dash = None
        self.client_feed = None
        self.client_move = None
        
        self.robot_data = RobotProperty()
        
        
    def connect(self):
        try:
            self.client_dash = DobotApiDashboard(self.ip, self.dash_port)
            self.client_move = DobotApiMove(self.ip, self.move_port)
            self.client_feed = DobotApi(self.ip, self.feedback_port)
            self.connected = True
            self.set_feed_back()
            print('connect SUCCESS')
        except:
            print('connect ERROR')
        
        
    def enable_robot(self):
        self.client_dash.EnableRobot()
    
    def disable_robot(self):
        self.client_dash.DisableRobot()
        
    def set_feed_back(self):
        thread = Thread(target=self.feed_back)
        thread.setDaemon(True)
        thread.start()
    
    def feed_back(self):
        hasRead = 0
        while True:
            if not self.connected:
                continue
            data = bytes()
            while hasRead < 1440:
                temp = self.client_feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0

            a = np.frombuffer(data, dtype=MyType)
            # print("robot_mode:", a["robot_mode"][0])
            # print("test_value:", hex((a['test_value'][0])))
            if hex((a['test_value'][0])) == '0x123456789abcdef':
                # print('tool_vector_actual',
                #       np.around(a['tool_vector_actual'], decimals=4))
                # print('q_actual', np.around(a['q_actual'], decimals=4))

                # Refresh Properties
                self.robot_data.speeding_scale = a["speed_scaling"][0]
                self.robot_data.robot_mode = a["robot_mode"][0]
                self.robot_data.digital_input_bits = bin(a["digital_input_bits"][0])[2:].rjust(64, '0')
                self.robot_data.digital_output_bits = bin(a["digital_output_bits"][0])[2:].rjust(64, '0')

                # Refresh coordinate points
                self.robot_data.q_actual = a["q_actual"]
                self.robot_data.tool_vector_actual = a["tool_vector_actual"]

                # check alarms
                if a["robot_mode"] == 9:
                    self.display_error_info()
            self.feedback_data = True
            time.sleep(0.005)
    
    
    def display_robot_data(self):
        if self.feedback_data:
            print('##################################')
            print(self.robot_data.tool_vector_actual)
            print(self.robot_data.q_actual)
    
    def display_error_info(self):
        error_list = self.client_dash.GetErrorID().split("{")[1].split("}")[0]

        error_list = json.loads(error_list)
        print("error_list:", error_list)
        if error_list[0]:
            for i in error_list[0]:
                self.form_error(i, self.alarm_controller_dict,
                                "Controller Error")

        for m in range(1, len(error_list)):
            if error_list[m]:
                for n in range(len(error_list[m])):
                    self.form_error(n, self.alarm_servo_dict, "Servo Error")
                    
    def movJ(self, x, y, z, rx, ry, rz):
        self.client_move.MovJ(x, y, z,rx, ry, rz)
                    
def main():
    robot = Robot()
    robot.connect()
    robot.enable_robot()
    
    time.sleep(0.4)
    position = robot.robot_data.tool_vector_actual[0]
    pos = copy.copy(position)
    pos[0] -= 20

    robot.movJ(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    position = robot.robot_data.tool_vector_actual[0]
    # for i in range(100000):
    #     robot.display_robot_data()
    #     time.sleep(0.1)
    
if __name__ == "__main__":
    main()