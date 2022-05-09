#!/usr/bin/python
#coding=utf-8  

from ipaddress import ip_address
from threading import Thread
from dobot_api import *

import time
import json
import copy
import crcmod
import serial

# CRC16校验，返回整型数
def crc16(veritydata):
    if not veritydata:
        return
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    return crc16(veritydata)


def test1():
    s_obj = serial.Serial("/dev/ttyUSB0", baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1)

    test_init = "01 06 01 00 00 01 49 F6"
    test_500 = "01 06 01 03 01 F4 78 21"
    test_1000 = "01 06 01 03 03 E8 78 88"
    test_000 = "01 06 01 03 00 00 78 36"

    test = bytes().fromhex(test_init)
    s_obj.write(test)
    data = s_obj.read(150*60)
    print(data)

    test = bytes().fromhex(test_500)
    s_obj.write(test)
    data = s_obj.read(150*60)
    print(data)

    test = bytes().fromhex(test_1000)
    s_obj.write(test)
    data = s_obj.read(150*60)
    print(data)

    test = bytes().fromhex(test_000)
    s_obj.write(test)
    data = s_obj.read(150*60)
    print(data)



def test2():
    def one_byte_crc(data, crc_data):
        """
        处理一个字节的crc校验环节
        :param data:待处理的数据
        :param crc_data:crc寄存器值，最初始的为0xffff
        :return:
        """
        # 把第一个8位二进制数据(通信信息帧的第一个字节)与16位的CRC寄存器的低8位相异或,把结果放于CRC寄存器。
        crc_data_tmp1 = (get_crc_low(crc_data) ^ data) + (0xff00 & crc_data)
        length = 8
        while True:
            # 把CRC寄存器的内容右移一位(朝低位)用0填补最高位,并检查右移后的移出位。
            # 如果移出位为0,重复第3步(再次右移一位);如果移出位为1,CRC寄存器与多项式A001(1010000000000001)进行异或。
            if is_right_zero(crc_data_tmp1):
                crc_data_tmp2 = (crc_data_tmp1 >> 1)
                crc_data_tmp1 = crc_data_tmp2
                length -= 1
            else:
                crc_data_tmp2 = ((crc_data_tmp1 >> 1) ^ 0xA001)
                crc_data_tmp1 = crc_data_tmp2
                length -= 1
                pass
            if length == 0:
                break
        crc_data = crc_data_tmp1
        # 返回对一个8位数据的crc校验
        return crc_data


    def get_crc_low(crc_data):
        """
        16位数据 获取低位数据
        :param crc_data:crc寄存器值
        :return:获取16位数据的低位数据
        """
        return crc_data & 0x00ff


    def is_right_zero(check_data):
        """
        测试最右方（最低位）是否为 0
        :param check_data: 待测试的数据
        :return: 0 -> True 1 -> False
        """
        if (check_data & 0x0001) == 0:
            return True
        else:
            return False


    def do_crc(data_array, crc_data):
        """
        生成一个由 一个字节的16进制数 组成的列表的crc校验结果
        ！ 不是crc校验码
        :param data_array: 一个字节的16进制数 组成的列表
        :param crc_data: 初始的crc寄存器值，为modbus初始为： 0xffff
        :return: crc校验结果
        """
        for data in data_array:
            crc_data = one_byte_crc(data, crc_data)
            pass
        return crc_data


    def get_crc_verify_code(crc_data):
        """
        生成crc校验码
        :param crc_data: 16进制数据 列表的 crc校验结果
        :return: crc校验码
        """
        return ((crc_data & 0x00ff) << 8) | ((crc_data & 0xff00) >> 8)


    crc = 0xffff
    # test_init = "0106010301F4"
    datalist = [0x01, 0x06, 0x01, 0x03, 0x00, 0x00]
    print('进行循环冗余校验结果: 0x{:x}'.format(do_crc(datalist, crc)))
    print('生成Modbus 循环冗余校验码: 0x{:x}'.format(get_crc_verify_code(do_crc(datalist, crc))))

test1()
# test2()


# class RobotProperty:
#     def __init__(self) -> None:
#         self.filled = False
#         self.speeding_scale = None
#         self.robot_mode = None
#         self.digital_input_bits = None
#         self.digital_output_bits = None
#         self.q_actual = None
#         self.tool_vector_actual = None
    

# class Robot():
#     def __init__(self, ip='192.168.1.6', dash_port=29999, move_port=30003, feedback_port=30004):
#         self.ip = ip
#         self.dash_port = dash_port
#         self.move_port = move_port
#         self.feedback_port = feedback_port
#         self.connected = False
#         self.feedback_data = False
        
#         self.client_dash = None
#         self.client_feed = None
#         self.client_move = None
        
#         self.robot_data = RobotProperty()
        
        
#     def connect(self):
#         try:
#             self.client_dash = DobotApiDashboard(self.ip, self.dash_port)
#             self.client_move = DobotApiMove(self.ip, self.move_port)
#             self.client_feed = DobotApi(self.ip, self.feedback_port)
#             self.connected = True
#             self.set_feed_back()
#             print('connect SUCCESS')
#         except:
#             print('connect ERROR')
        
        
#     def enable_robot(self):
#         self.client_dash.EnableRobot()
    
#     def disable_robot(self):
#         self.client_dash.DisableRobot()
        
#     def set_feed_back(self):
#         thread = Thread(target=self.feed_back)
#         thread.setDaemon(True)
#         thread.start()
    
#     def feed_back(self):
#         hasRead = 0
#         while True:
#             if not self.connected:
#                 continue
#             data = bytes()
#             while hasRead < 1440:
#                 temp = self.client_feed.socket_dobot.recv(1440 - hasRead)
#                 if len(temp) > 0:
#                     hasRead += len(temp)
#                     data += temp
#             hasRead = 0

#             a = np.frombuffer(data, dtype=MyType)
#             # print("robot_mode:", a["robot_mode"][0])
#             # print("test_value:", hex((a['test_value'][0])))
#             if hex((a['test_value'][0])) == '0x123456789abcdef':
#                 # print('tool_vector_actual',
#                 #       np.around(a['tool_vector_actual'], decimals=4))
#                 # print('q_actual', np.around(a['q_actual'], decimals=4))

#                 # Refresh Properties
#                 self.robot_data.speeding_scale = a["speed_scaling"][0]
#                 self.robot_data.robot_mode = a["robot_mode"][0]
#                 self.robot_data.digital_input_bits = bin(a["digital_input_bits"][0])[2:].rjust(64, '0')
#                 self.robot_data.digital_output_bits = bin(a["digital_output_bits"][0])[2:].rjust(64, '0')

#                 # Refresh coordinate points
#                 self.robot_data.q_actual = a["q_actual"]
#                 self.robot_data.tool_vector_actual = a["tool_vector_actual"]

#                 # check alarms
#                 if a["robot_mode"] == 9:
#                     self.display_error_info()
#             self.feedback_data = True
#             time.sleep(0.005)
    
    
#     def display_robot_data(self):
#         if self.feedback_data:
#             print('##################################')
#             print(self.robot_data.tool_vector_actual)
#             print(self.robot_data.q_actual)
    
#     def display_error_info(self):
#         error_list = self.client_dash.GetErrorID().split("{")[1].split("}")[0]

#         error_list = json.loads(error_list)
#         print("error_list:", error_list)
#         if error_list[0]:
#             for i in error_list[0]:
#                 self.form_error(i, self.alarm_controller_dict,
#                                 "Controller Error")

#         for m in range(1, len(error_list)):
#             if error_list[m]:
#                 for n in range(len(error_list[m])):
#                     self.form_error(n, self.alarm_servo_dict, "Servo Error")
                    
#     def movJ(self, x, y, z, rx, ry, rz):
#         self.client_move.MovJ(x, y, z,rx, ry, rz)
                    
# def main():
#     robot = Robot()
#     robot.connect()
#     robot.disable_robot()
    
    # time.sleep(0.4)
    # position = robot.robot_data.tool_vector_actual[0]
    # pos = copy.copy(position)
    # pos[0] -= 20

    # robot.movJ(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
    # position = robot.robot_data.tool_vector_actual[0]
    # # for i in range(100000):
    #     robot.display_robot_data()
    #     time.sleep(0.1)
    
# if __name__ == "__main__":
#     main()