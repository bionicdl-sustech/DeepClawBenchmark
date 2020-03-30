# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: URConnector
@Author: Haokun Wang
@Date: 2020/3/16 15:35
@Description:
"""
import socket
import struct
import time


class URConnector:
    def __init__(self, ip, port):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._ip = ip
        self._port = port
        self._recv_len = {'UR5': 1108, 'UR10e': 1114}

    def start(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(10)
        self._socket.connect((self._ip, self._port))

    def send(self, message: str):
        # In Python 3.x, a bytes-like object is required
        self._socket.send(message.encode())

    def recv(self, message_len: int):
        # receive bytes
        return self._socket.recv(message_len)

    def close(self):
        self._socket.close()

    def msg_unpack(self, ur_msg, start_mark: int, size_length: int, number_of_data: int):
        unpacked_msg = []
        for i in range(number_of_data):
            start = start_mark+i*size_length
            end = start_mark+(i+1)*size_length
            unpacked_msg.append(struct.unpack('!d', ur_msg[start:end])[0])
        return unpacked_msg

    def ur_get_state(self, ur='UR5'):
        self.send('get_state()\n')
        time.sleep(0.1)
        ur_msg = self.recv(self._recv_len[ur])
        msg = {'message_size': struct.unpack('!i', ur_msg[0:4])[0],
               'time': struct.unpack('!d', ur_msg[4:12])[0],
               'q_target': self.msg_unpack(ur_msg, 12, 8, 6),
               'qd_target': self.msg_unpack(ur_msg, 60, 8, 6),
               'qdd_target': self.msg_unpack(ur_msg, 108, 8, 6),
               'i_target': self.msg_unpack(ur_msg, 156, 8, 6),
               'm_target': self.msg_unpack(ur_msg, 204, 8, 6),
               'q_actual': self.msg_unpack(ur_msg, 252, 8, 6),
               'qd_actual': self.msg_unpack(ur_msg, 300, 8, 6),
               'i_actual': self.msg_unpack(ur_msg, 348, 8, 6),
               'i_control': self.msg_unpack(ur_msg, 396, 8, 6),
               'tool_vector_actual': self.msg_unpack(ur_msg, 444, 8, 6),
               'tcp_speed_actual': self.msg_unpack(ur_msg, 492, 8, 6),
               'tcp_force': self.msg_unpack(ur_msg, 540, 8, 6),
               'tool_vector_target': self.msg_unpack(ur_msg, 588, 8, 6),
               'tcp_speed_target': self.msg_unpack(ur_msg, 636, 8, 6),
               'digital_input_bits': self.msg_unpack(ur_msg, 684, 8, 1),
               'motor_temperatures': self.msg_unpack(ur_msg, 692, 8, 6),
               'controller_timer': self.msg_unpack(ur_msg, 740, 8, 1),
               'test_value': self.msg_unpack(ur_msg, 748, 8, 1),
               'robot_mode': self.msg_unpack(ur_msg, 756, 8, 1),
               'joint_mode': self.msg_unpack(ur_msg, 764, 8, 6),
               'safety_mode': self.msg_unpack(ur_msg, 812, 8, 1),
               'none_value_0': self.msg_unpack(ur_msg, 820, 8, 6),
               'tool_acelerometer_values': self.msg_unpack(ur_msg, 868, 8, 3),
               'none_value_1': self.msg_unpack(ur_msg, 892, 8, 6),
               'speed_scaling': self.msg_unpack(ur_msg, 940, 8, 1),
               'linear_momentum_norm': self.msg_unpack(ur_msg, 948, 8, 1),
               'none_value_2': self.msg_unpack(ur_msg, 956, 8, 1),
               'none_value_3': self.msg_unpack(ur_msg, 964, 8, 1),
               'v_main': self.msg_unpack(ur_msg, 972, 8, 1),
               'v_robot': self.msg_unpack(ur_msg, 980, 8, 1),
               'i_robot': self.msg_unpack(ur_msg, 988, 8, 1),
               'v_actual': self.msg_unpack(ur_msg, 996, 8, 6),
               'digital_outputs': self.msg_unpack(ur_msg, 1044, 8, 1),
               'program_state': self.msg_unpack(ur_msg, 1052, 8, 1),
               'elbow_position': self.msg_unpack(ur_msg, 1060, 8, 3),
               'elbow_velocity': self.msg_unpack(ur_msg, 1084, 8, 3)}
        return msg
