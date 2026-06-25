#!/usr/bin/env python3
# coding: utf-8

import time
import threading
import sys
import serial


# V0.0.2
class Speech(object):

    def __init__(self, com="/dev/myspeech"):
        # com="/dev/ttyUSB0"
        self.ser = serial.Serial(com, 115200)
        if self.ser.isOpen():
            print("Speech Serial Opened! Baudrate=115200")
        else:
            print("Speech Serial Open Failed!")

    def __del__(self):
        self.ser.close()
        print("speech serial Close!")

    # 选择播报语句
    def void_write(self, void_data):
        hex_string = int(void_data)
        cmd = [0xAA, 0x55, 0xFF, hex_string,0xFB]
        print(cmd)
        self.ser.write(cmd)
        time.sleep(0.005)
        self.ser.flushInput()


    # 读取识别的语音
    def speech_read(self):
        count = self.ser.inWaiting()
        if count:
            speech_data = self.ser.read(count)
            hex_data = speech_data.hex()
            if hex_data.startswith('aa55'):
                # 提取 '00' 和 '03' 部分
                byte1 = hex_data[4:6]  # 提取 '00'
                byte2 = hex_data[6:8]  # 提取 '03'
                # 将十六进制转换为整数
                # value1 = int(byte1, 16)
                value2 = int(byte2, 16)
                self.ser.flushInput()
                time.sleep(0.005)
                return value2
        else:
            return 999
        
