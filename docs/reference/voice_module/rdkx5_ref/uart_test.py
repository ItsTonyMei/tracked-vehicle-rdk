#!/usr/bin/env python3
# coding: utf-8

import time
import threading
import sys
import serial


ser = serial.Serial("/dev/ttyUSB0", 115200)
#播报词 Active broadcast content
This_red=0x60    
This_green=0x61
This_yellow=0x62
Recognize_yellow=0x63
Recognize_green=0x64
Recognize_blue=0x65
Recognize_red=0x66
init=0x67

if ser.isOpen():
    print("Speech Serial Opened! Baudrate=115200")
else:
    print("Speech Serial Open Failed!")

def void_write(void_data):
    hex_string = int(void_data)
    cmd = [0xAA, 0x55, 0xFF, hex_string,0xFB]
    ser.write(cmd)
    time.sleep(0.005)
    ser.flushInput()

def speech_read():
    count = ser.inWaiting()
    if count:
        speech_data = ser.read(count)
        hex_data = speech_data.hex()
        if hex_data.startswith('aa55'):
            # 提取 '00' 和 '03' 部分
            byte1 = hex_data[4:6]  # 提取 '00'
            byte2 = hex_data[6:8]  # 提取 '03'
            # 将十六进制转换为整数
            value1 = int(byte1, 16)
            value2 = int(byte2, 16)
            ser.flushInput()
            time.sleep(0.005)
            print(f"Read_ID: {value2}")
            #return value1,value2


void_write(init)
time.sleep(0.005)
while 1:
    speech_read()

