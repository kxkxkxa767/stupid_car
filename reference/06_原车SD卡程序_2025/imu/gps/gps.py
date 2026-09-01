#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import platform
import serial.tools.list_ports

def find_ttyUSB():
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 USB 串口设备: {}'.format(posts))

if __name__ == "__main__":
    find_ttyUSB()
    if (platform.system().find("Linux") >= 0):
        port = "/dev/ttyUSB0"
    else:
        port = "COM3"
    baudrate = 9600

    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if ser.isOpen():
            print("串口打开成功...")
        else:
            ser.open()
            print("打开串口成功...")
    except Exception as e:
        print(e)
        print("串口打开失败")
        exit(0)
    else:
        print("开始打印原始串口数据（Ctrl+C 退出）")
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    # 以16进制和ASCII两种方式打印
                    hex_str = ' '.join(['%02X' % b for b in data])
                    try:
                        ascii_str = data.decode('ascii', errors='replace')
                    except:
                        ascii_str = str(data)
                    print("[HEX]", hex_str)
                    print("[ASCII]", ascii_str)
        except KeyboardInterrupt:
            print("\n已退出")
        finally:
            ser.close()