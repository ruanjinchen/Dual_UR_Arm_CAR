#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
import serial
import time


def calculate_modbus_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')


def calculate_modbus_crc_from_string(input_crc):
    # 将输入的16进制字符串转换为整数列表
    data = [int(byte, 16) for byte in input_crc.split()]
    # 计算Modbus RTU CRC校验值
    modbus_crc_bytes = calculate_modbus_crc(data)
    # 将CRC校验值以十六进制表示
    modbus_crc_hex = [format(byte, '02X') for byte in modbus_crc_bytes]
    input_string_with_crc = input_crc + ' ' + ' '.join(modbus_crc_hex)
    result = input_string_with_crc
    print("原始数据：", data)
    print("计算得到的Modbus RTU CRC校验值：", modbus_crc_hex)
    print("带CRC校验值的字符串:", input_string_with_crc)
    # print(type(input_string_with_crc))
    return result


def main(args=None):
    rclpy.init(args=args)
    node = Node("gripper_controller")
    # node.get_logger().info("")

    serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=500)

    # 输入 不带CRC校验值
    input_string_1 = "09 10 03 E8 00 03 06 09 00 00 00 FF FF"
    input_string_2 = "09 10 03 E8 00 03 06 09 00 00 FF FF FF"
    # input_string_3 = "09 10 03 E8 00 03 06 09 00 00 80 10 FF"

    while rclpy.ok():
        input_key = input("Enter command:")
        if input_key == '1':
            send = calculate_modbus_crc_from_string(input_string_1)
            send_hex = bytes.fromhex(send)
            serial_port.write(send_hex)
        elif input_key == '2':
            send = calculate_modbus_crc_from_string(input_string_2)
            send_hex = bytes.fromhex(send)
            serial_port.write(send_hex)
        elif input_key == '3':
            break
        else:
            print("Invalid input.")

    node.destroy_node()
    rclpy.shutdown()
