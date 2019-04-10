#!/usr/bin/env python3.7

# import serial
import socket
import time

from hokuyo.driver import hokuyo
# from hokuyo.tools import serial_port
from hokuyo.tools import hokuyo_socket

# uart_port = '/dev/tty.lidar'
# uart_speed = 115200
lidar_host = 'phantom-zynq'
lidar_port = 12345

if __name__ == '__main__':
	# laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
	# port = serial_port.SerialPort(laser_serial)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((lidar_host, lidar_port))
	lidar_socket = hokuyo_socket.Socket(s)

	laser = hokuyo.Hokuyo(lidar_socket)
	laser.reset()
	time.sleep(0.2)

	print(laser.get_version_info())
	print('---')
	print(laser.get_sensor_specs())
	print('---')
	print(laser.get_sensor_state())
