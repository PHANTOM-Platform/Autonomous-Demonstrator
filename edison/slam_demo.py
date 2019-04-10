#!/usr/bin/env python3.7

import socket
import time
import math
import queue
import struct
import threading
from hokuyo.driver import hokuyo
from hokuyo.tools import hokuyo_socket
from roboclaw.motorcontrol import MotorControl

server_host = 'phantom-edison'
server_port = 60000
lidar_host = 'phantom-zynq'
lidar_port = 12345
roboclaw_tty = '/dev/ttyMFD1'

current_milli_time = lambda: int(round(time.time() * 1000))
current_micro_time = lambda: int(round(time.time() * 1000000))

dist = MotorControl.metres_to_pulses(2.0)
speed = MotorControl.metres_to_pulses(0.15)

axle = 0.197
radius = 0.505
arc_c = int(MotorControl.metres_to_pulses(radius * math.pi * 2))
arc_l = int(MotorControl.metres_to_pulses((radius + axle/2) * math.pi * 2))
arc_r = int(MotorControl.metres_to_pulses((radius - axle/2) * math.pi * 2))

ratio_l = arc_l / arc_c
ratio_r = arc_r / arc_c

speed_l = int(speed * ratio_l)
speed_r = int(speed * ratio_r)


if __name__ == '__main__':

	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.bind((server_host, server_port))
	server.listen(0)

	print('Listening for connections on port {}...'.format(server_port))
	conn, addr = server.accept()
	print('Connection from', addr)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((lidar_host, lidar_port))
	laser = hokuyo.Hokuyo(hokuyo_socket.Socket(s))

	scanning_thread = threading.Thread(target=laser.scanning_distances_loop)


	motors = MotorControl(roboclaw_tty, False)
	motors.stop_all()

	battery = motors.read_batt_voltage()
	print('*********************')
	print('Battery: {}V'.format(battery))
	print('*********************')

	sensor_data = queue.Queue()

	motors.reset_encoders()
	laser.reset()
	scanning_thread.start()
	time.sleep(1.0)
	start_time = current_micro_time()

	laser.enable_scanning(True)

	motors.drive_both_speed_distance(speed, dist)
	motors.drive_speed_distance(speed_l, speed_r, arc_l, arc_r)
	motors.drive_both_speed_distance(speed, dist)
	motors.drive_speed_distance(speed_l, speed_r, arc_l, arc_r)
	motors.stop_all_buffered()

	start_s = time.time()

	num_scans = 0

	print()
	print('Gathering scan data...')
	while not motors.buffers_empty():
		scan = []
		while len(scan) == 0:
			scan = laser.get_scan_distances()
		timestamp = current_micro_time() - start_time
		encoders = motors.read_encoders()
		sensor_data.put((timestamp, encoders, scan))
		print('.', end='', flush=True)
		num_scans += 1

	end_s = time.time()

	laser.enable_scanning(False)

	motors.wait_for_empty_buffers()
	motors.stop_all()

	print()
	total_s = end_s - start_s
	print('{} scans in {:.2f}s - {:.2f} scans/s - {:.2f}ms/scan'.format(num_scans, total_s, num_scans/total_s, total_s*1000.0/num_scans))
	print()

	# // struct containing complete sensor scan sent from robot server (Edison board)
	# #define ROBOT_LIDAR_SCAN_SIZE 682
	# typedef struct {
	# 	uint16_t status;    // status of robot
	# 	uint32_t timestamp;
	# 	int32_t q1, q2;     // odometry distance for left and right wheels respectively
	# 	int16_t d[ROBOT_LIDAR_SCAN_SIZE];
	# } sensor_data_t;
	packer = struct.Struct('H I 2i 682h')
	f = open("scan.dat", "wb")

	start_s = time.time()

	print('Sending scan data for {} scans...'.format(num_scans))
	print('|', ' '*(num_scans-2), '|', sep='')
	while not sensor_data.empty():
		data = sensor_data.get()
		flat_data = [0, data[0]] + list(data[1]) + data[2]
		packed_data = packer.pack(*flat_data)
		conn.sendall(packed_data)
		f.write(packed_data)
		print('.', end='', flush=True)

	end_s = time.time()
	print()
	total_s = end_s - start_s
	print('{} scans in {:.2f}s - {:.2f} scans/s'.format(num_scans, total_s, num_scans/total_s))
	print()

	print('Closing connection...')
	conn.close()

	f.close()
	laser.terminate()
