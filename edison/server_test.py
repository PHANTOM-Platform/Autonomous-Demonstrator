#!/usr/bin/env python3.7

import socket
import time

server_host = 'phantom-edison'
server_port = 60000

if __name__ == '__main__':
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((server_host, server_port))
	s.listen(0)

	print('Listening for connections on port {}...'.format(server_port))
	conn, addr = s.accept()
	print('Connection from', addr)
	time.sleep(1)
	print('Closing connection...')
	conn.close()
