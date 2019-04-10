#!/bin/bash

echo "Serving /dev/tty.lidar on tcp:12345..."
stty -F /dev/tty.lidar "406:0:8bd:8a30:3:1c:7f:15:4:2:64:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0"
nc -lk 12345 > /dev/tty.lidar < /dev/tty.lidar
