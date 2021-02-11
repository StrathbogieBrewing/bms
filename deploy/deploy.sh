#!/usr/bin/env bash

arduino --pref build.path=../build --verify ../bms/bms.ino

rsync ../build/bms.ino.hex johny@lunar:avr

ssh johny@lunar /bin/bash <<'EOT'
# start logging from serial port and wait to clear
pkill socat
sleep 1

# program new hex file to bms board
avrdude




# start logging from serial port again
# nohup socat /dev/ttyUSB0,echo=0,b9600 UDP4-DATAGRAM:192.168.8.255:12345,broadcast </dev/null >/dev/null 2>&1 &
EOT

# monitor network with
# socat UDP4-RECVFROM:12345,broadcast,fork -

# start data logging with ./datalog -d
# start web server with busybox httpd -p 8080
