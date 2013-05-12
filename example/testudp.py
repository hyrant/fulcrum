#!/usr/bin/env python

import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 0))

target = sys.argv[1]
sock.sendto("google.com", (target, 2048))
data, addr = sock.recvfrom(1024)
print "response: ", data
