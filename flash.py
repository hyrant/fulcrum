#!/usr/bin/env python

import termios, sys, os, struct, select, time, math

if len(sys.argv) < 2:
	raise Exception("Need an image to flash")
else:
	fileName = sys.argv[1]
	fileSize = os.stat(fileName).st_size
	if fileSize <= 4:
		raise Exception("Invalid file")
	inputFile = open(fileName, "rb")

if len(sys.argv) > 2:
	port = os.open(sys.argv[2], os.O_RDWR|os.O_NONBLOCK)
else:
	port = os.open("/dev/ttyUSB0", os.O_RDWR|os.O_NONBLOCK)
	
attr = termios.tcgetattr(port)

# iflag
attr[0] = attr[0] & ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.INLCR | termios.IGNCR | termios.ICRNL)

# oflag
attr[1] = attr[1] & ~(termios.OPOST)

# cflag
attr[2] = attr[2] & ~(termios.CSIZE | termios.CSTOPB | termios.CRTSCTS | termios.PARODD)
attr[2] = attr[2] | (termios.CS8 | termios.PARENB | termios.CLOCAL)

# lflag
attr[3] = attr[3] & ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)

# speed
attr[4] = termios.B57600
attr[5] = termios.B57600

# async
attr[6][termios.VMIN] = 0
attr[6][termios.VTIME] = 0

termios.tcsetattr(port, termios.TCSANOW, attr)

def readOrTimeout(n):
	result = ""
	while len(result) < n:
		readable, writable, exceptional = select.select([ port ], [ ], [ ], 1.0)
		if len(readable) == 0:
			raise Exception("Timeout reading bytes")
		result = result + os.read(port, n-len(result))
	return result

def waitForACK():
	response = readOrTimeout(1)
	if struct.unpack(">B", response)[0] != 0x79:
		raise Exception("No ACK from device")

def generateXOR(s):
	x = 0
	for offset in range(0,len(s)):
		x = x ^ struct.unpack_from(">B", s, offset)[0]
	return x
	
def commandAndACK(cmd):
	os.write(port, struct.pack(">BB", cmd, cmd ^ 0xFF))
	waitForACK()

def writeMemory(address, data):
	assert address % 4 == 0
	assert len(data) <= 256 and len(data) > 0
	assert len(data) % 4 == 0
	commandAndACK(0x31)
	packed = struct.pack(">I", address)
	packed = packed + struct.pack(">B", generateXOR(packed))
	os.write(port, packed)
	waitForACK()
	packed = struct.pack(">B", len(data)-1)
	packed = packed + data
	packed = packed + struct.pack(">B", generateXOR(packed))
	os.write(port, packed)
	waitForACK()
	
def readMemory(address, length):
	assert length > 0 and length <= 256
	commandAndACK(0x11)
	packed = struct.pack(">I", address)
	packed = packed + struct.pack(">B", generateXOR(packed));
	os.write(port, packed)
	waitForACK()
	commandAndACK(length-1)
	data = readOrTimeout(length)
	return data


# Initialization sequence
os.write(port, struct.pack(">B", 0x7F))
waitForACK()

# Read write protection register
packed = readMemory(0x1FFFF808, 8)
if struct.unpack_from("<B", packed, 0)[0] != 0xFF or \
		struct.unpack_from("<B", packed, 2)[0] != 0xFF or \
		struct.unpack_from("<B", packed, 4)[0] != 0xFF or \
		struct.unpack_from("<B", packed, 6)[0] != 0xFF:
	# Remove write protection
	commandAndACK(0x73)
	waitForACK()
	time.sleep(0.1)
	os.write(port, struct.pack(">B", 0x7F))
	waitForACK()
	
# Erase all pages
commandAndACK(0x43)
commandAndACK(0xFF)

# Write and verify
allFlash = inputFile.read()
fileSize = len(allFlash)
for offset in range(0, fileSize, 256):
	length = min(256, fileSize-offset)
	toWrite = allFlash[offset:offset+length]
	if length % 4 != 0:
		toWrite = toWrite + '\x00' * (4-length%4)
	writeMemory(0x08000000 + offset, toWrite)
	checkData = readMemory(0x08000000 + offset, len(toWrite))
	if checkData != toWrite:
		raise Exception("Error verifying data")
		
# Enable write protection
commandAndACK(0x63)
nSectors = int(math.ceil(fileSize / 4096.0))
packed = struct.pack(">B", nSectors-1)
for sector in range(0, nSectors):
	packed = packed + struct.pack(">B", sector)
packed = packed + struct.pack(">B", generateXOR(packed));
os.write(port, packed)
waitForACK()

os.close(port)
