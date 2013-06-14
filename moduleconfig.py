#!/usr/bin/env python

import termios, sys, base64

if len(sys.argv) > 1:
	port = open(sys.argv[1], "w")
else:
	port = open("/dev/ttyUSB0", "w")
    
ssid = raw_input("SSID [ANY]: ")
bssid = raw_input("BSSID (router MAC) [ANY]: ")
secturityMode = raw_input("Select security:\n\t0 - Open\n\t1 - WPA2\n\t2 - WPA\n\t3 - WEP64\n\t4 - WEP128\nEnter WLAN security mode [Open]: ").lower().replace(" ", "")
if secturityMode == "wpa2" or secturityMode == "1":
    passphrase = raw_input("WLAN Passphrase (32 characters maximum): ")
    if len(passphrase) > 32:
        raise Exception("Passphrase too long")
    secturityMode = "WPA2:" + passphrase
elif secturityMode == "wpa" or secturityMode == "2":
    passphrase = raw_input("WLAN Passphrase (32 characters maximum): ")
    if len(passphrase) > 32:
        raise Exception("Passphrase too long")
    secturityMode = "WPA:" + passphrase
elif secturityMode == "wep64" or secturityMode == "wep" or secturityMode == "3":
    key = raw_input("WLAN Key (5 bytes/10 hex digits): ")
    secturityMode = "WEP64:" + key
elif secturityMode == "wep128" or secturityMode == "4":
    key = raw_input("WLAN Key (13 bytes/26 hex digits): ")
    secturityMode = "WEP128:" + key
else:
    secturityMode = "OPEN"
ip = raw_input("IP Address [DHCP]: ").replace(" ", "")
if len(ip) > 0:
    netmask = raw_input("Netmask [255.255.255.0]: ").replace(" ", "")
    if len(netmask) < 7:
        netmask = "255.255.255.0"
    gateway = raw_input("Gateway [None]: ").replace(" ", "")
    dns = raw_input("DNS Server [None]: ").replace(" ", "")
    ip = "IP:" + ip + "|NETMASK:" + netmask
    if len(gateway) >= 7:
        ip = ip + "|GATEWAY:" + gateway
    if len(dns) >= 7:
        ip = ip + "|DNS:" + dns
else:
    leaseTimeout = raw_input("Lease timeout (seconds) [System Default]: ").replace(" ", "")
    if len(leaseTimeout) > 0:
        leaseTimeout = int(leaseTimeout)
        if leaseTimeout < 20:
            raise Exception("Invalid lease timeout")
        ip = "DHCP:" + str(leaseTimeout)
    else:
        ip = "DHCP"
auth = raw_input("System access authorization (username:password) [None]: ")
if len(auth) > 0:
    auth = "AUTH:" + base64.b64encode(auth.encode())
    
configureString = secturityMode + "|" + ip
if len(ssid) > 0:
    configureString = configureString + "|SSID:" + ssid
if len(bssid) > 0:
    configureString = configureString + "|BSSID:" + bssid
if len(auth) > 0:
    configureString = configureString + "|" + auth
configureString = configureString + "\n"
print configureString
	
attr = termios.tcgetattr(port)

# iflag
attr[0] = attr[0] & ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.INLCR | termios.IGNCR | termios.ICRNL)

# oflag
attr[1] = attr[1] & ~(termios.OPOST)

# cflag
attr[2] = attr[2] & ~(termios.CSIZE | termios.CSTOPB | termios.CRTSCTS | termios.PARODD)
attr[2] = attr[2] | (termios.CS7 | termios.PARENB | termios.CLOCAL)

# lflag
attr[3] = attr[3] & ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)

# speed
attr[4] = termios.B9600
attr[5] = termios.B9600

# async
attr[6][termios.VMIN] = 0
attr[6][termios.VTIME] = 0

termios.tcsetattr(port, termios.TCSANOW, attr)

termios.tcsendbreak(port, 0)
port.write(configureString)

port.close()
