import os
import sys
import socket

address = ('127.0.0.1', 8080)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    msg = raw_input("INPUT: ")
    s.sendto(msg, address)
    revcData, (remoteHost, remotePort) = s.recvfrom(1024)
    print(revcData)
s.close()