import serial
import socket

SERIAL_PORT = "COM6"
BAUDRATE = 115200

UDP_IP = "255.255.255.255"   # Broadcast
UDP_PORT = 8001

ser = serial.Serial(SERIAL_PORT, BAUDRATE)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print(f"Forwarding {SERIAL_PORT} -> UDP {UDP_IP}:{UDP_PORT}")

while True:
    line = ser.readline()
    if line:
        sock.sendto(line, (UDP_IP, UDP_PORT))
        print(str(line)[2:-5:])