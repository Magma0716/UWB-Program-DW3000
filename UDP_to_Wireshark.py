import serial
from scapy.all import wrpcap, Ether, Raw # 這裡雖然是 UWB，我們先用 Raw 封裝
from scapy.layers.dot15d4 import Dot15d4FCS # Wireshark 識別 UWB 常用 802.15.4 解析

ser = serial.Serial('COM3', 115200)
packets = []

print("Listening for UWB packets... Press Ctrl+C to stop.")

try:
    while True:
        # 讀取長度 (2 bytes)
        len_data = ser.read(2)
        frame_len = int.from_bytes(len_data, byteorder='little')
        
        # 讀取封包內容
        payload = ser.read(frame_len)
        
        # 轉換為 Scapy 物件 (假設為 802.15.4 格式)
        pkt = Dot15d4FCS(payload)
        packets.append(pkt)
        
        print(f"Captured packet: {frame_len} bytes")
        
except KeyboardInterrupt:
    if packets:
        wrpcap("uwb_capture.pcap", packets)
        print("\nSaved to uwb_capture.pcap")