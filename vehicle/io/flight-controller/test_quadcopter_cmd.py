from dataclasses import dataclass, field
import socket
import time
from duckiematrix_engine.utils.betaflight_sitl_types import FDMPacket, PWMPacket

# Send a command to the quadcopter continuously
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    packet = PWMPacket([2000, 2000, 2000, 2000])
    print(f"Sending packet: {packet}")
    socket.sendto(packet.to_bytes(), ('127.0.0.1', 9002))
    time.sleep(0.1)