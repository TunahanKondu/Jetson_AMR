#!/usr/bin/env python3

import socket
import struct


PLC_IP = '127.0.0.1'
PLC_PORT = 1515

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

sock.bind((PLC_IP, PLC_PORT))

print(
    f'Sahte PLC dinliyor: '
    f'{PLC_IP}:{PLC_PORT}'
)

while True:
    data, robot_address = sock.recvfrom(64)

    if len(data) != 7:
        print(
            f'Hatalı paket: {len(data)} byte'
        )
        continue

    status, pickup, dropoff, x_cm, y_cm = (
        struct.unpack('<BBBhh', data)
    )

    print(
        f'Robot={robot_address} | '
        f'durum={status}, '
        f'alım={pickup}, '
        f'bırakma={dropoff}, '
        f'x={x_cm}, y={y_cm}'
    )

    # Test görevi: A2 -> B1, başla/devam et
    response = bytes([
        2,  # A2
        1,  # B1
        2   # Başla/Devam et
    ])

    sock.sendto(
        response,
        robot_address
    )
