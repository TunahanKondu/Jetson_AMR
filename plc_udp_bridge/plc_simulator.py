#!/usr/bin/env python3
"""Standalone interactive PLC UDP simulator (Python standard library only)."""

import argparse
import select
import socket
import struct
import sys


HELP = 'Commands: set <pickup 1-3> <dropoff 1-3> <control 1-2> | show | help | quit'


def main():
    """Reply to each valid robot packet using the current mission values."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind-ip', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=1515)
    parser.add_argument('--pickup', type=int, choices=(1, 2, 3), default=1)
    parser.add_argument('--dropoff', type=int, choices=(1, 2, 3), default=2)
    parser.add_argument('--control', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    task = (args.pickup, args.dropoff, args.control)
    count = 0
    stdin_open = True
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.bind_ip, args.port))
            print(f'PLC listening: {args.bind_ip}:{args.port}', flush=True)
            print(HELP, flush=True)
            while True:
                readers = [sock, sys.stdin] if stdin_open else [sock]
                ready, _, _ = select.select(readers, [], [], 0.2)
                if stdin_open and sys.stdin in ready:
                    line = sys.stdin.readline()
                    if not line:
                        stdin_open = False
                    else:
                        words = line.strip().split()
                        if words == ['quit']:
                            break
                        if words == ['help']:
                            print(HELP, flush=True)
                        elif words == ['show']:
                            print(f'Task: A{task[0]} -> B{task[1]}, control={task[2]}',
                                  flush=True)
                        elif words and words[0] == 'set':
                            try:
                                values = tuple(int(value) for value in words[1:])
                                if (len(values) != 3 or values[0] not in (1, 2, 3)
                                        or values[1] not in (1, 2, 3)
                                        or values[2] not in (1, 2)):
                                    raise ValueError('pickup/dropoff=1-3, control=1-2')
                                task = values
                                print(f'Task: A{task[0]} -> B{task[1]}, '
                                      f'control={task[2]} (next robot packet)', flush=True)
                            except ValueError as error:
                                print(f'Invalid command: {error}', flush=True)
                        elif words:
                            print(HELP, flush=True)
                if sock not in ready:
                    continue
                try:
                    data, robot_address = sock.recvfrom(65535)
                    if len(data) != 7:
                        print(f'Invalid RX: {len(data)} byte from {robot_address}',
                              flush=True)
                        continue
                    status, pickup, dropoff, x_cm, y_cm = struct.unpack('<BBBhh', data)
                    if (not 1 <= status <= 8 or pickup not in (1, 2, 3)
                            or dropoff not in (1, 2, 3)):
                        print(f'Invalid RX fields: {data.hex(" ")}', flush=True)
                        continue
                    count += 1
                    print(f'#{count} Robot={robot_address[0]}:{robot_address[1]} | '
                          f'RX 7 byte: {data.hex(" ")} | status={status}, '
                          f'pickup=A{pickup}, dropoff=B{dropoff}, '
                          f'X={x_cm / 100:.2f} m, Y={y_cm / 100:.2f} m', flush=True)
                    response = struct.pack('<BBB', *task)
                    sock.sendto(response, robot_address)
                    print(f'TX 3 byte: {response.hex(" ")}', flush=True)
                except OSError as error:
                    print(f'UDP error: {error}', flush=True)
    except KeyboardInterrupt:
        pass
    except OSError as error:
        print(f'Cannot start simulator: {error}', file=sys.stderr)
        return 1
    print('PLC simulator closed.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
