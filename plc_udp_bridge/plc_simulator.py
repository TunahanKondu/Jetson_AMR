#!/usr/bin/env python3
"""Standalone interactive PLC UDP simulator (Python standard library only)."""

import argparse
import select
import socket
import struct
import sys
import time


HELP = 'Commands: set <pickup 1-3> <dropoff 1-3> <control 1-2> | show | help | quit'


def main():
    """Reply to each valid robot packet using the current mission values."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind-ip', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=1515)
    parser.add_argument('--pickup', type=int, choices=(1, 2, 3), default=1)
    parser.add_argument('--dropoff', type=int, choices=(1, 2, 3), default=2)
    parser.add_argument('--control', type=int, choices=(1, 2), default=2)
    parser.add_argument(
        '--door-delay', type=float, default=5.0,
        help='Seconds between automatic door control=1 and control=2')
    parser.add_argument(
        '--no-auto-door', action='store_true',
        help='Disable automatic status=5 door handshake')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if args.door_delay < 0.0:
        parser.error('--door-delay must be zero or positive')
    task = (args.pickup, args.dropoff, args.control)
    count = 0
    stdin_open = True
    last_robot_packet = None
    timeout_reported = False
    door_wait_seen = False
    door_release_at = None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.bind_ip, args.port))
            print(f'PLC listening: {args.bind_ip}:{args.port}', flush=True)
            print(HELP, flush=True)
            while True:
                readers = [sock, sys.stdin] if stdin_open else [sock]
                ready, _, _ = select.select(readers, [], [], 0.2)
                now = time.monotonic()
                if (last_robot_packet is not None
                        and now - last_robot_packet > 1.0
                        and not timeout_reported):
                    print('WARNING: Robot packet timeout (>1.0 s)', flush=True)
                    timeout_reported = True
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
                    status, pickup, dropoff, x_scaled, y_scaled = struct.unpack(
                        '<BBBhh', data)
                    if (not 1 <= status <= 8 or pickup not in (1, 2, 3)
                            or dropoff not in (1, 2, 3)):
                        print(f'Invalid RX fields: {data.hex(" ")}', flush=True)
                        continue
                    count += 1
                    last_robot_packet = time.monotonic()
                    timeout_reported = False

                    if not args.no_auto_door:
                        if status == 5:
                            if not door_wait_seen:
                                door_wait_seen = True
                                door_release_at = time.monotonic() + args.door_delay
                                task = (task[0], task[1], 1)
                                print(
                                    'AUTO DOOR: robot status=5; control=1 '
                                    f'({args.door_delay:.1f} s sonra control=2)',
                                    flush=True)
                            elif (task[2] == 1 and door_release_at is not None
                                  and time.monotonic() >= door_release_at):
                                task = (task[0], task[1], 2)
                                door_release_at = None
                                print(
                                    'AUTO DOOR: süre doldu; control=2 (kapı açık)',
                                    flush=True)
                        elif door_wait_seen:
                            door_wait_seen = False
                            door_release_at = None
                            print('AUTO DOOR: kapı çevrimi tamamlandı.', flush=True)

                    print(f'#{count} Robot={robot_address[0]}:{robot_address[1]} | '
                          f'RX 7 byte: {data.hex(" ")} | status={status}, '
                          f'pickup=A{pickup}, dropoff=B{dropoff}, '
                          f'X={x_scaled / 100.0:.2f} m, '
                          f'Y={y_scaled / 100.0:.2f} m', flush=True)
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
