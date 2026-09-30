#!/usr/bin/env python3
"""Small dependency-free VPPC v1 WebSocket receiver and protocol checker."""
import argparse
import base64
import hashlib
import os
import socket
import struct
import time
from urllib.parse import urlparse

HEADER = struct.Struct('<4sHHQQIHHI')
POINT = struct.Struct('<ffff')


def exact(sock, n):
    chunks = bytearray()
    while len(chunks) < n:
        part = sock.recv(n - len(chunks))
        if not part:
            raise ConnectionError('WebSocket closed')
        chunks.extend(part)
    return bytes(chunks)


def send_control(sock, opcode, payload=b''):
    mask = os.urandom(4)
    sock.sendall(bytes((0x80 | opcode, 0x80 | len(payload))) + mask +
                 bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def connect(url):
    parsed = urlparse(url)
    if parsed.scheme != 'ws' or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError('expected ws://host:port/path')
    port = parsed.port or 80
    sock = socket.create_connection((parsed.hostname, port), timeout=5)
    sock.settimeout(10)
    key = base64.b64encode(os.urandom(16)).decode('ascii')
    path = (parsed.path or '/') + (('?' + parsed.query) if parsed.query else '')
    request = (f'GET {path} HTTP/1.1\r\nHost: {parsed.hostname}:{port}\r\n'
               f'Upgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {key}\r\n\r\n')
    sock.sendall(request.encode('ascii'))
    response = bytearray()
    while b'\r\n\r\n' not in response:
        if len(response) > 8192:
            raise ValueError('oversized handshake')
        response.extend(exact(sock, 1))
    lines = response.decode('latin1').split('\r\n')
    headers = dict(line.split(':', 1) for line in lines[1:] if ':' in line)
    accept = base64.b64encode(hashlib.sha1((key +
        '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode('ascii')).digest()).decode('ascii')
    if not lines[0].startswith('HTTP/1.1 101 ') or headers.get('Sec-WebSocket-Accept', '').strip() != accept:
        raise ValueError('WebSocket handshake failed')
    return sock


def read_binary(sock):
    max_message = 64 * 1024 * 1024 + 65535 + HEADER.size
    message = bytearray()
    started = False
    while True:
        first, second = exact(sock, 2)
        if first & 0x70:
            raise ValueError('unsupported WebSocket extension bits')
        if second & 0x80:
            raise ValueError('server frames must not be masked')
        length = second & 0x7f
        if length == 126:
            length = struct.unpack('!H', exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack('!Q', exact(sock, 8))[0]
        opcode = first & 0x0f
        if opcode >= 8:
            if not first & 0x80 or length > 125:
                raise ValueError('invalid WebSocket control frame')
            control = exact(sock, length)
            if opcode == 9:
                send_control(sock, 10, control)
            elif opcode == 8:
                raise ConnectionError('server sent close')
            elif opcode != 10:
                raise ValueError('unsupported WebSocket control frame')
            continue
        if length > max_message - len(message):
            raise ValueError('oversized WebSocket message')
        if opcode == 2 and not started:
            started = True
        elif opcode != 0 or not started:
            raise ValueError('expected binary message or continuation')
        # ponytail: keep only one bounded message; WebSocket fragments are not point-cloud frames.
        message.extend(exact(sock, length))
        if first & 0x80:
            return bytes(message)


def parse_frame(data):
    if len(data) < HEADER.size:
        raise ValueError('short VPPC header')
    magic, version, kind, sequence, timestamp, count, stride, name_len, payload_len = HEADER.unpack_from(data)
    if (magic, version, kind, stride) != (b'VPPC', 1, 1, POINT.size):
        raise ValueError('unsupported VPPC frame')
    if count > (2**32 - 1) // stride or payload_len != count * stride:
        raise ValueError('point count and payload length disagree')
    if len(data) != HEADER.size + name_len + payload_len:
        raise ValueError('VPPC message length mismatch')
    frame_id = data[HEADER.size:HEADER.size + name_len].decode('utf-8')
    points = data[HEADER.size + name_len:]
    return sequence, timestamp, frame_id, count, points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url', nargs='?', default='ws://127.0.0.1:8765/')
    args = parser.parse_args()
    with connect(args.url) as sock:
        start = time.monotonic()
        frames = total = 0
        while True:
            sequence, stamp, frame_id, count, points = parse_frame(read_binary(sock))
            frames += 1
            total += len(points)
            duration = max(time.monotonic() - start, 1e-6)
            examples = [POINT.unpack_from(points, i * POINT.size) for i in range(min(count, 3))]
            print(f'sequence={sequence} timestamp_ns={stamp} frame_id={frame_id!r} '
                  f'point_count={count} payload_bytes={len(points)} '
                  f'receive_hz={frames/duration:.2f} MB/s={total/duration/1e6:.2f} '
                  f'first_points={examples}', flush=True)


if __name__ == '__main__':
    main()
