#!/usr/bin/env python3
"""Live ROS/WebSocket check: one scan, finite filter, disconnect, reconnect, slow reader."""
import argparse
import math
import socket
import struct
import time

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visionpro_pointcloud_receiver import connect, read_binary, parse_frame, POINT

FIELDS = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
          for i, name in enumerate(('x', 'y', 'z', 'intensity'))]


def cloud(stamp, points):
    return PointCloud2(header=Header(stamp=stamp, frame_id='camera_init'),
                       height=1, width=len(points), fields=FIELDS,
                       is_bigendian=False, point_step=16, row_step=16 * len(points),
                       data=b''.join(POINT.pack(*point) for point in points), is_dense=False)


def receive(pub, stamp, points, url):
    with connect(url) as sock:
        sock.settimeout(3)
        message = cloud(stamp, points)
        for _ in range(6):
            pub.publish(message)
            time.sleep(0.05)
        sequence, timestamp, frame_id, count, payload = parse_frame(read_binary(sock))
        assert timestamp == stamp.to_nsec(), (timestamp, stamp.to_nsec())
        assert frame_id == 'camera_init'
        assert count == 2 and len(payload) == 32, (count, len(payload))
        assert POINT.unpack_from(payload, 0) == (1.0, 2.0, 3.0, 4.0)
        assert POINT.unpack_from(payload, 16) == (5.0, 6.0, 7.0, 8.0)
        return sequence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='ws://127.0.0.1:8765/')
    parser.add_argument('--topic', default='/cloud_registered_frame')
    parser.add_argument('--slow-test', action='store_true')
    args = parser.parse_args()
    rospy.init_node('visionpro_gateway_smoke', anonymous=True)
    pub = rospy.Publisher(args.topic, PointCloud2, queue_size=1)
    deadline = time.monotonic() + 10
    while pub.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert pub.get_num_connections(), 'Gateway did not subscribe'
    points = [(1., 2., 3., 4.), (math.nan, 0., 0., 0.), (5., 6., 7., 8.)]
    first = receive(pub, rospy.Time(123, 456), points, args.url)
    time.sleep(0.2)
    second = receive(pub, rospy.Time(124, 789), points, args.url)
    assert second > first, (first, second)
    print(f'PASS binary protocol, finite filter, metadata, disconnect/reconnect: sequence {first} -> {second}')
    with connect(args.url) as sock:
        sock.settimeout(3)
        large_points = [(1., 2., 3., 4.)] * 4096  # 64 KiB forces Beast fragmentation.
        message = cloud(rospy.Time(126, 0), large_points)
        for _ in range(6):
            pub.publish(message)
            time.sleep(0.05)
        _, stamp, frame_id, count, payload = parse_frame(read_binary(sock))
        assert stamp == rospy.Time(126, 0).to_nsec()
        assert frame_id == 'camera_init' and count == 4096 and len(payload) == 65536
        assert POINT.unpack_from(payload, 65520) == (1.0, 2.0, 3.0, 4.0)
    print('PASS fragmented 64 KiB WebSocket message')
    if args.slow_test:
        with connect(args.url) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            large = cloud(rospy.Time(125, 0), [(1., 2., 3., 4.)] * 100000)
            for _ in range(60):
                pub.publish(large)
                time.sleep(0.03)
            print('Slow reader held 60 large frames; check Gateway dropped_frames increased in its statistics')


if __name__ == '__main__':
    main()
