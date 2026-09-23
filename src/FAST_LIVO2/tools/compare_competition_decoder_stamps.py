#!/usr/bin/env python3
import argparse
import json
import math
import time
import rospy
from sensor_msgs.msg import CompressedImage, Image


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def nonmonotonic_count(values):
    return sum(current <= previous for previous, current in zip(values, values[1:]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--count', type=int, default=1000)
    p.add_argument('--timeout', type=float, default=60.0)
    a = p.parse_args()
    compressed = []
    raw = []
    compressed_seq = []
    raw_seq = []
    rospy.init_node('competition_decoder_stamp_probe', anonymous=True)
    rospy.Subscriber('/tiaozhanbei/camera/left/image/compressed', CompressedImage,
                     lambda msg: (compressed.append(msg.header.stamp.to_sec()),
                                  compressed_seq.append(msg.header.seq)), queue_size=2000)
    rospy.Subscriber('/petrochemical/camera/left/image_raw', Image,
                     lambda msg: (raw.append(msg.header.stamp.to_sec()),
                                  raw_seq.append(msg.header.seq)), queue_size=2000)
    deadline = time.monotonic() + a.timeout
    rate = rospy.Rate(100)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if len(compressed) >= a.count and len(raw) >= a.count:
            break
        rate.sleep()
    paired_count = min(a.count, len(compressed), len(raw))
    delta = [raw[index] - compressed[index] for index in range(paired_count)]
    abs_delta = [abs(value) for value in delta]
    out = {
        'compressed_count': len(compressed), 'raw_count': len(raw),
        'paired_count': paired_count,
        'pairing': 'arrival_index',
        'compressed_unique_seq_count': len(set(compressed_seq[:a.count])),
        'raw_unique_seq_count': len(set(raw_seq[:a.count])),
        'delta_median_s': percentile(delta, 0.5),
        'delta_p95_s': percentile(delta, 0.95),
        'delta_max_abs_s': max(abs_delta, default=None),
        'delta_nonzero_count': sum(x != 0.0 for x in delta),
        'compressed_nonmonotonic_count': nonmonotonic_count(compressed[:a.count]),
        'raw_nonmonotonic_count': nonmonotonic_count(raw[:a.count]),
    }
    print(json.dumps(out, sort_keys=True))


if __name__ == '__main__':
    main()
