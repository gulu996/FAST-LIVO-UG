#!/usr/bin/env python3
"""Independent read-only ROS ZED audit. No camera settings or timestamps changed."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time

PREFIX = '/zed2i/zed_node/'


def topics(raw=False):
    image = 'image_raw_color' if raw else 'image_rect_color'
    left, right = ('left_raw', 'right_raw') if raw else ('left', 'right')
    return [PREFIX + left + '/' + image, PREFIX + right + '/' + image,
            PREFIX + left + '/camera_info', PREFIX + right + '/camera_info',
            PREFIX + 'imu/data', PREFIX + 'imu/data_raw']


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    def percentile(p):
        index = (len(ordered) - 1) * p
        lower = int(index)
        return ordered[lower] + (ordered[min(lower + 1, len(ordered) - 1)] - ordered[lower]) * (index - lower)
    return dict(min=min(values), mean=statistics.mean(values), p50=percentile(.5),
                p95=percentile(.95), p99=percentile(.99), max=max(values),
                std=statistics.pstdev(values))


def stamp_stats(stamps, expected_hz):
    intervals = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    span = stamps[-1] - stamps[0] if stamps else 0
    return dict(count=len(stamps), first_ns=stamps[0] if stamps else None,
                last_ns=stamps[-1] if stamps else None,
                header_rate_hz=(len(stamps) - 1) * 1e9 / span if span > 0 else None,
                expected_hz=expected_hz, interval_ms=distribution(intervals),
                non_increasing=sum(dt <= 0 for dt in intervals),
                intervals_gt_1_5_nominal=sum(dt > 1500 / expected_hz for dt in intervals),
                duplicate_stamps=len(stamps) - len(set(stamps)))


def pair_stats(left, right):
    if not left or not right:
        return dict(status='MISSING_TOPIC', left_count=len(left), right_count=len(right))
    start, end = max(min(left), min(right)), min(max(left), max(right))
    a = {v for v in left if start <= v <= end}
    b = {v for v in right if start <= v <= end}
    return dict(status='PASS' if a and a == b else 'FAIL',
                common_start_ns=start, common_end_ns=end,
                left_unique_count=len(a), right_unique_count=len(b),
                exact_matches=len(a & b), left_only=sorted(a - b), right_only=sorted(b - a))


def ros_object(value):
    if hasattr(value, '__slots__'):
        return {key: ros_object(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_object(item) for item in value]
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def write_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def file_stem(topic):
    return topic.strip('/').replace('/', '__')


def proc_sample(pid):
    stat = Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, ticks=int(stat[11]) + int(stat[12]),
                rss_bytes=int(stat[21]) * os.sysconf('SC_PAGE_SIZE'),
                command=Path('/proc/{}/cmdline'.format(pid)).read_bytes().replace(b'\0', b' ').decode(errors='replace'))


def find_zed_pids():
    found = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            args = (directory / 'cmdline').read_bytes().split(b'\0')
            if args and (args[0].endswith(b'/zed_wrapper_node') or
                         (args[0].endswith(b'/nodelet') and b'__name:=zed_node' in args)):
                found.append(int(directory.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return found


def self_test():
    timestamps = [1_700_000_000_000_000_000 + i * 33_333_333 for i in range(31)]
    result = stamp_stats(timestamps, 30)
    assert abs(result['header_rate_hz'] - 30) < 1e-5
    assert result['non_increasing'] == result['intervals_gt_1_5_nominal'] == 0
    assert stamp_stats([1, 1, 0, 100_000_000], 30)['non_increasing'] == 2
    assert stamp_stats([1, 1, 0, 100_000_000], 30)['intervals_gt_1_5_nominal'] == 1
    assert pair_stats(timestamps, timestamps[2:])['status'] == 'PASS'
    assert pair_stats(timestamps, timestamps[:10] + timestamps[11:])['left_only'] == [timestamps[10]]
    assert pair_stats([], timestamps)['status'] == 'MISSING_TOPIC'
    assert pair_stats([1, 2], [3, 4])['status'] == 'FAIL'
    assert distribution([1, 3])['p50'] == 2
    assert proc_sample(os.getpid())['rss_bytes'] > 0
    print('ros_audit self-test PASS: precision, rates, discontinuities, pairing, /proc parsing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir', help="Fresh output directory (required except for --self-test)")
    parser.add_argument('--duration', type=float, default=35)
    parser.add_argument('--raw', action='store_true')
    parser.add_argument('--imu-hz', type=float, default=200)
    parser.add_argument('--pid', type=int, action='append', default=[])
    parser.add_argument('--hash-payload', action='store_true', help='Hash every serialized message; extra CPU cost, normally leave off.')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.outdir:
        parser.error("--outdir is required")
    if not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.imu_hz) or args.imu_hz <= 0:
        parser.error('duration and imu-hz must be finite and positive')
    import io
    import rospy
    from sensor_msgs.msg import Image, CameraInfo, Imu
    from diagnostic_msgs.msg import DiagnosticArray
    output = Path(args.outdir)
    output.mkdir(parents=True, exist_ok=False)
    selected = topics(args.raw)
    rows = {topic: [] for topic in selected}
    first_images, first_info, diagnostics, resources = {}, {}, [], []
    hashes = {topic: hashlib.sha256() for topic in selected}
    collecting = True
    start_wall, start_mono = time.time_ns(), time.monotonic_ns()
    stop_mono = start_mono + round(args.duration * 1e9)
    def callback(message, topic):
        wall, mono = time.time_ns(), time.monotonic_ns()
        if not collecting or mono >= stop_mono:
            return
        row = dict(stamp_ns=message.header.stamp.to_nsec(), seq=message.header.seq,
                   arrival_wall_ns=wall, arrival_monotonic_ns=mono, frame_id=message.header.frame_id)
        if isinstance(message, Image):
            row.update(width=message.width, height=message.height, encoding=message.encoding,
                       step=message.step, is_bigendian=message.is_bigendian, data_bytes=len(message.data))
            if topic not in first_images:
                first_images[topic] = message
        elif isinstance(message, CameraInfo):
            row.update(width=message.width, height=message.height)
            if topic not in first_info:
                first_info[topic] = ros_object(message)
        else:
            for field in ['orientation', 'angular_velocity', 'linear_acceleration']:
                vector = getattr(message, field)
                for axis in vector.__slots__:
                    row[field + '_' + axis] = getattr(vector, axis)
            for field in ['orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance']:
                row[field] = json.dumps(list(getattr(message, field)))
        if args.hash_payload:
            buffer = io.BytesIO()
            message.serialize(buffer)
            hashes[topic].update(buffer.getvalue())
            row['serialized_sha256'] = hashlib.sha256(buffer.getvalue()).hexdigest()
        rows[topic].append(row)
    def diagnostic_callback(message):
        if collecting and time.monotonic_ns() < stop_mono:
            diagnostics.append(dict(arrival_wall_ns=time.time_ns(), arrival_monotonic_ns=time.monotonic_ns(),
                                    message=ros_object(message)))
    rospy.init_node('zed_l1_readonly_audit', anonymous=True, disable_signals=True)
    # ponytail: six bounded subscriptions; image buffers hold several 1080p frames, not an archive.
    subscribers = [rospy.Subscriber(topic, cls, callback, callback_args=topic,
                                    queue_size=10 if cls is Image else 1000,
                                    buff_size=64 * 1024 * 1024 if cls is Image else 1024 * 1024,
                                    tcp_nodelay=True)
                   for topic, cls in zip(selected, [Image, Image, CameraInfo, CameraInfo, Imu, Imu])]
    subscribers.append(rospy.Subscriber('/diagnostics', DiagnosticArray, diagnostic_callback, queue_size=100))
    previous = {}
    try:
        while not rospy.is_shutdown() and time.monotonic_ns() < stop_mono:
            now = time.monotonic_ns()
            sample = dict(wall_ns=time.time_ns(), monotonic_ns=now, processes=[])
            for pid in args.pid or find_zed_pids():
                try:
                    item = proc_sample(pid)
                    prior = previous.get(pid)
                    item['cpu_percent_one_core_100'] = ((item['ticks'] - prior[0]) / os.sysconf('SC_CLK_TCK') /
                                                       ((now - prior[1]) / 1e9) * 100) if prior else None
                    previous[pid] = (item['ticks'], now)
                    sample['processes'].append(item)
                except (FileNotFoundError, PermissionError, ProcessLookupError) as error:
                    sample['processes'].append(dict(pid=pid, error=str(error)))
            sample['meminfo_kib'] = {line.split(':')[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                                    if line.startswith(('MemTotal:', 'MemAvailable:', 'SwapTotal:', 'SwapFree:'))}
            try:
                gpu = subprocess.run(['nvidia-smi', '--query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu,power.draw',
                                      '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=2)
                sample['gpu_csv_fields'] = 'index,name,gpu_util_percent,memory_util_percent,used_MiB,total_MiB,temp_C,power_W'
                sample['gpu_csv'] = gpu.stdout.strip()
                sample['gpu_returncode'] = gpu.returncode
                if gpu.stderr:
                    sample['gpu_stderr'] = gpu.stderr.strip()
            except (OSError, subprocess.TimeoutExpired) as error:
                sample['gpu_error'] = str(error)
            resources.append(sample)
            time.sleep(max(0, min(1 - (time.monotonic_ns() - now) / 1e9, (stop_mono - time.monotonic_ns()) / 1e9)))
    except KeyboardInterrupt:
        pass
    finally:
        collecting = False
        for subscriber in subscribers:
            subscriber.unregister()
    end_wall, end_mono = time.time_ns(), time.monotonic_ns()
    summary = dict(master_uri=os.environ.get('ROS_MASTER_URI'), start_wall_ns=start_wall, start_monotonic_ns=start_mono,
                   end_wall_ns=end_wall, end_monotonic_ns=end_mono, raw_images=args.raw, topics={},
                   note='ROS header stamps retained verbatim. Apparent arrival offsets include transport/scheduling; they are not clock accuracy or SDK dropped-frame counts.')
    for topic in selected:
        values = rows[topic]
        stamps = [row['stamp_ns'] for row in values]
        result = stamp_stats(stamps, args.imu_hz if '/imu/' in topic else 30)
        result['arrival_rate_hz'] = ((len(values) - 1) * 1e9 / (values[-1]['arrival_monotonic_ns'] - values[0]['arrival_monotonic_ns'])) if len(values) > 1 else None
        result['apparent_arrival_minus_header_ms'] = distribution([(row['arrival_wall_ns'] - row['stamp_ns']) / 1e6 for row in values])
        if args.hash_payload:
            result['serialized_stream_sha256'] = hashes[topic].hexdigest()
        summary['topics'][topic] = result
        write_csv(output / (file_stem(topic) + '.csv'), values)
    stamps = lambda topic: [row['stamp_ns'] for row in rows[topic]]
    summary['left_right_pairing'] = pair_stats(stamps(selected[0]), stamps(selected[1]))
    summary['left_image_info_pairing'] = pair_stats(stamps(selected[0]), stamps(selected[2]))
    summary['right_image_info_pairing'] = pair_stats(stamps(selected[1]), stamps(selected[3]))
    summary['imu_data_raw_pairing'] = pair_stats(stamps(selected[4]), stamps(selected[5]))
    write_json(output / 'camera_info_first.json', first_info)
    write_json(output / 'diagnostics.json', diagnostics)
    write_json(output / 'resources.json', resources)
    # Save after capture so PNG compression cannot stall subscription callbacks.
    summary['first_images'] = {}
    try:
        import cv2
        from cv_bridge import CvBridge
        bridge = CvBridge()
        for topic, message in first_images.items():
            filename = file_stem(topic) + '_first.png'
            if not cv2.imwrite(str(output / filename), bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')):
                raise RuntimeError('cv2.imwrite failed: ' + filename)
            summary['first_images'][topic] = dict(file=filename, stamp_ns=message.header.stamp.to_nsec(),
                                                  width=message.width, height=message.height, encoding=message.encoding,
                                                  pixels_sha256=hashlib.sha256(message.data).hexdigest())
    except Exception as error:
        summary['image_save_error'] = str(error)
    write_json(output / 'summary.json', summary)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
