#!/usr/bin/env python3
"""SHADOW_ONLY recorder/replayer. Never subscribes or publishes sensor_msgs/Image."""
import argparse
from bisect import bisect_right
import csv
from dataclasses import asdict
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import resource
import statistics
import sys
import threading
import time
import uuid
from types import SimpleNamespace

sys.dont_write_bytecode = True
# Direct invocation also works without sourcing any existing workspace.
_PACKAGE = Path(__file__).resolve().parents[1]
if (_PACKAGE / 'src').is_dir():
    sys.path.insert(0, str(_PACKAGE / 'src'))
# The audited protocol is an existing dependency, not copied into this package.
# Explicit process-local import path avoids sourcing/changing the old workspace.
_bootstrap = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
_bootstrap.add_argument('--anchor-module-root', default=os.environ.get(
    'SENSOR_TIME_BRIDGE_PYTHON_ROOT'))
_reader_root = _bootstrap.parse_known_args()[0].anchor_module_root
if _reader_root:
    sys.path.insert(0, str(Path(_reader_root).expanduser().resolve()))
from zed_time_bridge.time_mapping import Config, MappingResult, ShadowMapper
if not _reader_root:
    _protocol = importlib.import_module('sensor_time_bridge.imu_anchor')
    _reader_root = str(Path(_protocol.__file__).resolve().parents[1])

SCHEMA = 1
LATENCY_EDGES_NS = [-10_000_000,-5_000_000,0,1_000_000,5_000_000,10_000_000,50_000_000,100_000_000]
CSV_FIELDS = ['record_index', 'stream', 'raw_stamp_ns', 'frame_id', 'seq',
              'arrival_wall_ns', 'arrival_mono_ns', 'processing_mono_ns',
              'wall_minus_header_ns', 'zed_session', 'session_id', 'mapping_version',
              'mapped_stamp_ns', 'valid', 'reason', 'clock_id', 'software_version',
              'load_label', 'units', 'angular_velocity_x', 'angular_velocity_y',
              'angular_velocity_z', 'mapping_json']
CSV_FIELDS += [name for name in MappingResult.__dataclass_fields__ if name not in CSV_FIELDS]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_reader(module_root):
    """Reuse the real protocol/reader, never duplicate or write its mmap format."""
    root = Path(module_root).expanduser().resolve()
    if not (root / 'sensor_time_bridge/imu_anchor.py').is_file():
        raise ValueError('sensor_time_bridge source root missing: ' + str(root))
    sys.path.insert(0, str(root))
    module = importlib.import_module('sensor_time_bridge.imu_anchor')
    expected = root / 'sensor_time_bridge/imu_anchor.py'
    if Path(module.__file__).resolve() != expected:
        raise ValueError('unexpected sensor_time_bridge import: ' + module.__file__)
    return module


class Sidecar:
    def __init__(self, outdir, config, provenance, load_label='', extra=None):
        self.path = Path(outdir).expanduser()
        self.path.mkdir(parents=True, exist_ok=False)
        self.config = asdict(config)
        self.load_label = load_label
        self.manifest = {'schema': SCHEMA, 'mode': 'SHADOW_ONLY', 'provenance': provenance,
                         'recording_id': str(uuid.uuid4()),
                         'hardware_status': 'HARDWARE_PENDING', 'config': self.config,
                         'load_label': load_label, 'software_version': '0.1.0',
                         'created_wall_ns': time.time_ns(),
                         'clock_id': 'CLOCK_MONOTONIC',
                         'anchor_protocol_module': sys.modules['sensor_time_bridge.imu_anchor'].__file__,
                         'anchor_protocol_sha256': digest(sys.modules['sensor_time_bridge.imu_anchor'].__file__),
                         'image_time_semantics': 'SDK_IMAGE_HOST_MEMORY_REFERENCE_NOT_EXPOSURE',
                         'imu_time_semantics': 'SDK_IMU_ACQUISITION_TIMESTAMP_HOST_MAPPING_UNCERTAIN',
                         'source_sha256': {str(p): digest(p) for p in sorted(_PACKAGE.rglob('*.py'))
                                           if '__pycache__' not in p.parts}}
        self.manifest.update(extra or {})
        (self.path / 'manifest.json').write_text(json.dumps(self.manifest, indent=2) + '\n')
        self.log = (self.path / 'records.jsonl').open('x', buffering=65536)
        self.csv_file = (self.path / 'events.csv').open('x', newline='', buffering=65536)
        self.csv = csv.DictWriter(self.csv_file, CSV_FIELDS)
        self.csv.writeheader()
        self.index = 0
        self.reasons = {}
        self.valid = 0
        self.events = 0
        self.latency = {}

    def write(self, record):
        record = dict(record, record_index=self.index)
        self.index += 1
        self.log.write(json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n')
        if record['kind'] == 'event':
            result, event = record['result'], record['input']
            row = {k: event.get(k, '') for k in CSV_FIELDS}
            row.update({k: value for k, value in result.items() if k in CSV_FIELDS and k not in
                        ('stream','raw_stamp_ns','arrival_wall_ns','arrival_mono_ns','processing_mono_ns')})
            row.update(record_index=record['record_index'],
                       wall_minus_header_ns=event['arrival_wall_ns'] - event['raw_stamp_ns'],
                       clock_id='CLOCK_MONOTONIC', software_version='0.1.0',
                       load_label=self.load_label,
                       mapping_json=json.dumps(result, separators=(',', ':'), allow_nan=False))
            self.csv.writerow(row)
            self.events += 1
            self.valid += int(result['valid'])
            reason = result['reason']
            self.reasons[reason] = self.reasons.get(reason, 0) + 1
            lag = event['arrival_wall_ns']-event['raw_stamp_ns']
            stats = self.latency.setdefault(event['stream'],dict(n=0,negative_count=0,
                min_ns=lag,max_ns=lag,sum_ns=0,histogram_counts=[0]*(len(LATENCY_EDGES_NS)+1)))
            stats['n'] += 1
            stats['negative_count'] += int(lag<0)
            stats['min_ns'],stats['max_ns'] = min(stats['min_ns'],lag),max(stats['max_ns'],lag)
            stats['sum_ns'] += lag
            stats['histogram_counts'][bisect_right(LATENCY_EDGES_NS,lag)] += 1

    def close(self, extra=None):
        self.log.close()
        self.csv_file.close()
        result = dict(records=self.index, events=self.events, valid=self.valid,
                      invalid=self.events-self.valid, reasons=self.reasons,
                      mode='SHADOW_ONLY', hardware_status='HARDWARE_PENDING')
        for stats in self.latency.values():
            stats['negative_fraction']=stats['negative_count']/stats['n']
            stats['mean_ns']=stats['sum_ns']/stats['n']
        result['arrival_minus_raw_diagnostics'] = dict(load_label=self.load_label,
            histogram_edges_ns=LATENCY_EDGES_NS,streams=self.latency,
            semantics='Signed numeric wall minus SDK header; not physical transport or exposure delay')
        result.update(extra or {})
        (self.path / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        files = [p for p in self.path.iterdir() if p.is_file() and p.name != 'SHA256SUMS']
        (self.path / 'SHA256SUMS').write_text(''.join(digest(p)+'  '+p.name+'\n' for p in sorted(files)))
        return result


def process(mapper, record):
    kind, data = record['kind'], record['input']
    if kind == 'clock':
        result = mapper.observe_clock(**data)
    elif kind == 'anchor':
        data = dict(data)
        snapshot = data.pop('snapshot')
        result = mapper.observe_anchor(SimpleNamespace(**snapshot) if snapshot else None, **data)
    elif kind == 'event':
        result = mapper.map_event(stream=data['stream'], raw_stamp_ns=data['raw_stamp_ns'],
                                  arrival_wall_ns=data['arrival_wall_ns'],
                                  arrival_mono_ns=data['arrival_mono_ns'],
                                  zed_session=data['zed_session'],
                                  now_mono_ns=data.get('processing_mono_ns',data['arrival_mono_ns'])).as_dict()
    else:
        raise ValueError('unknown sidecar record kind: '+str(kind))
    return dict(record, result=result)


def session_for_header(recording_id, caller, stream, seq, states):
    """CameraInfo has multiple publication paths: its seq is not a boot counter."""
    generation, last_imu_seq = states.get(caller, (0, None))
    if stream == 'imu_raw':
        if last_imu_seq is not None and seq < last_imu_seq:
            generation += 1
        last_imu_seq = seq
    states[caller] = (generation, last_imu_seq)
    return '{}:{}:{}'.format(recording_id, caller, generation)


def process_memory_kib():
    # VmHWM describes this executable's address space; ru_maxrss can retain the
    # launching parent's pre-exec high-water mark after fork/exec on Linux.
    fields = dict(line.split(':',1) for line in Path('/proc/self/status').read_text().splitlines() if ':' in line)
    return {'proc_rss_kib':int(fields['VmRSS'].split()[0]),
            'proc_hwm_kib':int(fields['VmHWM'].split()[0])}


def verify_input(run):
    entries = (run / 'SHA256SUMS').read_text().splitlines()
    checked = set()
    for line in entries:
        expected, name = line.split('  ',1)
        if Path(name).name != name or digest(run/name) != expected:
            raise ValueError('sidecar hash mismatch/invalid path: '+name)
        checked.add(name)
    if not {'manifest.json','records.jsonl'}.issubset(checked):
        raise ValueError('sidecar manifest/records not protected by checksum')


def replay(args):
    run = Path(args.input_dir).resolve()
    verify_input(run)
    manifest = json.loads((run/'manifest.json').read_text())
    if manifest['schema'] != SCHEMA or manifest['clock_id'] != 'CLOCK_MONOTONIC':
        raise ValueError('unsupported sidecar schema or clock domain')
    config = Config(**manifest['config'])
    mapper = ShadowMapper(config)
    sink = Sidecar(args.outdir, config, 'offline_replay', manifest.get('load_label',''),
                   {'input_manifest_sha256':digest(run/'manifest.json'), 'input_run':str(run)})
    mismatches = []
    completed = False
    try:
        with (run/'records.jsonl').open() as source:
            for expected_index, line in enumerate(source):
                original = json.loads(line)
                if original['record_index'] != expected_index:
                    raise ValueError('record ordering/index corruption')
                reproduced = process(mapper, {'kind':original['kind'],'input':original['input']})
                # Full frozen result equality; includes invalids, not only successful stamps.
                if reproduced['result'] != original['result']:
                    mismatches.append(expected_index)
                sink.write(reproduced)
        completed = True
    finally:
        summary = sink.close({'replay_mismatch_count':len(mismatches),
                              'mismatch_indices':mismatches[:100],
                              'completed':completed,
                              'software_status':'SOFTWARE_VERIFIED' if completed and not mismatches else 'FAILED'})
    if mismatches:
        raise ValueError('offline replay mismatch: '+str(mismatches[:10]))
    return summary


def l1_audit(args):
    """Read historical headers only. Missing historic clockpairs/anchors stay missing."""
    base = Path(args.input_dir)
    names = {'left':'left__image_rect_color','right':'right__image_rect_color',
             'left_info':'left__camera_info','right_info':'right__camera_info','imu_raw':'imu__data_raw'}
    events, hashes, stats = [], {}, {}
    for stream, suffix in names.items():
        p = base/('zed2i__zed_node__'+suffix+'.csv')
        if not p.exists():
            continue
        hashes[str(p)] = digest(p)
        with p.open() as f:
            rows = list(csv.DictReader(f))
        diffs = []
        for row in rows:
            e = dict(stream=stream, raw_stamp_ns=int(row['stamp_ns']), frame_id=row['frame_id'],
                     seq=int(row['seq']), arrival_wall_ns=int(row['arrival_wall_ns']),
                     arrival_mono_ns=int(row['arrival_monotonic_ns']),
                     processing_mono_ns=int(row['arrival_monotonic_ns']),
                     zed_session='l1:'+base.name)
            for axis in 'xyz':
                if 'angular_velocity_'+axis in row:
                    e['angular_velocity_'+axis] = float(row['angular_velocity_'+axis])
                    e['units'] = 'rad/s'
            events.append(e)
            diffs.append(e['arrival_wall_ns']-e['raw_stamp_ns'])
        stats[stream] = dict(n=len(diffs), negative_count=sum(d<0 for d in diffs),
                             negative_fraction=sum(d<0 for d in diffs)/len(diffs) if diffs else None,
                             min_ns=min(diffs) if diffs else None,
                             max_ns=max(diffs) if diffs else None,
                             mean_ns=statistics.mean(diffs) if diffs else None)
        if diffs:
            ordered=sorted(diffs)
            # Nearest-rank quantiles preserve integer diagnostic differences.
            stats[stream].update({name:ordered[max(0,math.ceil(q*len(ordered))-1)]
                                  for name,q in [('p01_ns',.01),('p50_ns',.5),('p95_ns',.95),('p99_ns',.99)]})
    if not events:
        raise ValueError('no recognized L1 metadata CSV files')
    input_session = 'l1:' + hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
    for event in events:
        event['zed_session'] = input_session
    config = Config()
    mapper = ShadowMapper(config)
    sink = Sidecar(args.outdir, config, 'historical_L1_no_clockpairs_no_anchors',args.load_label,
                   {'input_sha256':hashes,'missing_history':['bracketed_clockpairs','livox_anchors']})
    completed = False
    try:
        for event in sorted(events,key=lambda e:e['arrival_mono_ns']):
            sink.write(process(mapper,dict(kind='event',input=event)))
        completed = True
    finally:
        result = sink.close({'latency':stats,'completed':completed,
                             'software_status':'SOFTWARE_VERIFIED' if completed else 'FAILED',
                             'note':'Pairing/diagnostic replay only; no valid device mapping is possible.'})
    if result['valid']:
        raise AssertionError('historical L1 without anchors must never produce valid device time')
    return result


def record(args):
    if not math.isfinite(args.duration) or args.duration <= 0:
        raise ValueError('duration must be finite and positive')
    if args.camera_info and not args.accept_camera_info_workload:
        raise ValueError('CameraInfo may trigger wrapper work: explicitly acknowledge with --accept-camera-info-workload')
    config = Config(**json.loads(Path(args.config).read_text())) if args.config else Config()
    protocol = load_reader(args.anchor_module_root)
    reader = protocol.LivoxImuAnchorReader(str(Path(args.anchor).expanduser()))
    boot_path = Path('/proc/sys/kernel/random/boot_id')
    mapper = ShadowMapper(config)
    boot = boot_path.read_text().strip()
    sink = Sidecar(args.outdir,config,'live_clock_only' if not(args.imu_topic or args.camera_info) else 'live_metadata',
                   args.load_label,{'boot_id':boot,'anchor_path':reader.path,
                   'reader_module':protocol.__file__,'reader_sha256':digest(protocol.__file__),
                   'imu_topic':args.imu_topic,'camera_info':args.camera_info,
                   'image_subscriptions':[],'image_publishers':[]})
    lock = threading.RLock()
    rospy = None
    subscribers = []
    errors = []
    zed_sessions = {}
    stopping = False

    def callback(message, stream):
        nonlocal stopping
        wall, mono = time.time_ns(), time.monotonic_ns()
        try:
            with lock:
                if stopping:
                    return
                # Publisher identity+seq regression can expose ROS restarts. SDK-only
                # restart with continuing stamps/seq is not externally observable.
                connection = getattr(message,'_connection_header',{}) or {}
                caller = connection.get('callerid','unknown')
                seq = int(message.header.seq)
                session = session_for_header(sink.manifest['recording_id'],caller,stream,seq,zed_sessions)
                event = dict(stream=stream,raw_stamp_ns=message.header.stamp.to_nsec(),
                             frame_id=message.header.frame_id,seq=seq,
                             arrival_wall_ns=wall,arrival_mono_ns=mono,
                             processing_mono_ns=time.monotonic_ns(),
                             zed_session=session)
                if stream=='imu_raw':
                    event.update({ 'angular_velocity_'+a:getattr(message.angular_velocity,a) for a in 'xyz'})
                    event['units']='rad/s'
                sink.write(process(mapper,dict(kind='event',input=event)))
        except Exception as exc:
            with lock:
                errors.append(type(exc).__name__+': '+str(exc))
                stopping = True

    if args.imu_topic or args.camera_info:
        import rospy
        from sensor_msgs.msg import Imu, CameraInfo
        rospy.init_node('zed_time_shadow', anonymous=True, disable_signals=True)
        if args.imu_topic:
            subscribers.append(rospy.Subscriber(args.imu_topic,Imu,callback,callback_args='imu_raw',queue_size=100))
        if args.camera_info:
            for eye in ('left','right'):
                subscribers.append(rospy.Subscriber(args.namespace+'/'+eye+'/camera_info',CameraInfo,
                                                   callback,callback_args=eye+'_info',queue_size=10))
    start, cpu_start = time.monotonic_ns(), time.process_time_ns()
    next_clock, next_anchor = start, start
    completed = False
    try:
        while time.monotonic_ns()-start < round(args.duration*1e9) and not stopping:
            if rospy is not None and rospy.is_shutdown():
                break
            now = time.monotonic_ns()
            with lock:
                if now >= next_clock:
                    # Reading boot_id each sample also makes replay semantics explicit.
                    boot = boot_path.read_text().strip()
                    before = time.monotonic_ns()
                    realtime = time.time_ns()
                    after = time.monotonic_ns()
                    data = dict(m_before_ns=before,realtime_ns=realtime,m_after_ns=after,
                                boot_id=boot,boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME))
                    sink.write(process(mapper,dict(kind='clock',input=data)))
                    next_clock = now+50_000_000
                if now >= next_anchor:
                    # Boundary stat checks are additional to the reused reader's own
                    # fstat; an identity race fails closed, never silently blends files.
                    def identity():
                        try:
                            s=os.stat(reader.path)
                            return [s.st_dev,s.st_ino,s.st_size]
                        except OSError:
                            return None
                    first = identity()
                    result = reader.read()
                    second = identity()
                    error = 'inode_race' if first!=second else result.failure_reason
                    data = dict(snapshot=asdict(result.snapshot) if result.snapshot else None,
                                now_mono_ns=time.monotonic_ns(),identity=second,read_error=error)
                    sink.write(process(mapper,dict(kind='anchor',input=data)))
                    next_anchor=now+5_000_000
            time.sleep(0.001)
        completed = not errors
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            stopping=True
        for subscriber in subscribers:
            subscriber.unregister()
        reader.close()
        elapsed=time.monotonic_ns()-start
        cpu=time.process_time_ns()-cpu_start
        summary=sink.close({'elapsed_ns':elapsed,'cpu_ns':cpu,
                           'cpu_percent_one_core':100*cpu/elapsed,
                           'max_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                           **process_memory_kib(),
                           'status':mapper.status(),'callback_errors':errors,
                           'completed':completed,
                           'software_status':'SOFTWARE_VERIFIED' if completed else 'FAILED',
                           'performance_scope':'collector only; not an image or dual-device A/B test'})
    if errors:
        raise ValueError('metadata callback failed: '+errors[0])
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    live=sub.add_parser('record')
    live.add_argument('--outdir',required=True)
    live.add_argument('--duration',type=float,default=30)
    live.add_argument('--anchor',default=str(Path.home()/'timeshare_imu'))
    live.add_argument('--anchor-module-root',default=_reader_root)
    live.add_argument('--config')
    live.add_argument('--load-label',default='clock_only')
    live.add_argument('--imu-topic',default='')
    live.add_argument('--camera-info',action='store_true')
    live.add_argument('--accept-camera-info-workload',action='store_true')
    live.add_argument('--namespace',default='/zed2i/zed_node')
    offline=sub.add_parser('replay')
    offline.add_argument('--input-dir',required=True)
    offline.add_argument('--outdir',required=True)
    old=sub.add_parser('l1-audit')
    old.add_argument('--input-dir',required=True)
    old.add_argument('--outdir',required=True)
    old.add_argument('--load-label',default='historical_L1')
    # ROS launch remappings are irrelevant to the pure clock recorder.
    args=parser.parse_args([v for v in sys.argv[1:] if ':=' not in v])
    try:
        result={'record':record,'replay':replay,'l1-audit':l1_audit}[args.command](args)
    except (OSError,ValueError,AssertionError) as exc:
        parser.exit(1,'FAILED: '+str(exc)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
