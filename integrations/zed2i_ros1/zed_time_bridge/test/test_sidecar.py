"""End-to-end recorded input replay and actual reused mmap reader checks."""
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'src'))
sys.dont_write_bytecode=True
from sensor_time_bridge.imu_anchor import ImuAnchorSnapshot, pack_anchor, LivoxImuAnchorReader
from zed_time_bridge.time_mapping import Config, ShadowMapper
spec=importlib.util.spec_from_file_location('zed_time_audit',PACKAGE/'scripts/zed_time_audit.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)

R0=1_792_000_000_000_000_000
D0=1_590_192_000_000_000_000
M0=1_000_000_000


def anchor(index,m):
    return ImuAnchorSnapshot(0x31414D49,1,11,2*(index+1),D0+(m-M0),m,m+100,2,True,10_000_000)


def synthetic_run(path):
    config=Config()
    mapper=ShadowMapper(config)
    sink=audit.Sidecar(path,config,'synthetic_known_clock','synthetic')
    for i in range(12):
        m=M0+i*50_000_000
        sink.write(audit.process(mapper,{'kind':'clock','input':dict(
            m_before_ns=m-10,realtime_ns=R0+m-M0,m_after_ns=m+10,
            boot_id='synthetic-boot',boottime_ns=m+1_000_000)}))
        sink.write(audit.process(mapper,{'kind':'anchor','input':dict(
            snapshot=dataclasses.asdict(anchor(i,m)),now_mono_ns=m+100,identity=[7,9,64],read_error='')}))
        if i>=3:
            for stream in ('left','right','left_info','left_info','right_info','imu_raw'):
                e=dict(stream=stream,raw_stamp_ns=R0+m-M0,
                       arrival_wall_ns=R0+m-M0+1000,arrival_mono_ns=m+1000,
                       processing_mono_ns=m+1000,frame_id='synthetic',seq=i,
                       zed_session='camera-session')
                row=audit.process(mapper,{'kind':'event','input':e})
                assert row['result']['valid'],row
                assert row['result']['mapped_stamp_ns']==D0+m-M0,row
                sink.write(row)
    return sink.close({'software_status':'SOFTWARE_VERIFIED','fixture':'synthetic_exact_unit_slopes'})


class SidecarTest(unittest.TestCase):
    def test_camera_info_seq_is_not_restart_and_recording_ids_are_unique(self):
        states={}
        first=audit.session_for_header('run-a','zed','left_info',100,states)
        self.assertEqual(first,audit.session_for_header('run-a','zed','left_info',0,states))
        self.assertEqual(first,audit.session_for_header('run-a','zed','imu_raw',100,states))
        restarted=audit.session_for_header('run-a','zed','imu_raw',0,states)
        self.assertNotEqual(first,restarted)
        self.assertEqual(restarted,audit.session_for_header('run-a','zed','right_info',1,states))
        self.assertNotEqual(first,audit.session_for_header('run-b','zed','imu_raw',100,{}))
        with tempfile.TemporaryDirectory() as tmp:
            a=audit.Sidecar(Path(tmp)/'a',Config(),'synthetic')
            b=audit.Sidecar(Path(tmp)/'b',Config(),'synthetic')
            self.assertNotEqual(a.manifest['recording_id'],b.manifest['recording_id'])
            a.close();b.close()

    def test_process_memory_is_from_proc_and_positive(self):
        memory=audit.process_memory_kib()
        self.assertGreater(memory['proc_rss_kib'],0)
        self.assertGreaterEqual(memory['proc_hwm_kib'],memory['proc_rss_kib'])

    def test_exact_replay_and_frozen_stereo_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            summary=synthetic_run(root/'source')
            replay=audit.replay(SimpleNamespace(input_dir=str(root/'source'),outdir=str(root/'replay')))
            self.assertEqual(replay['replay_mismatch_count'],0)
            self.assertEqual(summary['valid'],54)
            self.assertEqual(replay['valid'],54)
            groups={}
            for line in (root/'source/records.jsonl').read_text().splitlines():
                row=json.loads(line)
                if row['kind']=='event' and row['input']['stream']!='imu_raw':
                    key=row['input']['raw_stamp_ns']
                    if key in groups:
                        self.assertEqual(row['result'],groups[key])
                    groups[key]=row['result']

    def test_replay_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            synthetic_run(root/'source')
            with (root/'source/records.jsonl').open('a') as f:f.write('{}\n')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                audit.replay(SimpleNamespace(input_dir=str(root/'source'),outdir=str(root/'replay')))
            self.assertFalse((root/'replay').exists())

    def test_output_never_overwrites_existing_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            sentinel=Path(tmp)/'keep'
            sentinel.write_text('unchanged')
            with self.assertRaises(FileExistsError):audit.Sidecar(tmp,Config(),'synthetic')
            self.assertEqual(sentinel.read_text(),'unchanged')

    def test_reused_reader_reopens_inode_and_preserves_invalid_snapshot(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'anchor'
            reader=LivoxImuAnchorReader(str(p))
            self.assertEqual(reader.read().failure_reason,'missing')
            first=anchor(0,M0)
            p.write_bytes(pack_anchor(first))
            self.assertEqual(reader.read().snapshot,first)
            old_inode=p.stat().st_ino
            replacement=Path(tmp)/'replacement'
            second=dataclasses.replace(anchor(1,M0+5_000_000),ready=False)
            replacement.write_bytes(pack_anchor(second))
            os.replace(str(replacement),str(p))
            self.assertNotEqual(old_inode,p.stat().st_ino)
            result=reader.read()
            self.assertFalse(result.success)
            self.assertEqual(result.failure_reason,'not_ready')
            self.assertEqual(result.snapshot,second)
            reader.close()


if __name__=='__main__':unittest.main()
