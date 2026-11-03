import importlib.util,types,unittest,sys
from pathlib import Path
p=str(Path(__file__).resolve().parents[1]/'scripts/uwb_serial_node.py')
spec=importlib.util.spec_from_file_location('uwb_node',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Contract(unittest.TestCase):
 def test_host_mapping_is_not_device_measurement(self):
  node=m.UwbSerialNode.__new__(m.UwbSerialNode);node.timestamp_mode='livox_imu_anchor';node.time_offset_ns=0;node.replay_mode='rebase'
  node.imu_time_mapper=types.SimpleNamespace(map_time=lambda host,off:types.SimpleNamespace(warning=False,success=True,mapped_stamp_ns=1762267490276699543,mapping_session_id=42,writer_epoch=1,anchor_uncertainty_ns=1000000))
  x=node._timestamp_for_line(123456789,False,0)
  self.assertEqual(x[:4],(1762267490276699543,42,1,1000000));self.assertEqual(x[4],m.RawSerialFrame.HOST_RECEIVE_LOCAL)
  self.assertNotEqual(x[4],m.RawSerialFrame.DEVICE_TIME_MAPPED_LOCAL)
 def test_preserve_is_not_certificate(self):
  node=m.UwbSerialNode.__new__(m.UwbSerialNode);node.replay_mode='preserve';node.replay_session_id=42;node.replay_uncertainty_ns=1000000
  x=node._timestamp_for_line(0,True,1762267490276699543);self.assertEqual(x[0],1762267490276699543);self.assertEqual(x[4],m.RawSerialFrame.REPLAY_PRESERVED)
if __name__=='__main__':unittest.main()
