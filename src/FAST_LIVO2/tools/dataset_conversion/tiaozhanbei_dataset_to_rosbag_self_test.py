#!/usr/bin/env python3
"""Small assert-based regression check for the streaming bag converter."""

import json
import tempfile
from pathlib import Path

import rosbag

import tiaozhanbei_dataset_to_rosbag as converter


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main():
    with tempfile.TemporaryDirectory(prefix="tiaozhanbei_converter_test_") as folder:
        root = Path(folder)
        write(
            root / "imu" / "imu.csv",
            "timestamp_ns,angular_velocity_x_rad_s,angular_velocity_y_rad_s,angular_velocity_z_rad_s,"
            "linear_acceleration_x_m_s2,linear_acceleration_y_m_s2,linear_acceleration_z_m_s2\n"
            "1000000000,0.1,0.2,0.3,0,0,9.8\n"
            "1100000000,0.2,0.3,0.4,0.1,0,9.7\n",
        )
        write(
            root / "lidar" / "points.csv",
            "frame_timestamp_ns,frame_index,point_index,x_m,y_m,z_m,intensity,ring,point_time_s\n"
            "1000000000,0,0,1,2,3,4,1,0.0\n"
            "1000000000,0,1,2,3,4,5,16,0.1\n"
            "1200000000,1,0,3,4,5,6,1,0.0\n"
            "1200000000,1,1,4,5,6,7,16,0.1\n",
        )
        camera = root / "camera" / "left"
        camera.mkdir(parents=True)
        (camera / "1050000000.jpg").write_bytes(b"\xff\xd8test\xff\xd9")
        (camera / "1150000000.jpg").write_bytes(b"\xff\xd8test\xff\xd9")
        write(
            root / "gnss.csv",
            "timestamp_ns,east_m,north_m,up_m,cov_ee,cov_en,cov_eu,cov_nn,cov_nu,cov_uu,quality,num_satellites\n"
            "1075000000,1,2,3,1,0,0,1,0,2,2,8\n"
            "1175000000,2,3,4,1,0,0,1,0,2,1,10\n",
        )
        output = root / "test.bag"
        summary = root / "summary.json"
        args = converter.parse_args(
            [
                "--dataset", str(root),
                "--output", str(output),
                "--gnss-solution", str(root / "gnss.csv"),
                "--compression", "none",
                "--expected-events", "8",
                "--progress-label", "SELF TEST",
                "--summary-json", str(summary),
            ]
        )
        converter.convert(args)
        document = json.loads(summary.read_text(encoding="utf-8"))
        assert document["written"] == {"lidar": 2, "imu": 2, "camera": 2, "gnss": 2}
        with rosbag.Bag(str(output), "r") as bag:
            info = bag.get_type_and_topic_info().topics
            assert {topic: item.msg_type for topic, item in info.items()} == converter.OUTPUT_TYPES
            stamps = []
            for topic, msg, event_time in bag.read_messages():
                assert event_time == msg.header.stamp
                stamps.append((event_time.secs, event_time.nsecs))
                if topic == converter.TOPICS["lidar"]:
                    assert [field.name for field in msg.fields] == ["x", "y", "z", "intensity", "time", "ring"]
                    assert msg.point_step == 26
            assert stamps == sorted(stamps)
    print("tiaozhanbei_dataset_to_rosbag_self_test: PASS")


if __name__ == "__main__":
    main()
