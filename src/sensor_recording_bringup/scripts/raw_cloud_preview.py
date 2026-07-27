#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Lightweight Livox point-cloud preview publisher.

Input:
    livox_ros_driver2/CustomMsg on /livox/lidar
Output:
    sensor_msgs/PointCloud2 on /mapping/globalMap

The output is only the latest downsampled scan. It is not a registered or
accumulated global map and does not run localization, mapping, or IMU fusion.
"""

import math
import struct
from typing import List, Set, Tuple

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def grayscale_rgb_as_float(value: int) -> float:
    """Pack an 8-bit grayscale value into PCL-compatible float32 RGB."""
    level = max(0, min(255, int(value)))
    rgb_uint32 = (level << 16) | (level << 8) | level
    return struct.unpack("<f", struct.pack("<I", rgb_uint32))[0]


class RawCloudPreview:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.output_topic = rospy.get_param("~output_topic", "/mapping/globalMap")
        self.output_frame = rospy.get_param("~frame_id", "camera_init")

        self.publish_rate = max(0.1, float(rospy.get_param("~publish_rate", 2.0)))
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 2)))
        self.voxel_size = max(0.0, float(rospy.get_param("~voxel_size", 0.15)))
        self.min_range = max(0.0, float(rospy.get_param("~min_range", 0.8)))
        self.max_range = max(self.min_range, float(rospy.get_param("~max_range", 40.0)))
        self.max_points = max(100, int(rospy.get_param("~max_points", 6000)))
        self.use_reflectivity_color = bool(
            rospy.get_param("~use_reflectivity_color", True)
        )

        self.min_range_sq = self.min_range * self.min_range
        self.max_range_sq = self.max_range * self.max_range
        self.min_publish_period = rospy.Duration.from_sec(1.0 / self.publish_rate)
        self.last_publish_time = rospy.Time(0)

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.publisher = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1, latch=False
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            CustomMsg,
            self.cloud_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "raw_cloud_preview started: %s -> %s, frame=%s, rate=%.2f Hz, "
            "voxel=%.3f m, range=[%.2f, %.2f] m, max_points=%d",
            self.input_topic,
            self.output_topic,
            self.output_frame,
            self.publish_rate,
            self.voxel_size,
            self.min_range,
            self.max_range,
            self.max_points,
        )

    def cloud_callback(self, msg: CustomMsg) -> None:
        now = rospy.Time.now()
        if now - self.last_publish_time < self.min_publish_period:
            return
        self.last_publish_time = now

        output_points: List[Tuple[float, float, float, float]] = []
        occupied_voxels: Set[Tuple[int, int, int]] = set()

        for index, point in enumerate(msg.points):
            if index % self.point_stride != 0:
                continue

            x = float(point.x)
            y = float(point.y)
            z = float(point.z)

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            range_sq = x * x + y * y + z * z
            if range_sq < self.min_range_sq or range_sq > self.max_range_sq:
                continue

            if self.voxel_size > 0.0:
                voxel = (
                    math.floor(x / self.voxel_size),
                    math.floor(y / self.voxel_size),
                    math.floor(z / self.voxel_size),
                )
                if voxel in occupied_voxels:
                    continue
                occupied_voxels.add(voxel)

            if self.use_reflectivity_color:
                reflectivity = getattr(point, "reflectivity", 160)
                rgb = grayscale_rgb_as_float(reflectivity)
            else:
                rgb = grayscale_rgb_as_float(200)

            output_points.append((x, y, z, rgb))
            if len(output_points) >= self.max_points:
                break

        if not output_points:
            rospy.logwarn_throttle(
                5.0,
                "raw_cloud_preview received lidar messages but no points passed "
                "the configured filters",
            )
            return

        header = Header()
        header.stamp = msg.header.stamp
        if header.stamp == rospy.Time(0):
            header.stamp = now
        header.frame_id = self.output_frame

        cloud = point_cloud2.create_cloud(header, self.fields, output_points)
        self.publisher.publish(cloud)

        rospy.loginfo_throttle(
            5.0,
            "raw_cloud_preview publishing %d/%d points on %s",
            len(output_points),
            len(msg.points),
            self.output_topic,
        )


def main() -> None:
    rospy.init_node("raw_cloud_preview", anonymous=False)
    RawCloudPreview()
    rospy.spin()


if __name__ == "__main__":
    main()
