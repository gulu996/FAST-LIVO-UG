#!/usr/bin/env python3
import ast
import os
import struct
import termios
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

from uwb_serial_driver.exclusive_serial import (
    DEFAULT_DTR,
    DEFAULT_RTS,
    open_locked,
)


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, ".."))


def serial_attributes():
    return [0, 0, 0, 0, 0, 0, [0] * 32]


class ExclusiveSerialTest(unittest.TestCase):
    def open_with_ioctl(self, ioctl, dtr=DEFAULT_DTR, rts=DEFAULT_RTS):
        with mock.patch(
            "uwb_serial_driver.exclusive_serial.os.open", return_value=17
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.fcntl.flock"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.tty.setraw"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.termios.tcgetattr",
            side_effect=lambda _fd: serial_attributes(),
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.termios.tcsetattr"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.fcntl.ioctl",
            side_effect=ioctl,
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.os.close"
        ) as close:
            fd = open_locked("/dev/uwb-test", 115200, dtr, rts)
            return fd, close

    def test_defaults_set_dtr_and_clear_rts(self):
        self.assertTrue(DEFAULT_DTR)
        self.assertFalse(DEFAULT_RTS)
        written = []

        def ioctl(_fd, request, argument):
            if request == termios.TIOCMGET:
                return struct.pack("I", termios.TIOCM_RTS)
            written.append(struct.unpack("I", argument)[0])
            return 0

        fd, close = self.open_with_ioctl(ioctl)
        self.assertEqual(17, fd)
        self.assertEqual([termios.TIOCM_DTR], written)
        close.assert_not_called()

    def test_override_clears_dtr_and_sets_rts(self):
        written = []

        def ioctl(_fd, request, argument):
            if request == termios.TIOCMGET:
                return struct.pack("I", termios.TIOCM_DTR)
            written.append(struct.unpack("I", argument)[0])
            return 0

        self.open_with_ioctl(ioctl, dtr=False, rts=True)
        self.assertEqual([termios.TIOCM_RTS], written)

    def test_each_reopen_reapplies_control_lines(self):
        writes = []

        def ioctl(_fd, request, argument):
            if request == termios.TIOCMGET:
                return struct.pack("I", 0)
            writes.append(struct.unpack("I", argument)[0])
            return 0

        self.open_with_ioctl(ioctl)
        self.open_with_ioctl(ioctl)
        self.assertEqual([termios.TIOCM_DTR, termios.TIOCM_DTR], writes)

    def test_control_line_failure_closes_fd_and_names_port(self):
        error = OSError(25, "ioctl unsupported")
        with mock.patch(
            "uwb_serial_driver.exclusive_serial.os.open", return_value=23
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.fcntl.flock"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.tty.setraw"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.termios.tcgetattr",
            side_effect=lambda _fd: serial_attributes(),
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.termios.tcsetattr"
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.fcntl.ioctl",
            side_effect=error,
        ), mock.patch(
            "uwb_serial_driver.exclusive_serial.os.close"
        ) as close:
            with self.assertRaisesRegex(OSError, "/dev/uwb-test"):
                open_locked("/dev/uwb-test", 115200)
        close.assert_called_once_with(23)

    def test_read_loop_opens_once_per_connection_and_always_closes(self):
        node_path = os.path.join(PACKAGE_DIR, "scripts", "uwb_serial_node.py")
        with open(node_path, "r", encoding="utf-8") as stream:
            tree = ast.parse(stream.read())
        run_serial = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_run_serial"
        )
        calls = [
            node.func
            for node in ast.walk(run_serial)
            if isinstance(node, ast.Call)
        ]
        self.assertEqual(
            1,
            sum(
                isinstance(call, ast.Name) and call.id == "open_locked"
                for call in calls
            ),
        )
        self.assertEqual(
            1,
            sum(
                isinstance(call, ast.Attribute)
                and isinstance(call.value, ast.Name)
                and call.value.id == "os"
                and call.attr == "close"
                for call in calls
            ),
        )
        self.assertFalse(
            any(
                isinstance(call, ast.Attribute) and call.attr == "ioctl"
                for call in calls
            )
        )

    def test_launch_defaults_and_override_chain(self):
        uwb = ET.parse(os.path.join(PACKAGE_DIR, "launch", "uwb_serial.launch"))
        uwb_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in uwb.getroot().findall("arg")
        }
        self.assertEqual("true", uwb_args["dtr"])
        self.assertEqual("false", uwb_args["rts"])
        node = uwb.getroot().find("node")
        params = {
            element.attrib["name"]: element.attrib
            for element in node.findall("param")
        }
        self.assertEqual("$(arg dtr)", params["dtr"]["value"])
        self.assertEqual("bool", params["dtr"]["type"])
        self.assertEqual("$(arg rts)", params["rts"]["value"])
        self.assertEqual("bool", params["rts"]["type"])

        sensors = ET.parse(
            os.path.join(
                SRC_DIR, "sensor_recording_bringup", "launch", "sensors_only.launch"
            )
        )
        uwb_include = next(
            element
            for element in sensors.getroot().iter("include")
            if "uwb_serial.launch" in element.attrib["file"]
        )
        include_args = {
            element.attrib["name"]: element.attrib["value"]
            for element in uwb_include.findall("arg")
        }
        self.assertEqual("$(arg uwb_dtr)", include_args["dtr"])
        self.assertEqual("$(arg uwb_rts)", include_args["rts"])

        record = ET.parse(
            os.path.join(
                SRC_DIR, "sensor_recording_bringup", "launch", "record_all.launch"
            )
        )
        sensors_include = next(record.getroot().iter("include"))
        outer_args = {
            element.attrib["name"]: element.attrib["value"]
            for element in sensors_include.findall("arg")
        }
        self.assertEqual("$(arg uwb_dtr)", outer_args["uwb_dtr"])
        self.assertEqual("$(arg uwb_rts)", outer_args["uwb_rts"])


if __name__ == "__main__":
    unittest.main()
