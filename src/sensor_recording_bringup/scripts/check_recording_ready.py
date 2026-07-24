#!/usr/bin/env python3
import sys

import rospy
from sensor_time_msgs.msg import TimeStatus


def main():
    rospy.init_node("check_recording_ready", anonymous=True)
    timeout = float(rospy.get_param("~timeout_s", 10.0))
    try:
        status = rospy.wait_for_message("/sensor_time/status", TimeStatus, timeout=timeout)
    except rospy.ROSException:
        print("NOT_READY: no /sensor_time/status")
        return 2
    if not status.local_ready or status.session_id == 0:
        print("NOT_READY: LOCAL_SENSOR_TIME/session invalid")
        return 3
    print(
        "READY: session={} state={} utc_mapping_valid={} (UTC is not a recording gate)".format(
            status.session_id, status.time_state, int(status.utc_mapping_valid)
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
