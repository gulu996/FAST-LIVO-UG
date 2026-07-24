#!/usr/bin/env python3
import math


USABLE_TIME_STATES = (2, 3, 4)  # LOCKED, HOLDOVER, RELOCKING


def utc_to_local(utc_ns, mapping, last_local_ns):
    if (
        mapping is None
        or mapping.mapping_version == 0
        or mapping.time_state not in USABLE_TIME_STATES
        or not math.isfinite(mapping.utc_ns_per_local_ns)
        or mapping.utc_ns_per_local_ns <= 0.0
    ):
        return 0, False
    delta_ns = utc_ns - mapping.utc_reference_ns
    local_ns = int(round(
        mapping.local_reference_ns + delta_ns / mapping.utc_ns_per_local_ns
    ))
    return (local_ns, True) if local_ns > last_local_ns else (0, False)


def valid_for_fusion(local_time_valid, position_valid, quality, accepted_qualities):
    return (
        local_time_valid
        and position_valid
        and quality in accepted_qualities
    )
