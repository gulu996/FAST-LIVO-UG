#!/usr/bin/env python3
"""Small assert-based check for evaluate_lio_sequential_shadow.py."""

import importlib.util
import csv
import math
import tempfile
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("evaluate_lio_sequential_shadow.py")
SPEC = importlib.util.spec_from_file_location("sequential_shadow", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def sample(index, state_z, direction_z=0.0, geometry=0.0):
    return {
        "timestamp": index * 0.05,
        "relative_time_s": index * 0.05,
        "frame_id": index,
        "q_state": math.expm1(max(0.0, state_z)),
        "q_velocity": 0.0,
        "state_z": state_z,
        "velocity_z": 0.0,
        "direction": 0.5,
        "direction_z": direction_z,
        "weak_geometry_score": geometry,
        "q_state_ewma": max(0.0, state_z),
        "q_state_cusum": max(0.0, state_z - 1.0),
        "rolling_robust_z": state_z,
        "cumulative_dv_025": 0.0,
        "cumulative_dv_050": 0.0,
        "cumulative_dv_100": 0.0,
    }


thresholds = {
    "q_state_fixed": MODULE.FIXED_CHI2_19_P999,
    "state_z": 3.0,
    "ewma": 3.0,
    "cusum": 3.0,
    "rolling_robust_z": 3.0,
    "direction_z": 1.0,
    "weak_geometry_score": 1.0,
}
rows = [sample(index, 0.0) for index in range(20)]
rows += [sample(index, 5.0, 2.0, 2.0) for index in range(20, 40)]
series = MODULE.method_series(rows, thresholds, consecutive_count=3)
assert not series["B_consecutive_q_state"][20]["active"]
assert series["B_consecutive_q_state"][22]["active"]
assert series["E_state_direction"][20]["active"]
assert series["F_state_direction_geometry"][20]["active"]

machine = MODULE.run_state_machine(
    series["B_consecutive_q_state"], persistent_after_s=0.10,
    would_reject_after_s=0.20, recovery_m=2, recovery_n=3)
assert any(row["state"] == "PERSISTENT_SUSPECT" for row in machine)
assert any(row["state"] == "WOULD_REJECT" for row in machine)
assert MODULE.episode_durations(
    [{"timestamp": 0.0, "active": True},
     {"timestamp": 0.05, "active": False}],
    lambda row: row["active"]) == [0.05]

# A real M-of-N recovery tolerates N-M dirty samples; this pattern has two.
m_of_n_tail = []
for offset, active in enumerate((False, False, True, False, False,
                                 False, True, False, False, False), 40):
    row = sample(offset, 5.0 if active else 0.0)
    row.update({"active": active, "score": float(active), "threshold": 3.0})
    m_of_n_tail.append(row)
m_of_n_machine = MODULE.run_state_machine(
    series["B_consecutive_q_state"] + m_of_n_tail,
    persistent_after_s=0.10, would_reject_after_s=0.20,
    recovery_m=8, recovery_n=10)
assert m_of_n_machine[-1]["state"] == "NORMAL"
assert m_of_n_machine[-1]["transition_reason"] == "m_of_n_clean_recovery"

retrigger_tail = []
for offset, active in enumerate((False,) + (True,) * 8, 40):
    row = sample(offset, 5.0 if active else 0.0)
    row.update({"active": active, "score": float(active), "threshold": 3.0})
    retrigger_tail.append(row)
retrigger_machine = MODULE.run_state_machine(
    series["B_consecutive_q_state"] + retrigger_tail,
    persistent_after_s=0.10, would_reject_after_s=0.20,
    recovery_m=8, recovery_n=10)
assert retrigger_machine[-1]["state"] == "WOULD_REJECT"
assert any(row["transition_reason"] == "recovery_retriggered_would_reject"
           for row in retrigger_machine)

recovery_tail = []
for index in range(40, 55):
    row = sample(index, 0.0)
    row.update({"active": False, "score": 0.0, "threshold": 3.0})
    recovery_tail.append(row)
recovery_input = series["B_consecutive_q_state"] + recovery_tail
recovery = MODULE.recovery_comparison(
    recovery_input, would_after_s=0.20, recovery_m=2, recovery_n=3,
    time_recovery_s=0.15)
assert {item["strategy"] for item in recovery} == {
    "immediate", "m_of_n", "time_based", "evidence_decay"}
assert all(item["reject_episode_count"] == 1 for item in recovery)
assert 0.0 < MODULE.empirical_upper_tail(5.0, [1.0, 2.0, 3.0]) < 0.5

with tempfile.TemporaryDirectory() as directory:
    output = Path(directory) / "union.csv"
    MODULE.write_csv(output, [{"normal_only": 1}, {"failure_only": 2}])
    with output.open(newline="") as stream:
        union_rows = list(csv.DictReader(stream))
    assert union_rows[1]["failure_only"] == "2"
print("test_lio_sequential_shadow: PASS")
