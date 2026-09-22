# P4-B same-snapshot map causality report (2026-09-21)

## Decision

`SAME_SNAPSHOT_MAP_CAUSALITY_READY = NO`

`ROOT-F_REMAINS`

The snapshot and same-process B0 replay are operational, but the mandatory
scientific baseline gate failed. The formal diagnostic run reached the fixed
snapshot at 895.098665 s and restored every following B0 frame exactly, yet
the first fork frame was already at 157.28 m/s with zero correspondences. In
the precommitted 900--906 s focus window its speed was 203.01 m/s median and
219.71 m/s maximum, versus the earlier G0 maximum of about 5.24 m/s. This is
not a comparable failure onset or speed build-up.

Per protocol, B1, B2, stadtgarten_seq2, and construction_seq2 were therefore
not run. No map-causality conclusion can be drawn from a harness whose B0 has
already changed the failure initial condition. The retained root label is the
previous provisional classification; P4-B did not newly confirm it.

No ground truth, GNSS error, or reference trajectory was used. No production
fix, commit, or push was performed.

## Fixed protocol and diagnostic boundary

The capture boundary is after the previous LIO transaction, permitted map
insertion, sliding, and diagnostics. The next immutable LiDAR/IMU packet is
then consumed by the fork and by production. P4-B is default-off.

This first harness cannot clone intervening accepted VIO updates. Its formal
configs therefore run LIO-only and locally enable deterministic LiDAR update
ordering. The B0 reproduction gate was required to prove that this diagnostic
contract preserved the known failure. It did not, so the LIO-only causal
experiment stops here.

Snapshot serialization and each fork frame use a diagnostic runner wrapper
that pauses only the runner-owned rosbag process group. The completed
request/ack/done/resumed handshake prevents input advancement while a large
snapshot or branch computation is in progress. P4-B configs enlarge bounded
LiDAR/IMU queues so messages already accepted by ROS are not discarded during
snapshot capture; the successful gate reported zero buffer overflow. Normal
`ours.yaml` behavior is unchanged.

## Executed evidence

### Runnable gates

- Release build: PASS.
- `p4_fork_snapshot_self_test`: PASS, including deep-copy isolation, empty and
  populated maps, multi-layer octrees, valid/invalid planes, mature/recent
  planes, provenance, sliding map, disk roundtrip, three-branch initial
  parity, and recent-5 point-level refit.
- Final 60--61 s live gate:
  `2026-09-21-132110-prefix`; 10 consecutive B0 frames from 60.1584 to
  61.0709 s, maximum interval 0.10418 s, 10/10 exact next-frame restore
  checks, zero overflow, snapshot/disk logical hash equality, horizon
  complete, seven child exits audited, runner PASS.
- Formal B0-only gate:
  `2026-09-21-132624-prefix`; runner PASS, seven child exits audited, 4139-row
  trajectory outputs validated, 1969.80 s wall time, 2,112,788 KiB maximum
  algorithm RSS, and `ground_truth_used=false`.

### Formal snapshot and restore

- Snapshot boundary: 895.098665 s, frame 8812.
- Fork rows: 147, from 895.198778 to 910.009190 s.
- `MEMORY_RESTORE_NEXT_FRAME`: 147/147 PASS; maximum state, covariance,
  information-matrix, and RHS difference all exactly zero; deskew hashes and
  correspondence lists all equal.
- Snapshot parity: PASS; horizon complete: YES.
- Logical full SHA-256 and disk-reloaded logical SHA-256:
  `f181a4ba79379a928088e9a50b80daca5f48157eace8a390eae6a9b84d7b440b`.
- Snapshot file size: 32 MiB. Disk file SHA-256 equals the logical full hash
  because the binary format is the canonical stable encoding.

### Why B0 failed the reproduction gate

| Window (s) | Frames | speed median/max (m/s) | committed | correspondence median/range | position growth (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 895--900 | 48 | 170.77 / 185.81 | 0 | 0 / 0--0 | 815.67 |
| 900--906 | 59 | 203.01 / 219.71 | 0 | 0 / 0--0 | 1193.88 |
| 906--910 | 39 | 230.94 / 241.39 | 0 | 0 / 0--0 | 890.13 |

At the first fork frame the position was approximately
(-1397.48, 360.32, -961.11) m and velocity was
(-118.58, 85.80, -57.57) m/s. The 900--906 s displacement direction was
approximately (-858.88, 713.59, -422.43) m. With zero correspondences,
translation information eigenvalues and residuals were also zero. The earlier
G0 failure instead built to about 5.24 m/s and retained hundreds of
correspondences with nonzero weak geometry. Thus only the broad terminal class
(`no correspondence -> inertial flight`) matches; onset, magnitude,
correspondence scale, and spectrum do not.

## Required questions

1. **What mutable state is in the snapshot?** Nominal state and full 19x19
   covariance; propagated state; cross-frame `ImuProcess` state (last IMU by
   value, times, initialization statistics and flags, means, last angular
   velocity/specific acceleration, noises, extrinsics, IMU poses and pending
   clouds); local/long-term octrees; retained real fit supports and
   provenance; manager frame/slide/position state, visual membership,
   degeneracy/direction hysteresis, motion window and update provenance; and
   mapper map-update/guard/pause, gravity, timing and scan-index lifecycle.
2. **Why is it enough for the next LiDAR frame?** These are the audited
   cross-frame inputs to propagation, deskew, scan matching, transaction,
   map update and sliding. The 147 exact real-data next-frame checks provide
   dynamic evidence for the LIO path.
3. **How is the map deep-cloned?** Sorted voxel keys are serialized to owned
   recursive trees; child slots 0--7, points, covariances, planes, thresholds,
   maturity/update state, counters, supports and provenance are copied.
   Restore allocates fresh nodes and planes.
4. **How is pointer identity removed?** Addresses, allocators, ROS handles and
   streams are excluded. Stable key order, fixed child slots, and fixed field
   order define both hashing and disk encoding.
5. **Are B0/B1/B2 hashes identical before fork?** The executable populated-map
   three-branch test passes. Formal B1/B2 were intentionally not instantiated
   after B0 failed, so no formal three-way claim is made.
6. **Does capture/restore reproduce the next frame?** Yes for the executed LIO
   contract: 10/10 in the short live gate and 147/147 in formal B0, exactly for
   state, covariance, deskew, correspondences, information and RHS.
7. **Is disk roundtrip consistent?** Canonical disk reload and component/full
   hashes are identical in self-test, short gate, and formal B0. A formal
   disk-seeded B1 next-frame solve was not run because it is downstream of the
   failed B0 gate.
8. **Does B0 reproduce seq1 failure?** No. It was already at 157.28 m/s and
   zero correspondences on its first frame; 900--906 s speed was about forty
   times the prior G0 maximum.
9. **Does B1 frozen change the 900--906 s build-up?** Not tested; mandatory
   stop after question 8.
10. **Does B2 point-level recent exclusion change the build-up?** Not tested.
    Its implementation excludes the last five accepted map frame IDs from
    real retained support points and refits with the production plane fitter.
11. **When do correspondence/geometry first diverge?** Not tested formally;
    no B1/B2 branch was allowed after the failed B0 gate.
12. **When does state first materially diverge?** Not tested formally.
13. **Does map divergence precede state divergence?** Not tested formally.
14. **What is the seq2 normal B1/B2 cost?** Not tested; normal controls are
    downstream of B0 reproduction.
15. **What is the construction normal B1/B2 cost?** Not tested for the same
    reason.
16. **Does ROOT-E have causal support?** No new causal support. Engineering
    parity does not compensate for a changed failure initial condition.
17. **Is the previous state-map-correlation classification still the most
    reasonable one?** It remains the provisional label because this invalid
    causal experiment cannot displace it; P4-B did not prove it.
18. **Is the production map-lifecycle prototype gate reached?** No. B0
    reproduction, formal three-way divergence, both normal controls, and a
    selective map-lifecycle effect are all required and absent.

## Artifacts

- Formal run:
  `/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/raw_outputs/ours/stadtgarten_seq1/2026-09-21-132624-prefix`
- Final short gate:
  `/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/raw_outputs/ours/stadtgarten_seq1/2026-09-21-132110-prefix`
- Snapshot contract: `reports/p4b_snapshot_contract_audit_20260921.md`
- Snapshot/hash/format: `include/p4_fork_snapshot.h`,
  `src/p4_fork_snapshot.cpp`
- Fork harness: `include/p4_fork_harness.h`, `src/p4_fork_harness.cpp`
- Self-test: `src/p4_fork_snapshot_self_test.cpp`
- Fixed diagnostic configs: `config/rtk_slam_dataset/p4b_*.yaml`
- Input-pause runner: `tools/run_p4b_with_paused_capture.sh`
- Internal evaluator: `tools/evaluate_p4b_same_snapshot.py`

No commit or push was performed.
