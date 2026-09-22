# P4-B snapshot contract audit (2026-09-21)

## Scope and fixed protocol

This audit precedes implementation of the P4-B same-snapshot harness. P4-B is
diagnostic-only and default-off. It must not change the production IEKF,
measurement noise, matching thresholds, iteration count, map size, map age,
P2/P3, VIO, GNSS, or UWB behavior.

The precommitted main protocol is `stadtgarten_seq1`: capture at the first
completed LiDAR transaction at or after relative time 895 s, replay through
910 s, and inspect 900--906 s. The precommitted normal-control windows are
300--315 s for `stadtgarten_seq2` and 300--315 s for `construction_seq2`.
Changing a window after observing a result invalidates the causal comparison.

## Transaction boundary

```text
previous LiDAR frame
  IMU propagation and deskew
  -> scan-to-map estimate committed/rejected atomically
  -> accepted map insertion completed
  -> local-map sliding completed
  -> frame diagnostics completed
  -> P4-B capture
next synchronized raw measurement packet
```

Capture inside IMU propagation, an IEKF iteration, insertion, or sliding is
invalid. Only an LIO/LO boundary is eligible. Any accepted VIO state update
between capture and the next branch LIO frame must be represented by the
branch or must invalidate baseline parity; it cannot be silently ignored.

## Mutable-state contract

| Owner | State that crosses a LiDAR-frame boundary | Reason it is required |
|---|---|---|
| `LIVMapper` | committed `_state`, `state_propagat`, `lidar_map_inited`, map-update counter, map-guard latches/counters, external pause count, gravity-alignment state | Selects the next update seed and the next map lifecycle transition. |
| `StatesGroup` | rotation, position, velocity, gyro/accel bias, gravity, inverse exposure, full 19x19 covariance | The next propagation and update consume the whole state, not pose alone. |
| `ImuProcess` | last IMU sample by value, last propagation/scan times, init flags/counter, means, last angular velocity and specific acceleration, noise vectors, extrinsics, accumulated IMU pose history and pending cloud | Determines tail interpolation, propagation, covariance, and deskew of the next scan. |
| synchronized packet boundary | scan begin/end, last LIO update time, scan index and immutable point/IMU contents | Proves all branches consume the same bytes at the same frame boundary. Transient image ownership is not a logical snapshot pointer. |
| `VoxelMapManager` | local and long-term map membership, visual-observed voxel set, state/position/slide center, frame id, degeneracy and direction hysteresis, rolling correction window, root-update provenance | Determines correspondence queries, sliding, guard state, and age accounting. Output streams and ROS publishers are explicitly excluded. |
| `VoxelOctoTree` | voxel key, hierarchy, plane, retained points and covariances, layer/maturity/update flags, thresholds, node center/size, child-slot identity, new-point counter | Reconstructs the logical tree without pointer identity. |
| `VoxelPlane` | center, basis/normal, covariance, plane covariance, eigenvalues, radius/distance, validity/init/update flags, id, point count, provenance and update diagnostics | These fields directly affect point-to-plane association and uncertainty. |

No KD-tree or deferred deletion queue exists in this implementation. Sliding
acts synchronously by moving or deleting voxel roots; its state is the two map
memberships plus `position_last_`, `last_slide_position`, and the visual set.

## Critical audit findings

1. `IMU_Processing` contains hidden cross-frame state beyond `StatesGroup`:
   `last_imu`, `last_prop_end_time`, `time_last_scan`, `angvel_last`,
   `acc_s_last`, initialization statistics, `IMUpose`, and `pcl_wait_proc`.
   Copying only the state and covariance cannot reproduce the next deskew.
2. Mature octree planes deliberately clear `temp_points_`. Existing P4-A
   provenance vectors retain frame/time/origin labels but not the real
   `pointWithVar` values used by the fit. Therefore a valid B2 point-level
   exclusion cannot be reconstructed from the production plane alone.
3. P4-B must retain the actual last-fit support vector from map construction
   onward, but only while its default-off diagnostic flag is enabled. B2 must
   filter that vector before fitting and recompute center, normal, covariance,
   eigen spectrum, plane covariance, and validity. Dropping a whole mixed-age
   plane is not B2.
4. `voxel_plane_id` is process-global. A sequential same-process fork must
   snapshot and isolate this allocator or exclude it only with an explicit
   proof that identity cannot influence matching. Letting branch order consume
   different ids would create an avoidable logical-hash divergence.
5. Existing unordered maps cannot be hashed or serialized in iteration order.
   Stable encoding must sort voxel keys, recurse children in slots 0--7, encode
   scalar/matrix entries in a fixed order, and never encode addresses.

## Required validation gates

Before formal replay the executable self-test must cover empty, populated,
multi-layer, valid/invalid-plane, mature/recent, provenance-bearing, and
sliding-map snapshots. A mutation of branch A's state, covariance, retained
point, plane statistic, or provenance must leave B/C hashes unchanged.

Formal replay is gated in this order: snapshot parity; in-memory next-frame
equivalence; disk next-frame equivalence; forked B0 failure reproduction;
then B0/B1/B2 causality and the two normal controls. A failure at any earlier
gate prevents a map-causality claim.

## Execution addendum

After the data disk was mounted, a 60--61 s live gate verified ten consecutive
capture/restore next frames with exact state, covariance, deskew,
correspondence, information and RHS equality. The formal seq1 B0 run captured
at 895.098665 s and repeated that equality for all 147 frames through
910.009190 s; logical snapshot and disk hashes were identical.

The formal B0 reproduction gate nevertheless failed: the first fork frame was
already at 157.28 m/s with no correspondences, and the 900--906 s median speed
was 203.01 m/s instead of a build-up comparable to the earlier approximately
5.24 m/s G0 maximum. Therefore the audit ordering was enforced: B1/B2 and the
two normal controls were not run, and the snapshot contract is considered
engineering-ready for the executed LIO path but not causality-ready for this
full failure.
