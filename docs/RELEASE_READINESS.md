# Release and application readiness

Documentation baseline: **2026-10-28**. The catkin workspace root is the
repository; `src/FAST_LIVO2` is the package.

- Public `origin/main` before this cleanup: `e991e3838f11ed055cebe0fd00a03c109d9a076c`
  (`docs: document public tunnel reproduction and release status`).
- Current public algorithm commit: `9fe7e510889a12b3d2f29930c7710c5fe5e264d4`
  (`fix(lio): use floor voxel keys and metre neighbourhood units`), whose tree
  is identical to the pre-rewrite commit `5be5538`.
- The 2026-10-28 cleanup is prepared on the isolated branch
  `release/oss-cleanup-20261028`; it is not pushed by this task. Its commit is
  reported in the handoff rather than embedded here, because a commit cannot
  contain its own SHA.
- The maintainer's uncommitted `config/mid360.yaml` (`img_en: 1`), the local
  Hikrobot driver changes, the local ZED worktree and the GNSS/UWB backend files
  were preserved. The public candidate hides two hardware trees from Git
  tracking; it does not delete the maintainer's working copies.

## Ready now / still needed

| Item | State |
|---|---|
| English architecture, build and run documentation | Updated for the public tunnel entry and the cleaned public tree; statically checked and followed by an isolated core build |
| Public tunnel dataset | Share link and extraction code checked 2026-10-28; file name/size recorded; local SHA-256 recorded |
| A5/A1 voxel-key and neighbour-unit fix | In public algorithm commit `9fe7e510` (tree-identical to the tested `5be5538`); native test, tunnel replay and internal competition evidence exist |
| Public tunnel result | 3542 LIO commits, 0.542 m first/last, 2.02 m/s maximum adjacent-scan speed; LIVO with vision enabled; revisit diagnostic, not ATE |
| Restricted 804 s competition / DROP2 / real-map geometry | Existing summaries; private data and harness still restrict exact replay; **not the tunnel dataset** |
| GNSS, UWB and joint graph | Separate real 90 s software tests and a synthetic joint graph; no real simultaneous field-accuracy certification |
| Public tree contents | Core frontend/backend plus required messages, serial drivers and time bridge; optional MVS/STM32 trees and the FAST-Calib gitlink removed from public tracking; Livox driver retained under MIT |
| License metadata | `src/FAST_LIVO2/package.xml` corrected from `BSD` to `GPLv2`; GPLv3 and BSD-3-Clause license texts added where missing |
| License clearance for all distributed components | **Not complete**: GPLv2-only/GPLv3 combined-work question remains; upstream MVS wrapper `TODO` and vendor firmware terms remain unresolved upstream |
| Core build without optional hardware | Checked in the isolated candidate (see below); full hardware workspace not rebuilt |
| Fresh clone on a clean authorized host / CI | Still needed; the candidate build ran on the maintainer host with dependencies already installed |
| Remote publication / application | Cleanup not pushed; the OpenAI Codex for OSS form was not submitted by this task |

## Public reproduction status

The public tunnel share is the recommended reproduction entry. The link returned
HTTP 200, extraction code `mbpr` was accepted, and the share listing reported
`suidao2_resized.bag` at 16,334,415,367 bytes. The maintainer's exact-size local
copy has SHA-256
`9289318857b0ca009dab4570ac6f51915de1681fe60b2abc0b481c20d6711169`.
The file was not re-downloaded in this documentation pass, so this is link and
metadata verification, not an independent download/replay.

The 0.542 m result was run on the maintainer-local commit `5be5538`, which the
2026-10-28 history cleanup rewrote to `9fe7e510` with an identical tree. The
public branch now contains that algorithm tree. The result is therefore
maintainer-verified on the public code tree, but no third-party reproduction has
been reported. The run used `common/img_en: 1`, so it exercised the LIVO path,
not a pure LiDAR/IMU path.

## Public tree and build scope

The cleanup candidate tracks:

- `src/FAST_LIVO2` (frontend, GNSS adapter, fixed-lag backend, tests);
- `src/rpg_vikit` (linked visual toolkit);
- `src/gnss_comm` (message package required by `fast_livo`);
- `src/gnss_serial_driver`, `src/uwb_serial_driver`, `src/sensor_time_bridge`,
  `src/sensor_time_msgs`, `src/sensor_recording_bringup` (self-developed BSD-3
  packages);
- `src/livox_ros_driver2` (optional MIT driver; needs a separately installed
  Livox-SDK2 for live hardware).

It no longer tracks `src/mvs_ros_driver`, `src/stm32_timersync-open` or the bare
`src/FAST-Calib` gitlink. Those remain local and are documented as external
components in the [third-party register](../THIRD_PARTY_LICENSES.md). The core
build whitelist is:

```bash
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='sensor_time_msgs;sensor_time_bridge;gnss_comm;gnss_serial_driver;uwb_serial_driver;vikit_common;vikit_ros;sensor_recording_bringup;fast_livo'
```

The isolated candidate build of that whitelist completed successfully (exit 0)
without the MVS SDK, Livox-SDK2, the STM32 project or FAST-Calib. The following
native self-tests passed from that build: `lio_plane_map_self_test`,
`lio_update_transaction_self_test`, `lio_motion_consistency_self_test`,
`vio_update_transaction_self_test`, `preprocess_xyzirt_self_test`,
`livo_scan_transaction_self_test`, `gnss_adapter_self_test`,
`uwb_manager_self_test` and `rtk_fixed_lag_backend_self_test`. With the
externalized directories temporarily hidden, `roslaunch --files` resolved
`fast_livo mapping_mid360.launch`, `fast_livo live_mid360.launch` (camera
disabled by default), and
`sensor_recording_bringup record_all.launch enable_camera:=false`. This is a
build check on the maintainer host, not a clean-host CI run. The full hardware
workspace, Livox driver and external camera driver still require their SDKs and
were not rebuilt in this pass.

## License and data boundaries

- The FAST-LIVO2-derived component retains **GPLv2** (root and package `LICENSE`
  are byte-identical; SHA-256
  `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`).
- `src/FAST_LIVO2/package.xml` now says `GPLv2`, matching the license. The old
  `BSD` tag was never a relicensing grant.
- `src/rpg_vikit/vikit_common` declares GPLv3 and compiles GPLv3 components
  (`robust_cost.cpp`, `homography.cpp`) into a linked shared library;
  `src/gnss_comm` declares GPL-3.0-or-later. The GPLv2-only/GPLv3 combined-work
  question is unresolved. A submodule or separate clone does not solve it.
- The MVS wrapper's `TODO` license and the STM32 vendor notices remain
  unresolved upstream. Removing those paths from public tracking limits what
  this repository redistributes; it does not grant new rights to anyone who
  installs them separately.
- No component is relicensed to MIT, BSD or GPLv3 by this fork.
- See [third-party terms](../THIRD_PARTY_LICENSES.md) for the file-level register
  and upstream contacts. The repository is **not certified release-ready**.
- The tunnel bag is not in this repository. The maintainer states it is
  self-collected and shareable; users must still check privacy, participant and
  site constraints before redistributing it. Competition, GNSS and UWB recordings
  have no verified public redistribution grant here.

## Codex for Open Source preparation

The public repository is <https://github.com/gulu996/FAST-LIVO-UG>. Factual
material available for an application:

- A maintained ROS 1 FAST-LIVO2 fork with a separate GTSAM fixed-lag GNSS/UWB
  backend and transaction-oriented frontend.
- A public tree restricted to the components needed to build that core, with
  optional hardware and calibration tooling documented as external.
- A native geometry test, source-available transaction tests, and a documented
  public tunnel replay entry.
- A concrete license register with named upstream contacts and an explicit
  unresolved combined-work issue, rather than an unsupported “open-source ready”
  claim.
- Maintainer-run evidence on the public algorithm tree and separate internal
  competition evidence; no adoption, dependent-project or user-count claims are
  invented.

A support proposal can target reproducible geometry/transaction regressions,
code review, issue triage and release checks. API-credit requests should state
the workflow and resources actually needed. The program terms and current form
must be reviewed at submission time; no application or message to OpenAI was
sent by this task.

## Documentation checks performed

- Internal Markdown links and code fences checked; external tunnel link checked
  by HTTP and Baidu share metadata on 2026-10-28.
- `git diff` reviewed to exclude `config/mid360.yaml`, the local Hikrobot driver,
  ZED, GNSS/UWB runtime configuration, bags, logs, binaries and untracked
  internal reports.
- The bag's `rosbag info` was checked on the maintainer copy; no new long SLAM
  replay was run for this cleanup.
- No push, no history rewrite and no destructive cleanup was performed.
