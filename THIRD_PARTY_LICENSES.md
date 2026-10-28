# Licensing and third-party attribution

Reviewed against the public tree at `e991e383` and the 2026-10-28 cleanup
candidate. This is a source inventory and an unresolved-permissions register,
**not clearance of every component**. The cleanup removes two optional hardware
trees and one broken gitlink from public tracking, and corrects one contradictory
metadata tag; it does not relicense any third-party code.

## FAST-LIVO2-derived code

The root [LICENSE](LICENSE) is an unchanged copy of
[`src/FAST_LIVO2/LICENSE`](src/FAST_LIVO2/LICENSE), GNU GPL version 2. It
documents the license of the FAST-LIVO2-derived component and modifications; it
does **not** override per-file or separately bundled third-party terms.
Upstream's [LICENSE](https://github.com/hku-mars/FAST-LIVO2/blob/main/LICENSE)
and [README](https://github.com/hku-mars/FAST-LIVO2#5-license) identify GPLv2.
The upstream `LICENSE` and package metadata reviewed on 2026-10-28 do **not**
state “or any later version”. This fork therefore treats the FAST-LIVO2-derived
code as **GPLv2-only**, not GPLv2-or-later.

`src/FAST_LIVO2/package.xml` previously said `BSD` while the package contained
the GPLv2 `LICENSE`. That inherited tag was metadata, not a relicensing grant.
The cleanup changes the tag to `GPLv2`; this corrects the metadata to match the
existing license and does not change the license of any file.

Preserve all source attribution, copyright, warranty and license notices,
including Chunran Zheng, Fu Zhang and earlier LOAM authors where present.
Alternative commercial licensing is a matter for upstream rights holders, not
permission supplied by this fork. GPL distribution obligations, including
corresponding source for distributed binaries and change notices, still apply.
Historical modified-file notices still require review before a binary release;
this cleanup does not rewrite source headers or Git history.

## Public tree inventory

| Component / concrete paths | Evidence / attribution | Distribution status |
|---|---|---|
| [`src/FAST_LIVO2`](src/FAST_LIVO2) | GPLv2 package `LICENSE`; developer notices in `src/vio.cpp`, `include/IMU_Processing.h`, etc.; upstream package metadata corrected to `GPLv2` | Retained; linked-license issue below remains open |
| [`src/gnss_comm`](src/gnss_comm) | [GPLv3 text](src/gnss_comm/LICENSE), package `GPL-3.0-or-later`; message definitions only; maintainer CAO Shaozu | Retained because `fast_livo` includes its messages; combined-work question below |
| [`src/rpg_vikit`](src/rpg_vikit) | Package metadata GPLv3 for `vikit_common`/`vikit_ros`, BSD for `vikit_py`; full GPLv3 text added to [`src/rpg_vikit/LICENSE`](src/rpg_vikit/LICENSE) with the scope stated in [`NOTICE`](src/rpg_vikit/NOTICE); no separate package-level text found for `vikit_py` | Retained; GPLv3 files are compiled and linked, see below |
| [`src/livox_ros_driver2`](src/livox_ros_driver2) | [Livox MIT notice and scope](src/livox_ros_driver2/LICENSE.txt); package metadata MIT | Retained as an optional hardware driver; not required by the core build |
| [`src/livox_ros_driver2/3rdparty/rapidjson`](src/livox_ros_driver2/3rdparty/rapidjson) | [Tencent / Milo Yip license and dependency exceptions](src/livox_ros_driver2/3rdparty/rapidjson/license.txt) | Retain the full notice and exceptions, not just the word MIT |
| `src/FAST_LIVO2/include/livox_ros_driver/CustomMsg.h`, `CustomPoint.h` | Bundled generated legacy message headers | Verify source message version/provenance and notices before release; not a substitute for driver or SDK licensing |
| [`src/FAST_LIVO2/visionpro_web_viewer/vendor`](src/FAST_LIVO2/visionpro_web_viewer/vendor) | [MIT notice](src/FAST_LIVO2/visionpro_web_viewer/vendor/LICENSE), three.js authors, 2010–2023 | Preserve notice with bundled JS |
| [`src/gnss_serial_driver`](src/gnss_serial_driver), [`src/uwb_serial_driver`](src/uwb_serial_driver), [`src/sensor_time_bridge`](src/sensor_time_bridge), [`src/sensor_time_msgs`](src/sensor_time_msgs), [`src/sensor_recording_bringup`](src/sensor_recording_bringup) | BSD-3-Clause package metadata; complete BSD-3-Clause `LICENSE` added to each in this cleanup, copyright `2026, gulu996` | Retained; repository history in this fork shows `gulu996` as the tracked author, with no identified upstream package source |

### Optional hardware removed from public tracking

These paths are **not in the public candidate tree**. The maintainer's local
files remain untouched; they are omitted only from public Git tracking.

| Local path | Reason and upstream | Public replacement |
|---|---|---|
| `src/mvs_ros_driver` (Hikrobot/Hikvision camera wrapper) | `package.xml` license tag is `TODO`; the wrapper loads a separately installed proprietary MVS SDK. The upstream [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold) top-level GPLv3 does not by itself resolve the wrapper's missing package license | Install the camera driver separately from LIV_handhold (or the vendor); the core build and tunnel bag replay do not need it |
| `src/stm32_timersync-open` (STM32 trigger firmware) | Not a ROS package. Vendor notices in `CORE/`, `STM32F10x_FWLib/`, `SYSTEM/` and `USER/` are not erased by LIV_handhold's top-level GPLv3; several files carry ST/ARM terms | Obtain the firmware and its notices from LIV_handhold; not required by the core build |
| `src/FAST-Calib` | Was recorded as a bare gitlink (`160000`) without a `.gitmodules` entry; it is calibration tooling, not a core dependency | Obtain from [hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib) |

The public tree therefore contains the FAST-LIVO2 frontend, the GNSS/UWB factor
backend and its message/time-sync support, the self-developed serial drivers,
and the MIT-licensed Livox ROS driver. `sensor_recording_bringup` keeps a
runtime reference to the camera driver only inside an `enable_camera` group; the
default is now `false`, and its `package.xml` no longer declares `mvs_ros_driver`
as an execution dependency.

## Linked GPL version question — unresolved release blocker

`src/rpg_vikit/vikit_common/CMakeLists.txt` compiles both
`src/homography.cpp` and `src/robust_cost.cpp` into `libvikit_common.so`.
`fast_livo` links that library and includes `<vikit/robust_cost.h>` from
`src/FAST_LIVO2/include/vio.h` and related headers. The GPLv3 files are thus
built and linked into the combined application, not merely stored unused:

- `src/rpg_vikit/vikit_common/src/robust_cost.cpp`
- `src/rpg_vikit/vikit_common/src/homography.cpp`
- `src/rpg_vikit/vikit_common/include/vikit/robust_cost.h`
- `src/rpg_vikit/vikit_common/include/vikit/homography.h`
- `src/rpg_vikit/vikit_common/include/vikit/file_reader.h` (GPLv3 notice; not
  currently included from the FAST-LIVO2 sources found in this audit)

`src/gnss_comm` is GPL-3.0-or-later and is a build dependency for its message
headers. Treating it as “just messages” does not settle the combined-work
question.

The FAST-LIVO2-derived code is GPLv2-only (no upstream “or later” grant).
GPLv2-only and GPLv3 code cannot simply be combined and distributed under a new
root license; see the
[GNU compatibility explanation](https://www.gnu.org/licenses/rms-why-gplv3.en.html).
A submodule, a separate clone, a system package, or a second shared library does
**not** by itself remove the combined-work question. We also checked whether the
unused-looking `robust_cost.h` include or the homography source could simply be
removed; the `vikit_common` package is declared GPLv3 as a whole, so dropping
one include/source would not establish a permissive boundary and would be a
code/build change rather than a license fix. We found no functionality-preserving
legal measure that resolves the issue, and this cleanup did not add an exception,
upgrade the whole project to GPLv3, downgrade the GPLv3 components, or delete
code to hide the issue.

Permission or a compatible exception is required. Relevant upstream contacts
and files:

| Upstream / contact | Files in question |
|---|---|
| FAST-LIVO2 authors: Chunran Zheng `zhengcr@connect.hku.hk`, Fu Zhang `fuzhang@hku.hk` ([upstream](https://github.com/hku-mars/FAST-LIVO2)) | `src/FAST_LIVO2` GPLv2-only grant; request a GPLv3-compatible/dual-license clarification or exception |
| rpg_vikit maintainer: Christian Forster `forster@ifi.uzh.ch` ([upstream](https://github.com/xuankuzcr/rpg_vikit)) | `src/rpg_vikit/vikit_common`, `vikit_ros` GPLv3 package terms |
| dvo/`robust_cost` author: Christian Kerl (GPLv3-or-later notices in the file; contact through the dvo project) | `vikit_common/src/robust_cost.cpp`, `include/vikit/robust_cost.h` |
| PTAM/Isis Innovation notice (via the vikit/PTAM upstream) | `vikit_common/src/homography.cpp`, `include/vikit/homography.h` |
| `gnss_comm` maintainer: CAO Shaozu `shaozu.cao@gmail.com` ([package metadata](src/gnss_comm/package.xml)) | GPL-3.0-or-later message package |

Until a rights holder grants compatible terms, the public repository is **not
certified as a distributable combined work**, even though the source is
available. This is an unresolved release item, not a claim that the issue has
been solved by the cleanup.

## Vendor SDKs, local projects and data

- MVS is loaded from `/opt/MVS`; CMake selects architecture-specific vendor
  libraries. No tracked `.so`/`.a` was found. Do not upload the SDK installer,
  headers, examples or libraries without checking the vendor grant. Removing the
  wrapper from public tracking does not resolve the upstream wrapper's `TODO`
  license; it only stops this repository from redistributing that file.
- Livox-SDK2 is separately installed. Its license is not implied by the ROS
  driver's MIT notice. Follow the [SDK's own repository](https://github.com/Livox-SDK/Livox-SDK2).
- The local ZED worktree and Stereolabs SDK are not part of the public tree and
  are not authorized for inclusion by this repository's license.
- `rpg_vikit/vikit_common/test/data/scene_000.png` is existing third-party test
  imagery. Its data-specific permission was not established. Existing vendor/UI
  code is not new authorship by this project.
- The maintainer's self-collected tunnel recording is shared outside this
  repository (Baidu Netdisk; see
  [docs/experiments/tunnel.md](docs/experiments/tunnel.md)). The maintainer states
  that it is shareable. No bag is added here; users must check privacy,
  participant and site constraints before further redistribution.
- Competition, GNSS and UWB recordings have no verified public redistribution
  grant here. No bag, PCD, site photo, trajectory map or raw sensor payload is
  added. Experiment pages publish aggregate software observations only.

## Historical reports in this repository

Some tracked maintenance reports under the repository root and
`src/FAST_LIVO2/reports/` describe the pre-cleanup tree, including paths under
`src/mvs_ros_driver` and `src/stm32_timersync-open`. They are historical records
of maintainer work, not a statement that those paths are publicly tracked. Do
not infer a redistribution right from a path appearing in a report.

## Release actions still needed

1. Obtain a compatible license grant or exception for the GPLv2-only/GPLv3
   combination described above, or replace the GPLv3 components with
   independently licensed implementations as a separately reviewed code change.
2. Confirm driver/firmware rights if the externalized MVS wrapper or STM32
   firmware is ever redistributed again.
3. Audit historical modified-file notices and the bundled Livox message headers.
4. Obtain data/image permissions for any future redistribution.
5. Keep exact dependency versions and license texts with release artifacts.

A documentation commit or GitHub license badge cannot certify these actions.
