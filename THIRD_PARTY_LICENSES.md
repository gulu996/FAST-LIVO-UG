# Licensing and third-party attribution

Reviewed against tracked files at `d9497af` on 2026-10-26. This is a source
inventory and unresolved-permissions register, **not clearance of every component**.
No uncertain code, driver or firmware has been deleted or relicensed.

## FAST-LIVO2-derived code

The root [LICENSE](LICENSE) is an unchanged copy of
[`src/FAST_LIVO2/LICENSE`](src/FAST_LIVO2/LICENSE), GNU GPL version 2.
It documents the license of the FAST-LIVO2-derived component and modifications;
it does **not** override per-file or separately bundled third-party terms.
Upstream's [LICENSE](https://github.com/hku-mars/FAST-LIVO2/blob/main/LICENSE)
and [README](https://github.com/hku-mars/FAST-LIVO2#5-license) identify GPLv2.
We have not asserted an unverified “or later” grant for upstream code.

Preserve all source attribution, copyright, warranty and license notices,
including Chunran Zheng, Fu Zhang and earlier LOAM authors where present.
Alternative commercial licensing is a matter for upstream rights holders, not
permission supplied by this fork. GPL distribution obligations, including
corresponding source for distributed binaries and change notices, still apply.
Before a release, audit modified-file notices as well as Git history; this docs
commit does not rewrite historical source headers.

**Metadata discrepancy:** `src/FAST_LIVO2/package.xml` says `BSD`, inherited from
older package metadata. The source/package license says GPLv2. The tag is not
permission to redistribute this derived code under BSD. Correcting package
metadata is a follow-up source-release action; it was left unchanged to keep this
commit documentation-only.

## Tracked components and dependencies

| Component / concrete paths | Evidence / attribution | Distribution status |
|---|---|---|
| `src/FAST_LIVO2` | GPLv2 package LICENSE; developer notices in `src/vio.cpp`, `include/IMU_Processing.h`, etc. | Retain GPLv2 and notices; linked-license issue below remains open |
| `src/rpg_vikit/vikit_common`, `vikit_ros` | Package metadata GPLv3; `homography.h/.cpp` credits Isis Innovation Limited, 2008 / PTAM-GPL; `file_reader.h`, `robust_cost.h/.cpp` credit Christian Kerl, 2012 / dvo, GPLv3-or-later; `ringbuffer.h` has Hauke Strasdat's permissive notice | Mixed per-file terms; GPLv3 components actually built into `vikit_common`; resolve compatibility before distribution of the combined application |
| `src/rpg_vikit/vikit_py` | Package declares BSD | No package-level full license text found; verify provenance/notice completeness |
| `src/gnss_comm` | [GPLv3 text](src/gnss_comm/LICENSE), package `GPL-3.0-or-later`; current tree contains message definitions | Retain GPLv3; review generated-message/header use in the combined GPLv2 application, do not silently relicense |
| `src/livox_ros_driver2` | [Livox MIT notice and scope](src/livox_ros_driver2/LICENSE.txt) | Preserve its specified-file scope and copyright; independently check SDK terms |
| `src/livox_ros_driver2/3rdparty/rapidjson` | [Tencent / Milo Yip license and dependency exceptions](src/livox_ros_driver2/3rdparty/rapidjson/license.txt) | Retain full notice and exceptions, not just the word MIT |
| `src/FAST_LIVO2/include/livox_ros_driver/CustomMsg.h`, `CustomPoint.h` | Bundled generated legacy message headers | Verify source message version/provenance and notices before release; not a substitute for driver SDK licensing |
| `src/FAST_LIVO2/visionpro_web_viewer/vendor` | [MIT notice](src/FAST_LIVO2/visionpro_web_viewer/vendor/LICENSE), three.js authors, 2010–2023 | Preserve notice with bundled JS |
| `src/gnss_serial_driver`, `uwb_serial_driver`, `sensor_time_bridge`, `sensor_time_msgs`, `sensor_recording_bringup` | Each package declares BSD-3-Clause | Owner should confirm authorship and add complete copyright/license notices for these separate packages; root GPL text does not replace their terms |
| `src/mvs_ros_driver/package.xml`, `src/grab_trigger.cpp`, `beifen/grab_trigger.cpp` within that package | License tag `TODO`; wrapper references vendor SDK | **Unresolved** wrapper redistribution permission and provenance; SDK is external, see below |
| `src/stm32_timersync-open/CORE/core_cm3.{h,c}` | ARM Limited, 2009; notice permits distribution within tools supporting ARM processors | Scope-specific grant, not assumed GPL/MIT; confirm intended redistribution |
| `src/stm32_timersync-open/HARDWARE/{KEY,LED,TIMER}`, `SYSTEM/{delay,sys,usart}` | Vendor copyright notices in legacy encoding | **Unresolved** redistribution grant for vendor-derived routines |
| `src/stm32_timersync-open/USER/stm32f10x*`, `system_stm32f10x*`, startup files | ST/firmware headers carry their own notices | Review exact version/terms and notice completeness, including bundled Keil project/listing/map files |
| GTSAM 4.2 (installed externally) | [BSD-style license](https://github.com/borglab/gtsam/blob/4.2/LICENSE), with its own bundled dependencies | Follow exact installed GTSAM and third-party notices; not vendored here |
| Sophus `a621ff`, ROS, Eigen, PCL, OpenCV, Boost, optional mimalloc/OpenMP | Installed dependencies, not granted rights by this repository | Retain exact-version license records when redistributing a binary image |

## Linked GPL version question — unresolved release blocker

`vikit_common/CMakeLists.txt` compiles `src/homography.cpp` and `src/robust_cost.cpp`
into a shared library. The FAST-LIVO2 catkin link set uses that library. This is
more than an unused GPLv3 file in an archive. GPLv2-only and GPLv3 cannot simply
be combined under a newly written root license; see the
[GNU compatibility explanation](https://www.gnu.org/licenses/rms-why-gplv3.en.html).

Ask the relevant rights holders to clarify the upstream version grant or a
compatible permission/exception, and review the generated `gnss_comm` boundary.
We did not assume permission, add an exception, promote everything to GPLv3,
or remove code to hide this issue. The existing public repository's licensing
status requires maintainer follow-up independently of these documentation changes.

## Vendor SDKs, local projects and data

- MVS is loaded from `/opt/MVS`; CMake selects architecture-specific vendor
  libraries. No tracked `.so`/`.a` was found in this review. Do not upload the SDK
  installer, headers, examples or libraries without checking the vendor grant.
  Absence of SDK binaries does not resolve the wrapper's `TODO` license.
- Livox-SDK2 is separately installed. Its license is not implied by the ROS
  driver's MIT notice. Follow the [SDK's own repository](https://github.com/Livox-SDK/Livox-SDK2).
- The local `FAST-Calib` directory is not tracked at this revision. Obtain it
  separately from [upstream](https://github.com/hku-mars/FAST-Calib); do not infer
  its distribution from an older remote tree listing.
- The local ZED worktree and Stereolabs SDK are not part of this documentation
  change and are not authorized for inclusion by this repository's license.
- `rpg_vikit/vikit_common/test/data/scene_000.png` is existing third-party test
  imagery. Its data-specific permission was not established; it is not reused in
  new docs. Existing vendor/UI code is not new authorship by this project.
- Competition, GNSS and UWB recordings have no verified public redistribution
  grant here. No bag, PCD, site photo, trajectory map or raw sensor payload is
  added. New experiment pages publish aggregate software observations only.

## Release actions still needed

Resolve the linked GPL question; confirm driver/firmware rights; complete the
separate BSD package notices; correct the contradictory package metadata; check
historical modified-file notices; obtain permissions for any future data/images.
Keep exact versions and license texts with release artifacts. A documentation
commit or GitHub license badge cannot certify those actions.
