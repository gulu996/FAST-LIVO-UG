# Release and application readiness

Documentation baseline: **2026-10-28**. The catkin workspace root is the
repository; `src/FAST_LIVO2` is the package.

- Maintainer-local `main`: `5be5538b922ab3a621cd902ecb60b2658aeb3685`
  (`fix(lio): use floor voxel keys and metre neighbourhood units`).
- Public `origin/main`: `02343a3` (`fix(vio): separate observation maintenance
  from reference selection`).
- Local `main` is one commit ahead of the public branch. **No push was performed
  by this documentation change.**
- The maintainer's uncommitted `config/mid360.yaml` (`img_en: 1`), the Hikrobot
  driver changes, the ZED worktree and the GNSS/UWB backend files were preserved.

## Ready now / still needed

| Item | State |
|---|---|
| English architecture, build and run documentation | Updated for the public tunnel entry; statically checked, not a fresh install test |
| Public tunnel dataset | Share link and extraction code checked 2026-10-28; file name/size recorded; local SHA-256 recorded |
| A5/A1 voxel-key and neighbour-unit fix | In maintainer-local `5be5538`; native test, tunnel replay and internal competition evidence exist; **not on public `origin/main`** |
| Public tunnel result | 3542 LIO, 0.542 m first/last, 2.02 m/s maximum adjacent-scan speed; revisit diagnostic, not ATE |
| Restricted 804 s competition / DROP2 / real-map geometry | Existing summaries; private data and harness still restrict exact replay; **not the tunnel dataset** |
| GNSS, UWB and joint graph | Separate real 90 s software tests and a synthetic joint graph; no real simultaneous field-accuracy certification |
| License clearance for all distributed components | **Not complete**: GPLv2/GPLv3 combination, MVS wrapper `TODO`, firmware/vendor notices |
| Fresh clone build / CI | Still needed on a clean authorized environment; no clean-clone build was performed |
| Remote publication / application | No push; the OpenAI Codex for OSS form was not submitted by this task |

## Public reproduction status

The public tunnel share is the recommended reproduction entry. The link returned
HTTP 200, extraction code `mbpr` was accepted, and the share listing reported
`suidao2_resized.bag` at 16,334,415,367 bytes. The maintainer's exact-size local
copy has SHA-256
`9289318857b0ca009dab4570ac6f51915de1681fe60b2abc0b481c20d6711169`.
The file was not re-downloaded in this documentation pass, so this is link and
metadata verification, not an independent download/replay.

The 0.542 m tunnel result is tied to maintainer-local `5be5538`. Because that
commit is not on the public branch, the result is **maintainer-verified, not
public-code reproducible** until the commit is published. No third-party
reproduction has been reported.

## License and data boundaries

- The FAST-LIVO2-derived component retains **GPLv2** (root and package `LICENSE`
  are byte-identical; SHA-256
  `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`).
- `src/FAST_LIVO2/package.xml` still says `BSD`; this inherited metadata tag is
  **not** a relicensing grant and remains a correction action.
- `src/rpg_vikit/vikit_common` declares GPLv3 and compiles GPLv3 components into
  a linked shared library; `src/gnss_comm` declares GPL-3.0-or-later. The
  GPLv2/GPLv3 combined-work question is unresolved.
- `src/mvs_ros_driver` has a `TODO` license and depends on an external vendor SDK;
  STM32 vendor-derived files carry their own unresolved notices. No component is
  relicensed to MIT by this fork.
- See [third-party terms](../THIRD_PARTY_LICENSES.md) for the file-level register.
  The repository is **not certified release-ready**.
- The tunnel bag is not in this repository. The maintainer states it is
  self-collected and shareable; users must still check privacy, participant and
  site constraints before redistributing it. Competition, GNSS and UWB recordings
  have no verified public redistribution grant here.

## Codex for Open Source preparation

The public repository is <https://github.com/gulu996/FAST-LIVO-UG>. Factual
material available for an application:

- A maintained ROS 1 FAST-LIVO2 fork with a separate GTSAM fixed-lag GNSS/UWB
  backend and transaction-oriented frontend.
- A native geometry test, source-available transaction tests, and a documented
  public tunnel replay entry.
- A concrete license action plan and a release checklist rather than an
  unsupported “open-source ready” claim.
- Maintainer-run evidence on local `5be5538` and separate internal competition
  evidence; no adoption, dependent-project or user-count claims are invented.

A support proposal can target reproducible geometry/transaction regressions,
code review, issue triage and release checks. API-credit requests should state
the workflow and resources actually needed. The program terms and current form
must be reviewed at submission time; no application or message to OpenAI was
sent by this task.

## Documentation checks performed

- Internal Markdown links and code fences checked; external tunnel link checked
  by HTTP and Baidu share metadata on 2026-10-28.
- The bag's `rosbag info` was checked on the maintainer copy; no new long SLAM
  replay was run for this documentation commit.
- Git diff reviewed to exclude `config/mid360.yaml`, the Hikrobot driver, ZED,
  GNSS/UWB code, bags, logs, binaries and untracked internal reports.
- No push, no history rewrite and no destructive cleanup was performed.
