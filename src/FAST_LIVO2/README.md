# FAST-LIVO2 component of FAST-LIVO-UG

This is a modified version of **FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual
Odometry**, developed upstream by Chunran Zheng and collaborators at HKU MARS.

Use the [workspace README](../../README.md) for this fork's build and run commands,
[GNSS/UWB modes](../../docs/FUSION_MODES.md), and
[versioned experiments](../../docs/experiments/README.md). Clone the whole
workspace, not this directory inside another copy of the repository.

Upstream references:

- [FAST-LIVO2 source](https://github.com/hku-mars/FAST-LIVO2)
- [FAST-LIVO2 paper](https://arxiv.org/abs/2408.14035)
- [FAST-LIVO paper](https://arxiv.org/abs/2203.00893)
- [Hardware and synchronization](https://github.com/xuankuzcr/LIV_handhold)
- [FAST-Calib](https://github.com/hku-mars/FAST-Calib)
- [Upstream dataset access information](https://github.com/hku-mars/FAST-LIVO2#4-run-our-examples)

Upstream datasets are distinct from the private competition and local GNSS/UWB
recordings used by this fork. No download or redistribution permission for those
local recordings is implied.

This package retains its [GPLv2 LICENSE](LICENSE). For an alternative commercial
license, upstream names Chunran Zheng (`zhengcr@connect.hku.hk`) and Prof. Fu Zhang
(`fuzhang@hku.hk`). See [third-party terms and open issues](../../THIRD_PARTY_LICENSES.md).
The inherited `package.xml` BSD tag conflicts with the package license and has
not been changed in this documentation-only commit; it is not a relicensing grant.

## Backend ownership

GNSS/UWB factors consume transaction-completed raw odometry in the independent
GTSAM fixed-lag smoother; optimized poses never overwrite frontend state/P.
Rejected visual queries publish no speculative backend motion. A completed LIO
scan can publish legal prediction on measurement rejection. With managed policy,
`gnss_fusion/enable` selects GNSS and `rtk_backend/uwb_factor_backend_en` selects
UWB; `rtk_backend/enable` must be true for either. UWB anchors must already be in
the backend map frame. Legacy frontend fusion update selectors fail startup.

Documentation reorganized for this fork on 2026-10-26; original installation,
GTSAM and recording guidance is consolidated in the
[build/capture guide](../../docs/BUILD_AND_CAPTURE.md).
