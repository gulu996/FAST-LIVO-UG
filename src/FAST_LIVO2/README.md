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

Upstream datasets are distinct from the maintainer's recordings. The private
competition and local GNSS/UWB recordings have no public download grant here.
The self-collected tunnel recording is the public reproduction entry; see
[tunnel LIVO](../../docs/experiments/tunnel.md) for its link, file identity and
run commands. That share is not a redistribution grant for the competition or
GNSS/UWB data.

This package retains its [GPLv2 LICENSE](LICENSE). The `package.xml` license tag
was corrected from `BSD` to `GPLv2` in the 2026-10-28 cleanup; the old tag was
never a relicensing grant. For an alternative commercial license, upstream names
Chunran Zheng (`zhengcr@connect.hku.hk`) and Prof. Fu Zhang (`fuzhang@hku.hk`).
See [third-party terms and open issues](../../THIRD_PARTY_LICENSES.md), including
the unresolved GPLv2-only/GPLv3 combined-work question.

## Backend ownership

GNSS/UWB factors consume transaction-completed raw odometry in the independent
GTSAM fixed-lag smoother; optimized poses never overwrite frontend state/P.
Rejected visual queries publish no speculative backend motion. A completed LIO
scan can publish legal prediction on measurement rejection. With managed policy,
`gnss_fusion/enable` selects GNSS and `rtk_backend/uwb_factor_backend_en` selects
UWB; `rtk_backend/enable` must be true for either. UWB anchors must already be in
the backend map frame. Legacy frontend fusion update selectors fail startup.

Documentation reorganized for this fork on 2026-10-28. The public baseline is
`e991e383`; its algorithm commit is `9fe7e510` (tree-identical to the tested
`5be5538`). The cleanup branch adds documentation, license and public-tree
changes on top. Original installation, GTSAM and recording guidance is
consolidated in the [build/capture guide](../../docs/BUILD_AND_CAPTURE.md).
