# Reproducibility and experiment index

These are existing measurements, not experiments rerun for this documentation.
Raw files remain outside Git. [evidence_manifest.json](evidence_manifest.json)
identifies original reports/results by archive-relative path and SHA256.
A hash establishes artifact identity, not independent verification or data rights.
[Recorded commands](recorded_commands.json) preserve the actual long-run argument
arrays with private filesystem prefixes replaced by variables. They require the
original local harness; they are not a new public replay script.

| Experiment | Publicly runnable now? | Evidence scope |
|---|---|---|
| [Public tunnel LIVO](tunnel.md) | **Dataset yes**: Baidu share link/code checked; exact result requires maintainer-local `5be5538` (not pushed) | 3542 LIO, 0.542 m first/last; A5/A1 voxel-key/neighbour fix; not ATE |
| [804 s LIO/LIVO](competition-804.md) | **Not end-to-end**: private bag and diagnostic harness required; **not the tunnel dataset** | Completed isolated candidate runs; integrated build checked at 20 s |
| [DROP2](drop2.md) | Native query/transaction tests are source-available; exact replay is restricted | Two explicitly omitted unaccepted images; common persisted state/P equality |
| [415 s geometry and VIO](geometry.md) | Native geometric counterexamples yes; real-map replay restricted | Point/plane and patch-depth defects, descriptive trajectory windows |
| [GNSS independent](gnss.md) | Native adapter/factor tests yes; recorded 90 s test restricted | Real GNSS data through adapter and backend; no accuracy truth |
| [UWB independent](uwb.md) | Native range/Jacobian tests yes; recorded 90 s test restricted | Real ranges, separate dataset; device timing/calibration limitations |
| [Joint factor interfaces](joint-backend.md) | Yes, synthetic fixture source and commands available | Same graph/key, declared common frame/time; not field accuracy |

“Publicly runnable” assumes an authorized build of dependencies; it does not
resolve [license issues](../../THIRD_PARTY_LICENSES.md). The linked native checks
are existing source targets, not new simulations claiming to reproduce private data.
No external reviewer has yet independently reproduced the private 804 s results
or the public tunnel result.

## Version map

| Commit | Meaning |
|---|---|
| `dd4e9d7` | EOF callback scheduling uses WallRate |
| `b2dedce` | Integrated IMU/deskew initialization and atomic query/commit frontend |
| `dd35f77fa0c1ff4728eace5330d7d9905ce36534` | Remove legacy frontend GNSS/UWB updates; transaction-completed backend publication |
| `027fc06a7ffcf7209b22108d079848a3b860da23` | Plane validity and shared squared tangential-distance fixes |
| `d9497af2234d3fd368dfdc072918d81c5d59a585` | Correct VIO map key/size and reject invalid plane-warp depth support |
| `02343a3` | Separate visual observation maintenance from reference selection; current public `origin/main` |
| `5be5538` | Floor-based LiDAR voxel keys and metre-based neighbour queries; maintainer-local, one commit ahead of public `origin/main` |

The current documentation describes maintainer-local `5be5538`; the public branch
is `02343a3`. The tunnel result requires `5be5538` and is not yet reproducible
from the public branch alone. Backend real-input tests are from `dd35f77`;
later frontend revisions did not modify those backend libraries, but that is not
a new real joint-hardware test. Older rejected P5, global rebinning, A2
covariance, A6 small-angle and index-only candidates are not the current
released algorithm.

## Metrics and publication policy

- First-to-last position distance is a revisit-consistency diagnostic, **not ATE**.
- Maximum scan-difference speed is a stability diagnostic, not physical truth.
- Two algorithm trajectories are reference runs, not ground truth for each other.
- A local world-x extent is not automatically lateral/east error.
- Accepted visual count is not an accuracy score.
- Image omission invariance depends on which measurements actually commit.
- Independent real GNSS and UWB validation is not simultaneous field validation.

[Aggregate stability figure](assets/stability.svg) uses only the table statistics
in this repository; it contains no site images, map geometry or geographic coordinates.
No site photos or original screenshots were copied. Dataset owners must authorize
any future distribution of recordings, raw trajectories or real-map fixtures.
The [tunnel recording](tunnel.md) has a checked public share link and is the
public reproduction entry; it is not the competition recording. The competition
recording has no verified public URL here, and its 804 s numbers must not be
attributed to the tunnel data. Public upstream FAST-LIVO2 datasets are different
data and cannot validate these exact counts.
