# Doorway drift, plane geometry and VIO map queries

**Purpose:** separate implementation defects from unverified positioning error
and preserve the history of failed candidates. Data: private competition scans,
images and real maps around 415 s, 418/419 s and the later failure window.
No real-map fixture or site image is redistributed.

`027fc06` adds nondegenerate-plane validation and a shared squared tangential
point-to-plane distance gate. Four other systematic-audit candidates were rejected
and are not bundled into this result. Its LIVO first/last distance was 3.805381 m;
this alone was neither an accuracy pass nor a proven new fault.

The next **index-only** candidate queried the 0.8 m LiDAR map with matching keys
instead of the old 0.5 m visual grid, but failed near 502.306 s. It was not merged.
`d9497af` retains correct map indexing and also checks the positive depth of
plane-warp patch support in both cameras. Visual storage grid size remains separate.

A saved real fixed-input counterexample had 8 of 192 sampled positions behind a
camera while the old patch still had NCC 0.993023 and MSE 4.968121. The new guard
first changed a VIO posterior at 1.045082 s, reducing that frame's tracked patches
from 35 to 34. This proves a specific invalid-measurement path; it does not isolate
the unique cause of the later 502 s failure. No actual VIO acceptance occurred in
the old index-only failure's 488–502 s interval: accumulated history/LIO/map feedback
cannot be replaced by a claim of one bad VIO at 502 s.

| 420–430 s world-axis extent (m) | Previous two-fix version | Current candidate |
|---|---:|---:|
| x | 0.140426 | 0.120074 |
| y | 0.366794 | 0.378971 |
| z | 0.067425 | 0.088251 |

These are descriptive extents during a doorway segment, not lateral-error truth.
The current recorded 415–420 s extent is [0.875960, 2.216186, 0.164031] m.
A pure-LIO reference is not ground truth. No trajectory was forced to close.
Latest full LIVO first/last distance is 1.043327 m; accepted VIO count is 3467
(previous two-fix version 3564). Counts are not accuracy scores.

**Public native reproducer:**

```bash
source devel/setup.bash
rosrun fast_livo lio_plane_map_self_test
```

Source: [`lio_plane_map_self_test.cpp`](../../src/FAST_LIVO2/src/lio_plane_map_self_test.cpp).
It checks the shared geometry/key and patch-depth helpers with explicit fixtures.
The actual long command/config is in [competition-804.md](competition-804.md).
Local real-input proof/report hashes are in [evidence_manifest.json](evidence_manifest.json).
Raycast-enabled full duration, complete occlusion and plane uncertainty handling
remain unverified. New docs do not re-run or reinterpret rejected candidates.
