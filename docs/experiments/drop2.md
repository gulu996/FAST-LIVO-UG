# DROP2 visual-query and persistent-state consistency

**Purpose:** verify that image queries which do not commit a measurement do not
silently consume persistent IMU state, and test bounded full-duration behavior.
Dataset/config/build: the same restricted competition recording and `d9497af`
candidate as [the 804 s test](competition-804.md).

Only these two exact header stamps were withheld by the traced decoder:

```text
1785914769.106157476
1785914769.139537476
```

Original image identity/payload was checked in the local experiment. No timestamp
was changed, no state/P copied from NO_DROP, and no artificial event inserted.
Both images produced no accepted VIO measurement in the normal run.

| Pair check | Result |
|---|---|
| Common LIO state/P records | 4029 / 4029 exact |
| Common persistent transaction records | 28128 / 28128 exact |
| Actual selected IMU groups | 28129 / 28129 exact |
| Effective integration supports | 28127 / 28127 exact |
| Accepted visual identities | 3467 / 3467, same sequence |
| Native LIO transactions | 4029 / 4029 exact |

Pairing used event time/type and measurement identity, not row number. External
future-receive queue lengths were excluded only where actual selected input and
persistent release/propagation semantics were checked. Full per-step F/Q records
were not independently captured; formula identity is not a substitute for such logs.
Both trajectories completed without the frozen severe-instability trigger.
This does not prove invariance to dropping accepted measurements or arbitrary images.

**Actual command:** the NO_DROP execution form in `competition-804.md`, with
`runs/DROP2_804/run.launch` and its own output directory. The archived decoder
hook was the only intended injection. Stock `rosbag play` has no DROP2 argument;
no nonexistent public flag is advertised here.

**Public checks after build:** existing transaction and scan-contract test sources
are `src/FAST_LIVO2/src/imu_transaction_self_test.cpp`,
`vio_update_transaction_self_test.cpp`, `livo_scan_transaction_self_test.cpp`.
For example:

```bash
rosrun fast_livo imu_transaction_self_test
rosrun fast_livo livo_scan_transaction_self_test
```

These are software fixtures, not reproduction of the private 804 s payloads.
No new run was performed for this documentation. The local pair evidence hash
is recorded under `variants/VISIBLE/analysis/DROP2_PAIR.json` in the manifest.
