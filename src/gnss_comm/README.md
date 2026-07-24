# gnss_comm

`gnss_comm` is a compatibility-only ROS message package.

The message package name, message names, fields, field order, ROS datatypes and
MD5 sums are retained for existing nodes and bags. Runtime GNSS acquisition,
parsing, time conversion and coordinate conversion live in
`gnss_serial_driver`.

In particular, `/ublox_driver/receiver_pvt` remains
`gnss_comm/GnssPVTSolnMsg`.
