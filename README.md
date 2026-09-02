# ESP angular linearization LUT

This reusable component applies a cyclic, linearly interpolated correction to
a 14-bit absolute angle. A double runtime buffer lets the real-time reader run
without locks while a new table is installed from another core.

Two NVS slots protect the last valid calibration against an interrupted write.
Each table carries a format version, generation and IEEE CRC32. Uploads are
rejected when the CRC is wrong, the correction exceeds the configured bound,
or the corrected angular map would cease to be monotonic. New tables are
installed disabled and must be explicitly enabled after host-side validation.

The table is expressed in signed native sensor counts and spans one revolution.
Call `esp_angle_lut_apply()` after sensor direction/zero processing and before
angle unwrapping, speed estimation or control.
