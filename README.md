# ESP angular linearization LUT

This reusable component applies a cyclic, linearly interpolated correction to
an absolute angle. The native counts per revolution are selected with
`CONFIG_ESP_ANGLE_LUT_FULL_SCALE_COUNTS` (256 through 65536, power of two), so
the same component can serve 8- through 16-bit sensors. A double runtime buffer
lets the real-time reader run without locks while a new table is installed from
another core.

Two NVS slots protect the last valid calibration against an interrupted write.
Each table carries a format version, generation and IEEE CRC32. Uploads are
rejected when the CRC is wrong, the correction exceeds the configured bound,
or the corrected angular map would cease to be monotonic. New tables are
installed disabled and must be explicitly enabled after host-side validation.
`esp_angle_lut_install_detailed()` identifies the exact failed validation or
NVS persistence stage. Installation staging uses static internal storage so a
caller with a small task stack does not also carry the persistent blob there;
the original `esp_angle_lut_install()` API remains available.

The table is expressed in signed native sensor counts and spans one revolution.
The install API requires its source full scale and rejects a table generated
for a different resolution. Call `esp_angle_lut_apply()` after sensor
direction/zero processing and before angle unwrapping, speed estimation or
control.

The default is 16384 counts for compatibility with the MT6701 and stored
format-version-1 tables. Changing the configured resolution invalidates tables
from another scale and normally requires a new calibration.
