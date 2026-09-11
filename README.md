# ESP angular linearization LUT

`esp_angle_lut` corrects deterministic, position-dependent errors in a cyclic
absolute-angle measurement. It stores one signed correction table in NVS and
applies that table with cyclic linear interpolation in the real-time sampling
path.

The component is independent of the angle sensor and its electrical interface.
It does not communicate over I2C or SPI, command a motor, collect samples, or
calculate a calibration on the ESP32. Its input and output are simply native
angle counts representing one complete revolution.

This package also contains a reusable GNU Octave library that calculates,
validates, plots, serializes, and applies compatible correction tables.

## Component boundaries

| Responsibility | Provided here? | Owner |
|---|---:|---|
| Read an angle from a sensor | No | Sensor driver/application |
| Establish sensor direction and mechanical zero | No | Sensor driver/application |
| Collect raw angle versus time | No | Application or recorder component |
| Select stable calibration intervals | No | Application-specific host code |
| Calculate and validate the LUT | Yes | Octave library in `tools/octave` |
| Define a USB, UART, or network protocol | No | Application/transport |
| Validate and persist an uploaded LUT | Yes | ESP32 component |
| Apply the correction in the sampling path | Yes | ESP32 component |

The component can be used with any absolute encoder whose samples can be
represented as a power-of-two number of counts per revolution. Examples include
12-bit sensors with 4096 counts and 14-bit sensors with 16384 counts.

## End-to-end calibration flow

The surrounding application normally implements this workflow:

```text
absolute-angle sensor
        │
        ▼
raw native counts ──► capture raw angle and time
                              │
                              ▼
                    transfer capture to host
                              │
                              ▼
               select stable rotation intervals
                              │
                              ▼
                esp_angle_lut_calculate() in Octave
                              │
                  ┌───────────┴───────────┐
                  ▼                       ▼
            plots and metrics      int16 LUT + CRC32
                                          │
                                          ▼
                            application-defined transport
                                          │
                                          ▼
                          esp_angle_lut_install_detailed()
                                          │
                                   validate + NVS
                                          │
                                 read back and verify
                                          │
                                          ▼
                              esp_angle_lut_set_enabled(true)

Runtime:

sensor → direction/zero → esp_angle_lut_apply() → unwrap/Kalman → control
```

The time-series recorder and USB transport used by the example motor project
are convenient implementations of capture and transfer, but neither is a
dependency of `esp_angle_lut`. Another project may use a file, SD card, Wi-Fi,
BLE, UART, or a laboratory data-acquisition system instead.

## How the correction works

Let `N` be the configured number of sensor counts per revolution and `B` the
number of LUT bins. The table stores one signed correction `c[k]` at each
uniform node. The nominal distance between nodes is:

```text
bin_width = N / B
```

For a raw sample between nodes `k` and `k + 1`, the component calculates:

```text
fraction   = (raw % bin_width) / bin_width
correction = c[k] + fraction * (c[k + 1] - c[k])
corrected  = wrap(raw + correction, N)
```

The firmware uses integer arithmetic. Node selection and interpolation wrap
cyclically, so the successor of the final entry is the first entry.

If no valid table is loaded or correction is disabled,
`esp_angle_lut_apply()` returns the input normalized to one revolution. A
single runtime call site therefore works both before and after calibration.

## Build-time configuration

Open `idf.py menuconfig` and select **Angular linearization LUT**.

### `CONFIG_ESP_ANGLE_LUT_FULL_SCALE_COUNTS`

Number of distinct native angle counts in one revolution.

- accepted range: 256 through 65536;
- must be a power of two;
- must be at least the bin count;
- must be divisible by the bin count;
- default: 16384.

Typical values are:

| Sensor output | Value |
|---|---:|
| 8 bit | 256 |
| 10 bit | 1024 |
| 12 bit | 4096 |
| 14 bit | 16384 |
| 16 bit | 65536 |

Configure this value to match the scale supplied to
`esp_angle_lut_apply()`. Using the sensor's native resolution is normally best.
If the application rescales the sensor, the calibration data and every runtime
call must use the same scale.

Changing this option makes tables generated for another resolution
incompatible and normally requires a new physical calibration.

### `CONFIG_ESP_ANGLE_LUT_BIN_COUNT`

Number of uniformly spaced correction nodes around one revolution.

- accepted range: 16 through 256;
- must be a power of two;
- must divide `FULL_SCALE_COUNTS` exactly;
- default: 256.

More bins can represent finer spatial detail, but use more RAM, NVS, transfer
bandwidth, and calibration data. With 256 bins and 16384 counts per revolution,
the nodes are 64 counts, or 1.40625 degrees, apart.

### `CONFIG_ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS`

Maximum accepted absolute value of any signed correction entry.

- configured range: 1 through 32767 native counts;
- default: 1024 counts;
- the effective limit is additionally kept below half a revolution.

This is a safety and plausibility limit, not a filter parameter. When changing
sensor resolution, scale this value if the desired angular limit should remain
the same:

```text
limit_counts = limit_degrees * FULL_SCALE_COUNTS / 360
```

For example, 1024 counts at a 16384-count scale correspond to 22.5 degrees.

## Adding it to an ESP-IDF application

Place the component in the project's `components` directory or add it as a Git
submodule:

```text
project/
├── CMakeLists.txt
├── main/
└── components/
    └── esp_angle_lut/
        ├── CMakeLists.txt
        ├── Kconfig
        ├── esp_angle_lut.c
        └── include/esp_angle_lut.h
```

If an application component declares dependencies explicitly, add
`esp_angle_lut` and `nvs_flash` to its `REQUIRES` or `PRIV_REQUIRES` list:

```cmake
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_angle_lut nvs_flash
)
```

Include the public API with:

```c
#include "esp_angle_lut.h"
```

## Firmware lifecycle

The component is a singleton. It owns one Kconfig-selected configuration, one
persistent calibration, and one lock-free runtime view. There is no instance
handle.

```text
before init
    │ esp_angle_lut_init()
    ▼
initialized, no table        initialized, restored table
loaded=false                 loaded=true
enabled=false                enabled=stored NVS value
    │
    │ esp_angle_lut_install[_detailed]()
    ▼
new table loaded and persisted
loaded=true, enabled=false
    │ esp_angle_lut_set_enabled(true)
    ▼
correction active
loaded=true, enabled=true
    │ esp_angle_lut_set_enabled(false)
    ▼
table retained but bypassed
loaded=true, enabled=false
    │ esp_angle_lut_clear()
    ▼
table erased
loaded=false, enabled=false
```

A newly installed table is deliberately disabled. The application should read
it back, compare it with the source, and only then enable it.

## Initializing the component

NVS flash must be initialized once by the application before
`esp_angle_lut_init()` is called:

```c
#include "esp_angle_lut.h"
#include "nvs_flash.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_angle_lut_init());

    esp_angle_lut_status_t status;
    esp_angle_lut_get_status(&status);

    if (status.loaded && status.enabled) {
        /* A previously verified calibration was restored from NVS. */
    }

    /* Initialize the sensor and start the application tasks here. */
}
```

Call `esp_angle_lut_init()` exactly once. A second call returns
`ESP_ERR_INVALID_STATE`. Initialization succeeds even if no valid table exists;
in that case, `loaded` and `enabled` remain false and runtime application is a
bypass.

The application owns global NVS recovery policy. Handle an NVS partition or
version error according to the wider product's data-retention requirements
before initializing this component.

## Integrating a sensor driver

The preferred integration is for the driver to return one unsigned native
count in `[0, N - 1]`, where `N` equals
`ESP_ANGLE_LUT_FULL_SCALE_COUNTS`.

### Sensor already uses the configured resolution

For a 14-bit sensor and a 16384-count configuration:

```c
uint16_t raw_counts = 0;
ESP_ERROR_CHECK(my_sensor_get_angle_counts(&sensor, &raw_counts));

uint16_t corrected_counts = esp_angle_lut_apply(raw_counts);
float corrected_degrees =
    (float)corrected_counts * 360.0f /
    (float)ESP_ANGLE_LUT_FULL_SCALE_COUNTS;
```

The component does not know whether `my_sensor_get_angle_counts()` used SPI,
I2C, PWM, or another interface.

### Sensor uses a different count scale

Prefer changing `CONFIG_ESP_ANGLE_LUT_FULL_SCALE_COUNTS` to the native scale.
If the application must retain another LUT scale, use an explicit, consistent
conversion:

```c
#define SENSOR_COUNTS_PER_REVOLUTION 4096U

static uint16_t sensor_to_lut_counts(uint16_t sensor_counts)
{
    uint32_t scaled =
        ((uint32_t)sensor_counts * ESP_ANGLE_LUT_FULL_SCALE_COUNTS) /
        SENSOR_COUNTS_PER_REVOLUTION;
    return (uint16_t)(scaled &
                      (ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1U));
}
```

The raw data used to calculate the LUT must have the same angular convention
and resolution as the values supplied at runtime.

### Direction and zero convention

Apply direction reversal and mechanical-zero transformation before the LUT,
using the same convention during calibration and normal operation:

```text
sensor native count
        ↓
direction convention
        ↓
mechanical-zero offset
        ↓
esp_angle_lut_apply()
        ↓
angle unwrapping / estimator / controller
```

Do not change direction, zero, resolution, magnet position, or sensor mounting
after calibration without regenerating or appropriately transforming the LUT.
The table represents the complete physical and numerical measurement chain
used during its experiment.

## Calling the correction in a real-time loop

Only `esp_angle_lut_apply()` belongs in the high-rate sampling path:

```c
void control_sample(void)
{
    uint16_t raw_counts;
    if (my_sensor_get_angle_counts(&sensor, &raw_counts) != ESP_OK) {
        return; /* Preserve the previous estimator state on a sensor error. */
    }

    uint16_t logical_counts = apply_direction_and_zero(raw_counts);
    uint16_t corrected_counts = esp_angle_lut_apply(logical_counts);
    float corrected_degrees =
        (float)corrected_counts * 360.0f /
        (float)ESP_ANGLE_LUT_FULL_SCALE_COUNTS;

    kalman_update(&filter, corrected_degrees);
}
```

`esp_angle_lut_apply()`:

- performs no allocation;
- performs no NVS access;
- performs no logging;
- takes no lock;
- uses bounded integer arithmetic;
- supports concurrent publication of a new table;
- returns a value in `[0, FULL_SCALE_COUNTS - 1]`.

The function is suitable for a deterministic task, but is not declared
`IRAM_ATTR`. An application that must execute while the flash cache is disabled
needs an additional IRAM-safety review of this component and its complete call
chain.

Do not call installation, enable/disable, readback, clear, or other NVS-related
functions from the deterministic loop. Serialize those management operations
in one lower-priority application or communication task.

## Collecting calibration data

The ESP32 component does not calculate a LUT. The host algorithm requires:

- a strictly increasing time vector in seconds;
- the corresponding raw cyclic angle in degrees;
- one or more index vectors selecting stable, approximately constant-speed
  rotation intervals;
- enough complete revolutions to cover the whole angular range.

Capture the angle before `esp_angle_lut_apply()`. If an old table is active,
record a separate pre-LUT channel or disable correction during the experiment.
Recording only an already corrected angle causes the new table to model the
previous correction instead of the physical raw error.

A typical acquisition path is:

```text
sampling task
    ├── timestamp
    ├── raw angle before LUT
    └── actuator command or another plateau marker
             │
             ▼
       time-series buffer
             │
             ▼
       host-side transfer
```

Open-loop constant-duty plateaus are useful because feedback would react to
angle-induced speed ripple and mix controller behavior into the sensor
experiment. The reusable calculator does not require a particular duty cycle,
sample rate, motor, or transport; it only requires stable segment indices.

Avoid acceleration and deceleration transients. Select settled portions with
several complete revolutions. More revolutions improve angular coverage and
make the per-bin median more robust.

## Calculating a LUT in GNU Octave

Add the reusable library to the Octave path:

```octave
addpath ("components/esp_angle_lut/tools/octave");
```

Assume the acquisition produced column vectors `time_s` and `raw_angle_deg`.
The application has identified three stable intervals:

```octave
segment_1 = find (time_s >= 2.0  & time_s <= 7.5);
segment_2 = find (time_s >= 10.0 & time_s <= 15.5);
segment_3 = find (time_s >= 18.0 & time_s <= 23.5);
segments = {segment_1, segment_2, segment_3};
```

Use the same configuration values as the target firmware:

```octave
bin_count = 256;
full_scale_counts = 16384;
max_abs_correction_counts = 1024;

calibration = esp_angle_lut_calculate ( ...
    time_s, raw_angle_deg, segments, bin_count, ...
    full_scale_counts, max_abs_correction_counts);
```

Inspect and save the result before installing it:

```octave
esp_angle_lut_plot (calibration, 12);

fprintf ("Angular RMS: %.4f -> %.4f deg (%.1f%% reduction)\n", ...
         calibration.raw_rms_deg, calibration.corrected_rms_deg, ...
         calibration.reduction_percent);

fprintf ("Speed-ripple RMS: %.2f -> %.2f rpm (%.1f%% reduction)\n", ...
         calibration.raw_speed_rms_rpm, ...
         calibration.corrected_speed_rms_rpm, ...
         calibration.speed_reduction_percent);

save ("-mat7-binary", "angle_calibration.mat", "calibration");
```

Important result fields are:

| Field | Meaning |
|---|---|
| `correction_counts` | Signed `int16`-compatible LUT entries |
| `correction_deg` | Same table converted to degrees |
| `lut_angle_deg` | Raw-angle node associated with each entry |
| `bin_count` | Number of correction entries |
| `full_scale_counts` | Target counts per revolution |
| `max_abs_correction_counts` | Safety bound used during calculation |
| `payload` | Little-endian bytes ready for transport |
| `payload_crc32` | CRC-32/IEEE of `payload` |
| `selected_harmonics` | Harmonic bandwidth retained by the model |
| `revolution_count` | Number of complete revolutions processed |
| `raw_rms_deg` | Withheld angular-error RMS before correction |
| `corrected_rms_deg` | Withheld angular-error RMS after correction |
| `reduction_percent` | Angular RMS reduction |
| `raw_speed_rms_rpm` | Detrended instantaneous-speed RMS before correction |
| `corrected_speed_rms_rpm` | Same metric after correction |
| `speed_reduction_percent` | Speed-ripple RMS reduction |

### What the calculator does

`esp_angle_lut_calculate()`:

1. unwraps each stable cyclic-angle segment;
2. infers rotation direction;
3. interpolates complete-revolution crossing times;
4. constructs uniform ideal angular progress inside each revolution;
5. calculates `ideal angle - measured angle` versus raw phase;
6. uses alternating revolutions for training and withheld validation;
7. calculates the median training error in each angular bin;
8. fills missing bins with circular interpolation;
9. retains initially up to eight low spatial harmonics using FFT/IFFT;
10. removes mean correction so the LUT does not redefine zero;
11. converts bin-center estimates to firmware's boundary nodes;
12. quantizes corrections to native signed counts;
13. reduces retained harmonics until the corrected map is monotonic;
14. enforces the target firmware's correction limit;
15. evaluates angular error and detrended speed ripple on withheld turns;
16. serializes the table and calculates its CRC32.

The function requires at least `4 * bin_count` training samples and
`2 * bin_count` validation samples after complete-revolution extraction.
Intervals without at least two complete usable revolutions are ignored.

## Host-side validation and offline application

Validate an existing correction vector with firmware-equivalent rules:

```octave
[valid, report] = esp_angle_lut_validate ( ...
    calibration.correction_counts, ...
    calibration.full_scale_counts, ...
    calibration.max_abs_correction_counts);

if (! valid)
  error ("LUT is unsafe or incompatible");
endif

fprintf ("Maximum correction: %.0f counts\n", ...
         report.maximum_correction_counts);
fprintf ("Corrected step range: %.0f to %.0f counts\n", ...
         report.minimum_corrected_step_counts, ...
         report.maximum_corrected_step_counts);
```

Apply it to captured angles without connecting an ESP32:

```octave
corrected_angle_deg = esp_angle_lut_apply ( ...
    raw_angle_deg, calibration.correction_counts, ...
    calibration.full_scale_counts);
```

Recreate the exact binary payload and CRC:

```octave
[payload, crc] = esp_angle_lut_pack (calibration.correction_counts);
assert (crc == calibration.payload_crc32);
```

`payload` contains two bytes per entry, low byte first. Negative values use
two's-complement `int16` representation. The CRC is reflected CRC-32/IEEE with
polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, and final XOR
`0xFFFFFFFF`; it matches `esp_crc32_le()` on the ESP32.

## Transferring the LUT

This component intentionally does not define a serial protocol. A transport
must send at least:

- `bin_count`;
- `full_scale_counts`;
- the little-endian signed-correction payload;
- `payload_crc32`.

For 256 bins, the payload contains 512 bytes. Use `esp_angle_lut_pack()` or
`esp_angle_lut_int16_le_bytes()` to obtain the defined byte order rather than
transmitting an Octave numeric array as text.

The receiver should copy the complete payload into stable memory and call the
installation API from a non-real-time task. The caller retains ownership of its
input array; the installation function copies it before returning.

## Installing a table directly in C

This example assumes `received_lut` contains the signed entries produced by the
host, `expected_crc` was received independently, and initialization is complete:

```c
#include "esp_angle_lut.h"
#include "esp_log.h"

static esp_err_t install_received_lut(
    const int16_t received_lut[ESP_ANGLE_LUT_BIN_COUNT],
    uint32_t expected_crc)
{
    esp_angle_lut_install_failure_t failure =
        ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;

    esp_err_t error = esp_angle_lut_install_detailed(
        received_lut,
        ESP_ANGLE_LUT_BIN_COUNT,
        ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
        expected_crc,
        &failure);

    if (error != ESP_OK) {
        ESP_LOGE("APP", "LUT installation failed: %s",
                 esp_angle_lut_install_failure_name(failure));
        return error;
    }

    return ESP_OK; /* Loaded and persisted, but intentionally disabled. */
}
```

Use `esp_angle_lut_install()` when a simple `esp_err_t` is sufficient. Use the
detailed version when a protocol or user interface should report the exact
failed stage.

## Readback, verification, and enablement

Read the active table back before enabling it:

```c
#include "esp_check.h"
#include <string.h>

static int16_t readback[ESP_ANGLE_LUT_BIN_COUNT];

static esp_err_t verify_and_enable(
    const int16_t expected_lut[ESP_ANGLE_LUT_BIN_COUNT],
    uint32_t expected_crc)
{
    esp_angle_lut_status_t status;
    ESP_RETURN_ON_ERROR(
        esp_angle_lut_read(readback, ESP_ANGLE_LUT_BIN_COUNT, &status),
        "APP", "Could not read the installed LUT");

    if (status.payload_crc32 != expected_crc ||
        memcmp(readback, expected_lut, ESP_ANGLE_LUT_PAYLOAD_BYTES) != 0) {
        return ESP_ERR_INVALID_CRC;
    }

    return esp_angle_lut_set_enabled(true);
}
```

`esp_angle_lut_set_enabled(true)` fails if no valid table is loaded. Enable and
disable operations are persisted. Disabling bypasses correction but retains the
table:

```c
ESP_ERROR_CHECK(esp_angle_lut_set_enabled(false));
```

To disable correction and erase both persistent copies:

```c
ESP_ERROR_CHECK(esp_angle_lut_clear());
```

`esp_angle_lut_clear()` is destructive for the calibration. A new upload or
experiment is required before correction can be enabled again.

## Table validation rules

Firmware validates every upload before modifying NVS.

### Metadata compatibility

`bin_count` and `full_scale_counts` must exactly match the target firmware's
compiled values. A table generated for a 12-bit target is not silently scaled
for a 14-bit target.

### Payload integrity

The IEEE CRC32 over the exact little-endian correction bytes must match the
expected CRC supplied to the installation function.

### Correction amplitude

Every entry must satisfy:

```text
-ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS
    <= correction[k] <=
 ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS
```

### Corrected-map monotonicity

For each bin, including the cyclic last-to-first transition:

```text
corrected_step = bin_width + correction[next] - correction[current]
```

The accepted interval is:

```text
1 <= corrected_step <= 4 * bin_width
```

A nonpositive step would collapse or reverse part of the angular map. The upper
bound rejects an implausibly abrupt local expansion.

## Installation failure details

`esp_angle_lut_install_detailed()` optionally reports:

| Enum suffix | Stable string | Meaning |
|---|---|---|
| `_NONE` | `none` | No failure was recorded |
| `_NOT_INITIALIZED` | `not_initialized` | Initialization was not called |
| `_METADATA` | `invalid_metadata` | Pointer, bin count, or scale is incompatible |
| `_CRC` | `crc_mismatch` | Payload CRC does not match |
| `_CORRECTION_RANGE` | `correction_out_of_range` | An entry exceeds the configured limit |
| `_NON_MONOTONIC` | `non_monotonic_lut` | A corrected angular step is invalid |
| `_NVS_OPEN` | `nvs_open_failed` | The NVS namespace could not be opened |
| `_NVS_BLOB_WRITE` | `nvs_blob_write_failed` | The replacement blob could not be written |
| `_NVS_ACTIVE_WRITE` | `nvs_active_write_failed` | The selected-slot key could not be written |
| `_NVS_ENABLED_WRITE` | `nvs_enabled_write_failed` | The disabled state could not be written |
| `_NVS_COMMIT` | `nvs_commit_failed` | The NVS transaction could not be committed |

Use `esp_angle_lut_install_failure_name()` instead of duplicating this mapping
in a transport. Its result is a static string and must not be freed.

## Status fields

`esp_angle_lut_get_status()` provides a nonblocking snapshot:

```c
esp_angle_lut_status_t status;
esp_angle_lut_get_status(&status);
```

| Field | Meaning |
|---|---|
| `loaded` | A valid table is present in the runtime buffers |
| `enabled` | `esp_angle_lut_apply()` currently applies the table |
| `format_version` | Persistent/wire metadata version, currently 2 |
| `bin_count` | Compiled correction-entry count |
| `full_scale_counts` | Compiled counts per revolution |
| `max_abs_correction_counts` | Effective accepted amplitude limit |
| `generation` | Successful-installation generation of the active table |
| `payload_crc32` | CRC32 of the active signed-correction payload |

Passing `NULL` to `esp_angle_lut_get_status()` is allowed and has no effect.

## Persistence and interrupted-write recovery

The NVS namespace is `angle_lut`. Two blob slots, `slot0` and `slot1`, protect
the last valid table:

1. installation validates all input before opening NVS;
2. the replacement is written to the inactive slot;
3. the active-slot key changes only after the complete blob write succeeds;
4. the new table's enabled state is stored as false;
5. all changes are committed;
6. only after a successful commit is the new table published to RAM.

At boot, both slots are checked. The selected valid slot is preferred, but the
other valid slot is a fallback after an interrupted or corrupt write. Each blob
includes a magic number, format version, bin count, full scale, generation,
payload CRC, and correction entries.

Current format version 2 stores sensor full scale. Version-1 tables can be read
only at their historical 16384-count resolution and a compatible bin count.
Other incompatible stored blobs are ignored.

## Concurrency, ownership, and memory

The component owns all runtime and persistent copies. Installation input and
readback output belong to the caller and may be reused after the call returns.

Two runtime tables allow atomic publication: the writer fills the inactive
buffer and then publishes its pointer. `esp_angle_lut_apply()` acquires a
complete immutable table without taking a lock.

Use one serialized management context for installation, readback,
enable/disable, and clear operations. The supported concurrent pattern is a
high-rate reader calling `esp_angle_lut_apply()` while one lower-priority
management context publishes or changes a table. Do not launch simultaneous
installation or clear operations from multiple tasks.

Approximate static correction storage, excluding NVS library overhead, is:

```text
two runtime tables    = 2 * BIN_COUNT * sizeof(int16_t)
installation staging = blob metadata + BIN_COUNT * sizeof(int16_t)
```

At 256 bins, the correction arrays occupy 1536 bytes: 1024 bytes for the two
runtime tables and 512 bytes in installation staging.

## Function-by-function calling guide

### `esp_angle_lut_init()`

```c
esp_err_t error = esp_angle_lut_init();
```

Call once during application startup, after `nvs_flash_init()`. It initializes
the atomic runtime state, reads both NVS slots, publishes the preferred valid
table or its fallback, and restores persisted enablement. It returns `ESP_OK`
even when no table is stored. It can return an NVS open error, and returns
`ESP_ERR_INVALID_STATE` when called more than once.

### `esp_angle_lut_apply()`

```c
uint16_t corrected = esp_angle_lut_apply(raw_counts);
```

Call once for each valid sensor sample, after direction/zero processing and
before unwrapping or estimation. The argument is interpreted in the compiled
full-scale convention. The return value is always normalized to one revolution.
There is no error return: an unloaded or disabled LUT is a defined bypass.

### `esp_angle_lut_install()`

```c
esp_err_t error = esp_angle_lut_install(
    corrections, bin_count, full_scale_counts, payload_crc32);
```

Call from a serialized management or transport task. `corrections` must point
to exactly `bin_count` signed entries, and the metadata must match Kconfig. The
function validates, copies, persists, and publishes the table, leaving it
disabled. It returns `ESP_ERR_INVALID_STATE` before initialization,
`ESP_ERR_INVALID_ARG` for metadata/range/monotonicity failure,
`ESP_ERR_INVALID_CRC` for integrity failure, or an NVS error. Use the detailed
variant when those invalid-argument cases must be distinguished.

### `esp_angle_lut_install_detailed()`

```c
esp_angle_lut_install_failure_t failure;
esp_err_t error = esp_angle_lut_install_detailed(
    corrections, bin_count, full_scale_counts, payload_crc32, &failure);
```

This is the preferred entry point for communication protocols. It performs the
same operation as `esp_angle_lut_install()` and writes the exact failed stage to
`failure`. The final argument may be `NULL` when detail is unnecessary. On
success, it writes `ESP_ANGLE_LUT_INSTALL_FAILURE_NONE`.

### `esp_angle_lut_install_failure_name()`

```c
const char *reason = esp_angle_lut_install_failure_name(failure);
```

Use this utility in logs and machine-readable responses. It always returns a
valid null-terminated static string, including for an unknown enum value. Do
not modify or free the returned pointer.

### `esp_angle_lut_read()`

```c
int16_t corrections[ESP_ANGLE_LUT_BIN_COUNT];
esp_angle_lut_status_t status;
esp_err_t error = esp_angle_lut_read(
    corrections, ESP_ANGLE_LUT_BIN_COUNT, &status);
```

Use for readback, diagnostics, or exporting the installed calibration. The
destination must hold exactly the configured number of entries. `status` is
optional and may be `NULL`. The function returns `ESP_ERR_INVALID_STATE` if no
table is loaded, the destination is null, the supplied count is different, or
the active runtime pointer is unavailable. Do not race this management copy
with another installation or clear operation.

### `esp_angle_lut_set_enabled()`

```c
ESP_ERROR_CHECK(esp_angle_lut_set_enabled(true));  /* Apply the LUT. */
ESP_ERROR_CHECK(esp_angle_lut_set_enabled(false)); /* Bypass, retain LUT. */
```

Call from a management task. The state is committed to NVS before it is exposed
to real-time readers. Enabling returns `ESP_ERR_INVALID_STATE` before
initialization or without a loaded table. Disabling is allowed after
initialization even when no table is loaded. NVS failures are returned without
publishing the requested new state.

### `esp_angle_lut_clear()`

```c
esp_err_t error = esp_angle_lut_clear();
```

Call only when the stored calibration should be erased. Correction is bypassed
immediately; both table slots and their control keys are erased and committed.
Runtime loaded metadata is cleared after the persistent erase succeeds. The
function returns `ESP_ERR_INVALID_STATE` before initialization or the NVS
open/commit error. Missing individual keys are tolerated.

### `esp_angle_lut_get_status()`

```c
esp_angle_lut_status_t status;
esp_angle_lut_get_status(&status);
```

Use for diagnostics and protocol status responses. It copies compile-time
metadata and atomic runtime fields without allocation or NVS access. The
function has no error return. Passing `NULL` is an intentional no-op.

## API summary

| Function | Where to call it | Result |
|---|---|---|
| `esp_angle_lut_init()` | Application initialization | Initializes state and restores the newest valid NVS table |
| `esp_angle_lut_apply()` | High-rate sampling path | Returns corrected or bypassed normalized counts |
| `esp_angle_lut_install()` | Management/transport task | Validates and persists a table; leaves it disabled |
| `esp_angle_lut_install_detailed()` | Management/transport task | Same operation with an exact failure stage |
| `esp_angle_lut_install_failure_name()` | Diagnostics/protocol code | Returns a stable static failure string |
| `esp_angle_lut_read()` | Management/verification task | Copies the loaded table and optional status |
| `esp_angle_lut_set_enabled()` | Management task | Persistently enables or bypasses correction |
| `esp_angle_lut_clear()` | Management task | Disables and erases the calibration |
| `esp_angle_lut_get_status()` | Application or diagnostics | Copies a lock-free status snapshot |

## Recommended integration sequence

1. Select full scale, bin count, and correction limit in Kconfig.
2. Initialize NVS and call `esp_angle_lut_init()` once.
3. Place `esp_angle_lut_apply()` between sensor normalization and estimation.
4. Capture timestamped raw pre-LUT angles during stable rotations.
5. Select stable intervals in application-specific host code.
6. Call `esp_angle_lut_calculate()` with the firmware configuration values.
7. Inspect the plot and validation metrics.
8. Pack and transport the signed table, metadata, and CRC.
9. Install it with `esp_angle_lut_install_detailed()`.
10. Read back and compare the complete table and CRC.
11. Enable correction with `esp_angle_lut_set_enabled(true)`.
12. Repeat the experiment to verify improvement in the physical system.

## Reusable Octave function reference

| Function | Purpose |
|---|---|
| `esp_angle_lut_calculate()` | Build and cross-validate a LUT from time, raw angle, and stable segments |
| `esp_angle_lut_apply()` | Apply firmware-equivalent cyclic interpolation offline |
| `esp_angle_lut_validate()` | Check representation, correction limit, and monotonicity |
| `esp_angle_lut_pack()` | Serialize corrections and calculate their CRC |
| `esp_angle_lut_int16_le_bytes()` | Encode corrections in little-endian order |
| `esp_angle_lut_crc32_ieee()` | Calculate firmware-compatible CRC-32/IEEE |
| `esp_angle_lut_plot()` | Plot the table and withheld before/after validation |

Detailed documentation for every Octave function and internal helper is in
[`tools/octave/README.md`](tools/octave/README.md).

## Testing the host library

Run the sensor-independent regression test:

```text
octave --quiet components/esp_angle_lut/tools/octave/test_esp_angle_lut.m
```

The test verifies the canonical CRC vector, signed little-endian serialization,
calculation and correction at 14- and 12-bit resolutions, and rejection of a
nonmonotonic table. It uses synthetic data and requires no serial port, ESP32,
sensor, `instrument-control`, or surrounding motor-control application.

## Troubleshooting

### `esp_angle_lut_apply()` does not change the angle

Check `status.loaded` and `status.enabled`. Installation intentionally leaves a
new table disabled. If `loaded` is false after boot, the stored table may be
absent, corrupt, or incompatible with the compiled configuration.

### Installation reports `invalid_metadata`

The received bin count or full scale differs from Kconfig, or the correction
pointer is null. Generate the LUT with the exact target configuration.

### Installation reports `crc_mismatch`

Calculate CRC over the exact little-endian payload bytes, not over text,
Octave doubles, or a host-native representation with unspecified byte order.

### Installation reports `correction_out_of_range`

The experiment produced a correction larger than the firmware safety limit.
Inspect mounting, magnet alignment, stable-segment selection, and host
configuration before increasing the limit.

### Installation reports `non_monotonic_lut`

An adjacent correction difference would reverse, collapse, or expand the local
map excessively. Recalculate from better data or use stronger harmonic
smoothing; do not bypass this check.

### Corrected angle is worse than raw angle

Verify that calibration and runtime use the same sensor, mounting, rotation
direction, mechanical zero, count scale, and pre-LUT raw signal. Confirm that
selected intervals exclude speed transients.

### Calibration has too few samples or observed bins

Capture more complete revolutions, use longer settled intervals, or reduce bin
count. Ensure the angle repeatedly covers the complete revolution.

### A table disappeared after changing Kconfig

Tables are tied to full-scale and bin-count metadata. Changing either normally
requires a new calibration.
