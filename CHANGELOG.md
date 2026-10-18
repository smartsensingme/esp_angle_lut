# Change log

## 2026-10-18 — Independent LUT instances (unreleased)

Changes since ff8f51fad4dd9be596905d0ddb19114f2caf74f3:

- Add owned `esp_angle_lut_handle_t` instances and create/destroy APIs, with
  independent NVS namespaces, tables, enablement, generation and CRC. Instance
  apply/install/read/enable/clear/status APIs mirror the legacy functionality.
- Allocate handle state, two correction tables and installation staging in
  internal-capability RAM; sampling remains bounded integer-only, without
  allocation, I/O or locks. Namespace names are copied; duplicate live names
  and the reserved legacy namespace are rejected.
- Durably disable correction before installing a replacement table. Interrupted
  writes may retain either validated slot; an installation must not implicitly
  enable a replacement. Propagate unexpected key-erase failures during clear.
- Add native tests with NVS stubs for independent instances, interpolation/wrap,
  invalid input, persistence/reload, failure injection, fallback and legacy use.
  The runner selects the macOS SDK when needed and uses a temporary binary.

Compatibility: legacy function signatures and `angle_lut` namespace remain;
version-2 blobs and compatible version-1 loading remain supported. Existing
namespaces must continue to identify the same physical sensors. A failed install
can leave the old table disabled; read back/verify before explicit enablement.

Concurrency contract: serialize all lifecycle/management calls and stop sampling
readers during install, enable/disable, clear and destroy. Atomic pointer
publication is not a reclamation protocol for repeated buffer reuse. This
maintenance requirement also applies to legacy calls; it supersedes earlier
claims of unrestricted concurrent publication.

Validation: native ASan/UBSan regressions and ESP-IDF 6.0 component integration
builds. Software fault injection does not qualify real NVS power cuts. An IRAM
apply routine and internal buffers do not guarantee that an application's task
meets deadlines while flash/cache operations suspend other execution.
