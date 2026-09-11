#ifndef ESP_ANGLE_LUT_H_
#define ESP_ANGLE_LUT_H_

#include "esp_err.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ANGLE_LUT_FULL_SCALE_COUNTS CONFIG_ESP_ANGLE_LUT_FULL_SCALE_COUNTS
#define ESP_ANGLE_LUT_FORMAT_VERSION 2U
#define ESP_ANGLE_LUT_BIN_COUNT CONFIG_ESP_ANGLE_LUT_BIN_COUNT
#define ESP_ANGLE_LUT_PAYLOAD_BYTES (ESP_ANGLE_LUT_BIN_COUNT * sizeof(int16_t))
#define ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS                                \
  ((int32_t)((CONFIG_ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS <                 \
              (ESP_ANGLE_LUT_FULL_SCALE_COUNTS / 2U))                          \
                 ? CONFIG_ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS              \
                 : ((ESP_ANGLE_LUT_FULL_SCALE_COUNTS / 2U) - 1U)))

/** Runtime and persistent-table metadata returned to the application. */
typedef struct {
  /** True when a valid correction table is present in RAM. */
  bool loaded;
  /** True when esp_angle_lut_apply() is currently applying the table. */
  bool enabled;
  /** Persistent blob and host-protocol format version. */
  uint16_t format_version;
  /** Number of signed correction entries in the table. */
  uint16_t bin_count;
  /** Native sensor counts in one complete revolution. */
  uint32_t full_scale_counts;
  /** Largest accepted absolute correction, in native counts. */
  uint32_t max_abs_correction_counts;
  /** Monotonically increasing successful-installation generation. */
  uint32_t generation;
  /** IEEE CRC32 of the active signed-correction byte payload. */
  uint32_t payload_crc32;
} esp_angle_lut_status_t;

/** Detailed validation or NVS stage reported by table installation. */
typedef enum {
  ESP_ANGLE_LUT_INSTALL_FAILURE_NONE = 0,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NOT_INITIALIZED,
  ESP_ANGLE_LUT_INSTALL_FAILURE_METADATA,
  ESP_ANGLE_LUT_INSTALL_FAILURE_CRC,
  ESP_ANGLE_LUT_INSTALL_FAILURE_CORRECTION_RANGE,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NON_MONOTONIC,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_BLOB_WRITE,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ACTIVE_WRITE,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE,
  ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT,
} esp_angle_lut_install_failure_t;

/**
 * @brief Initialize the component and load the newest valid table from NVS.
 *
 * The application must initialize NVS flash before calling this function. The
 * function initializes the lock-free runtime state, reads both persistent
 * slots, activates the preferred valid table or its fallback, and restores the
 * persisted enable state. A valid table is not required for initialization to
 * succeed.
 *
 * This is a public entry point intended to be called once by application code.
 * It is not called by another function inside this component. Internally it
 * calls `read_slot()` for both NVS slots, `activate_blob()` for the selected
 * table, and esp_angle_lut_get_status() for the initialization log.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized, or
 *         an NVS error if the namespace cannot be opened.
 */
esp_err_t esp_angle_lut_init(void);

/**
 * @brief Apply the active cyclic correction table to one native angle sample.
 *
 * The input is normalized to one revolution, the two surrounding LUT bins are
 * linearly interpolated, and the signed correction is added with cyclic
 * wraparound. If correction is disabled or no table is active, the normalized
 * input is returned unchanged. The function performs no NVS access, allocation
 * or locking and is suitable for a real-time sampling path.
 *
 * This is a public entry point called only by application code; no function
 * inside this component calls it.
 *
 * @param[in] angle_counts Raw absolute angle in native sensor counts.
 * @return Corrected angle in the range
 *         `[0, ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1]`.
 */
uint16_t esp_angle_lut_apply(uint16_t angle_counts);

/**
 * @brief Validate and persist a new table without detailed failure reporting.
 *
 * This convenience API delegates the complete operation to
 * esp_angle_lut_install_detailed() with a null failure-detail pointer. A newly
 * installed table is loaded but disabled until esp_angle_lut_set_enabled(true)
 * is called.
 *
 * This public function is intended for application code and is not called by
 * another function inside this component. It internally calls
 * esp_angle_lut_install_detailed().
 *
 * @param[in] corrections Signed corrections in native sensor counts.
 * @param[in] bin_count Number of entries; must equal ESP_ANGLE_LUT_BIN_COUNT.
 * @param[in] full_scale_counts Sensor counts per revolution; must match the
 *            compiled configuration.
 * @param[in] payload_crc32 IEEE CRC32 over the little-endian correction bytes.
 * @return ESP_OK on success or the validation/NVS error from the detailed API.
 */
esp_err_t esp_angle_lut_install(const int16_t *corrections, size_t bin_count,
                                uint32_t full_scale_counts,
                                uint32_t payload_crc32);

/**
 * @brief Validate, persist and stage a table with detailed failure reporting.
 *
 * The table metadata, payload CRC, correction range and corrected-map
 * monotonicity are validated before any NVS write. The new blob is written to
 * the inactive slot, made the preferred slot, persisted as disabled, committed
 * atomically by NVS, and then published to the lock-free runtime buffer.
 *
 * This is a public entry point. It is called internally only by
 * esp_angle_lut_install(); the USB transport may also call it directly.
 * Internally it calls `validate_table()`, `activate_blob()`, and
 * esp_angle_lut_install_failure_name() when logging an NVS failure.
 *
 * @param[in] corrections Signed corrections in native sensor counts.
 * @param[in] bin_count Number of entries; must equal ESP_ANGLE_LUT_BIN_COUNT.
 * @param[in] full_scale_counts Sensor counts per revolution; must match the
 *            compiled configuration.
 * @param[in] payload_crc32 IEEE CRC32 over the little-endian correction bytes.
 * @param[out] failure Optional destination for the exact failed stage; may be
 *             null when only the esp_err_t result is required.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE before initialization,
 *         ESP_ERR_INVALID_ARG/ESP_ERR_INVALID_CRC for invalid data, or an NVS
 *         persistence error.
 */
esp_err_t esp_angle_lut_install_detailed(
    const int16_t *corrections, size_t bin_count, uint32_t full_scale_counts,
    uint32_t payload_crc32, esp_angle_lut_install_failure_t *failure);

/**
 * @brief Convert an installation-failure enum to its stable protocol name.
 *
 * This public utility performs no allocation and always returns a valid static
 * string, including for unknown enum values. Inside the component it is called
 * only by esp_angle_lut_install_detailed(); external transports also use it to
 * report machine-readable failures.
 *
 * @param[in] failure Failure stage to translate.
 * @return Pointer to an immutable, null-terminated static string.
 */
const char *
esp_angle_lut_install_failure_name(esp_angle_lut_install_failure_t failure);

/**
 * @brief Copy the active table and optionally its status metadata.
 *
 * The correction payload is copied from the currently published runtime table.
 * Passing a null status pointer requests only the corrections. The caller must
 * provide storage for ESP_ANGLE_LUT_BIN_COUNT signed entries.
 *
 * This public function is called only by application or transport code; it is
 * not called by another function inside this component. It internally calls
 * esp_angle_lut_get_status() when @p status is not null.
 *
 * @param[out] corrections Destination correction array.
 * @param[in] bin_count Capacity in entries; must equal the configured count.
 * @param[out] status Optional destination for a lock-free status snapshot.
 * @return ESP_OK on success or ESP_ERR_INVALID_STATE if no table is loaded or
 *         an argument does not satisfy the required state/capacity.
 */
esp_err_t esp_angle_lut_read(int16_t *corrections, size_t bin_count,
                             esp_angle_lut_status_t *status);

/**
 * @brief Persistently enable or disable angle correction.
 *
 * The requested state is committed to NVS before it is published to the
 * real-time reader. Enabling requires a loaded table; disabling is allowed
 * without one after initialization.
 *
 * This is a public entry point called only by application or transport code;
 * no function inside this component calls it.
 *
 * @param[in] enabled True to apply the active table, false to bypass it.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE before initialization or
 *         when enabling without a table, or an NVS error.
 */
esp_err_t esp_angle_lut_set_enabled(bool enabled);

/**
 * @brief Disable correction and erase both persistent table slots.
 *
 * Correction is disabled in RAM before erasing NVS. After a successful commit,
 * all runtime metadata and the active pointer are cleared. Individual missing
 * NVS keys are tolerated.
 *
 * This is a public entry point called only by application or transport code;
 * no function inside this component calls it.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE before initialization, or
 *         an NVS open/commit error.
 */
esp_err_t esp_angle_lut_clear(void);

/**
 * @brief Copy a lock-free snapshot of the component status.
 *
 * Compile-time metadata and atomically published runtime fields are copied into
 * the caller's structure. A null destination is silently ignored.
 *
 * This public function is called internally by esp_angle_lut_init() and by
 * esp_angle_lut_read() when status was requested. Application and transport
 * code may also call it directly.
 *
 * @param[out] status Destination status structure; may be null.
 */
void esp_angle_lut_get_status(esp_angle_lut_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ESP_ANGLE_LUT_H_ */
