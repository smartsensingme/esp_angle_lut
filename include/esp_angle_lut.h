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

#define ESP_ANGLE_LUT_SENSOR_COUNTS 16384U
#define ESP_ANGLE_LUT_FORMAT_VERSION 1U
#define ESP_ANGLE_LUT_BIN_COUNT CONFIG_ESP_ANGLE_LUT_BIN_COUNT
#define ESP_ANGLE_LUT_PAYLOAD_BYTES (ESP_ANGLE_LUT_BIN_COUNT * sizeof(int16_t))

typedef struct {
  bool loaded;
  bool enabled;
  uint16_t format_version;
  uint16_t bin_count;
  uint32_t generation;
  uint32_t payload_crc32;
} esp_angle_lut_status_t;

/** Load the newest valid table from NVS. NVS flash must be initialized. */
esp_err_t esp_angle_lut_init(void);

/** Apply the active cyclic LUT to a 14-bit angle using linear interpolation. */
uint16_t esp_angle_lut_apply(uint16_t angle_counts);

/** Validate, persist and stage a new table. A newly installed table is off. */
esp_err_t esp_angle_lut_install(const int16_t *corrections, size_t bin_count,
                                uint32_t payload_crc32);

/** Copy the installed table and its metadata. */
esp_err_t esp_angle_lut_read(int16_t *corrections, size_t bin_count,
                             esp_angle_lut_status_t *status);

/** Persistently enable or disable application of the installed table. */
esp_err_t esp_angle_lut_set_enabled(bool enabled);

/** Remove both persistent slots and disable correction immediately. */
esp_err_t esp_angle_lut_clear(void);

/** Obtain a lock-free status snapshot. */
void esp_angle_lut_get_status(esp_angle_lut_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ESP_ANGLE_LUT_H_ */
