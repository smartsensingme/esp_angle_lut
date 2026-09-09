#include "esp_angle_lut.h"

#include "esp_crc.h"
#include "esp_log.h"
#include "nvs.h"
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#define LUT_MAGIC 0x544C4741U /* "AGLT" in little endian. */
#define LUT_NVS_NAMESPACE "angle_lut"
#define LUT_NVS_ACTIVE_KEY "active"
#define LUT_NVS_ENABLED_KEY "enabled"

#if (ESP_ANGLE_LUT_FULL_SCALE_COUNTS &                                         \
     (ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1)) != 0
#error "ESP_ANGLE_LUT_FULL_SCALE_COUNTS must be a power of two"
#endif
#if (ESP_ANGLE_LUT_FULL_SCALE_COUNTS % ESP_ANGLE_LUT_BIN_COUNT) != 0
#error "ESP_ANGLE_LUT_BIN_COUNT must divide the sensor full scale"
#endif
#if (ESP_ANGLE_LUT_BIN_COUNT & (ESP_ANGLE_LUT_BIN_COUNT - 1)) != 0
#error "ESP_ANGLE_LUT_BIN_COUNT must be a power of two"
#endif

_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
               "Angle LUT protocol requires a little-endian target");
_Static_assert(!(-1 > ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS),
               "Angle LUT correction limit must remain a signed expression");

typedef struct {
  uint32_t magic;
  uint16_t format_version;
  uint16_t bin_count;
  uint32_t full_scale_counts;
  uint32_t generation;
  uint32_t payload_crc32;
  int16_t corrections[ESP_ANGLE_LUT_BIN_COUNT];
} angle_lut_blob_t;

/* Version 1 used a fixed 16384-count scale. Keep it readable after upgrade. */
typedef struct {
  uint32_t magic;
  uint16_t format_version;
  uint16_t bin_count;
  uint32_t generation;
  uint32_t payload_crc32;
  int16_t corrections[ESP_ANGLE_LUT_BIN_COUNT];
} angle_lut_blob_v1_t;

static const char *TAG = "ANGLE_LUT";
static const char *const slot_keys[] = {"slot0", "slot1"};
static int16_t runtime_tables[2][ESP_ANGLE_LUT_BIN_COUNT];
/* Installation calls deeply into NVS; keep its 500+ byte blob off task stacks.
 */
static angle_lut_blob_t install_blob;
static atomic_uintptr_t active_table;
static atomic_bool table_loaded;
static atomic_bool table_enabled;
static atomic_uint_least32_t table_generation;
static atomic_uint_least32_t table_crc32;
static uint8_t runtime_table_index;
static uint8_t storage_slot;
static bool initialized;

static uint32_t payload_crc(const int16_t *corrections) {
  return esp_crc32_le(0U, (const uint8_t *)corrections,
                      ESP_ANGLE_LUT_PAYLOAD_BYTES);
}

static esp_err_t validate_table(const int16_t *corrections, size_t bin_count,
                                uint32_t full_scale_counts,
                                uint32_t expected_crc,
                                esp_angle_lut_install_failure_t *failure) {
  if (corrections == NULL || bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
      full_scale_counts != ESP_ANGLE_LUT_FULL_SCALE_COUNTS) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_METADATA;
    }
    return ESP_ERR_INVALID_ARG;
  }
  if (payload_crc(corrections) != expected_crc) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_CRC;
    }
    return ESP_ERR_INVALID_CRC;
  }

  const int32_t bin_width =
      ESP_ANGLE_LUT_FULL_SCALE_COUNTS / ESP_ANGLE_LUT_BIN_COUNT;
  for (size_t index = 0; index < ESP_ANGLE_LUT_BIN_COUNT; index++) {
    int32_t correction = corrections[index];
    if (correction > ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS ||
        correction < -ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS) {
      if (failure != NULL) {
        *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_CORRECTION_RANGE;
      }
      ESP_LOGE(TAG, "correction[%u]=%ld exceeds +/-%u counts", (unsigned)index,
               (long)correction,
               (unsigned)ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS);
      return ESP_ERR_INVALID_ARG;
    }
    size_t next = (index + 1U) % ESP_ANGLE_LUT_BIN_COUNT;
    int32_t corrected_step =
        bin_width + (int32_t)corrections[next] - correction;
    if (corrected_step <= 0 || corrected_step > 4 * bin_width) {
      if (failure != NULL) {
        *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NON_MONOTONIC;
      }
      ESP_LOGE(TAG, "corrected step[%u]=%ld outside 1..%ld counts",
               (unsigned)index, (long)corrected_step, (long)(4 * bin_width));
      return ESP_ERR_INVALID_ARG;
    }
  }
  return ESP_OK;
}

static bool blob_valid(const angle_lut_blob_t *blob) {
  return blob->magic == LUT_MAGIC &&
         blob->format_version == ESP_ANGLE_LUT_FORMAT_VERSION &&
         blob->bin_count == ESP_ANGLE_LUT_BIN_COUNT &&
         blob->full_scale_counts == ESP_ANGLE_LUT_FULL_SCALE_COUNTS &&
         validate_table(blob->corrections, blob->bin_count,
                        blob->full_scale_counts, blob->payload_crc32,
                        NULL) == ESP_OK;
}

static esp_err_t read_slot(nvs_handle_t handle, uint8_t slot,
                           angle_lut_blob_t *blob) {
  size_t size = 0U;
  esp_err_t error = nvs_get_blob(handle, slot_keys[slot], NULL, &size);
  if (error != ESP_OK) {
    return error;
  }
  if (size == sizeof(*blob)) {
    error = nvs_get_blob(handle, slot_keys[slot], blob, &size);
    if (error != ESP_OK) {
      return error;
    }
    return blob_valid(blob) ? ESP_OK : ESP_ERR_INVALID_CRC;
  }
  if (size == sizeof(angle_lut_blob_v1_t) &&
      ESP_ANGLE_LUT_FULL_SCALE_COUNTS == 16384U) {
    angle_lut_blob_v1_t legacy;
    error = nvs_get_blob(handle, slot_keys[slot], &legacy, &size);
    if (error != ESP_OK) {
      return error;
    }
    if (legacy.magic != LUT_MAGIC || legacy.format_version != 1U ||
        legacy.bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
        validate_table(legacy.corrections, legacy.bin_count, 16384U,
                       legacy.payload_crc32, NULL) != ESP_OK) {
      return ESP_ERR_INVALID_CRC;
    }
    *blob = (angle_lut_blob_t){
        .magic = LUT_MAGIC,
        .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
        .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
        .full_scale_counts = 16384U,
        .generation = legacy.generation,
        .payload_crc32 = legacy.payload_crc32,
    };
    memcpy(blob->corrections, legacy.corrections, ESP_ANGLE_LUT_PAYLOAD_BYTES);
    return ESP_OK;
  }
  return ESP_ERR_INVALID_SIZE;
}

static void activate_blob(const angle_lut_blob_t *blob, uint8_t slot) {
  uint8_t next_runtime = runtime_table_index ^ 1U;
  memcpy(runtime_tables[next_runtime], blob->corrections,
         ESP_ANGLE_LUT_PAYLOAD_BYTES);
  atomic_store_explicit(&active_table, (uintptr_t)runtime_tables[next_runtime],
                        memory_order_release);
  runtime_table_index = next_runtime;
  storage_slot = slot;
  atomic_store(&table_generation, blob->generation);
  atomic_store(&table_crc32, blob->payload_crc32);
  atomic_store(&table_loaded, true);
}

esp_err_t esp_angle_lut_init(void) {
  if (initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  atomic_init(&active_table, (uintptr_t)NULL);
  atomic_init(&table_loaded, false);
  atomic_init(&table_enabled, false);
  atomic_init(&table_generation, 0U);
  atomic_init(&table_crc32, 0U);

  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(LUT_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (error != ESP_OK) {
    return error;
  }

  uint8_t preferred_slot = 0U;
  (void)nvs_get_u8(handle, LUT_NVS_ACTIVE_KEY, &preferred_slot);
  preferred_slot &= 1U;
  angle_lut_blob_t preferred;
  angle_lut_blob_t fallback;
  esp_err_t preferred_error = read_slot(handle, preferred_slot, &preferred);
  esp_err_t fallback_error = read_slot(handle, preferred_slot ^ 1U, &fallback);
  if (preferred_error == ESP_OK) {
    activate_blob(&preferred, preferred_slot);
  } else if (fallback_error == ESP_OK) {
    activate_blob(&fallback, preferred_slot ^ 1U);
  }

  uint8_t enabled = 0U;
  (void)nvs_get_u8(handle, LUT_NVS_ENABLED_KEY, &enabled);
  atomic_store(&table_enabled, atomic_load(&table_loaded) && enabled != 0U);
  nvs_close(handle);
  initialized = true;

  esp_angle_lut_status_t status;
  esp_angle_lut_get_status(&status);
  ESP_LOGI(
      TAG,
      "initialized: loaded=%d enabled=%d generation=%lu bins=%u counts=%lu",
      status.loaded, status.enabled, (unsigned long)status.generation,
      status.bin_count, (unsigned long)status.full_scale_counts);
  return ESP_OK;
}

uint16_t esp_angle_lut_apply(uint16_t angle_counts) {
  angle_counts &= ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1U;
  if (!atomic_load_explicit(&table_enabled, memory_order_acquire)) {
    return angle_counts;
  }
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &active_table, memory_order_acquire);
  if (table == NULL) {
    return angle_counts;
  }

  const uint32_t bin_width =
      ESP_ANGLE_LUT_FULL_SCALE_COUNTS / ESP_ANGLE_LUT_BIN_COUNT;
  uint32_t index = angle_counts / bin_width;
  uint32_t fraction = angle_counts % bin_width;
  int32_t first = table[index];
  int32_t second = table[(index + 1U) % ESP_ANGLE_LUT_BIN_COUNT];
  int32_t interpolated =
      first + ((second - first) * (int32_t)fraction) / (int32_t)bin_width;
  int32_t corrected = (int32_t)angle_counts + interpolated;
  corrected %= (int32_t)ESP_ANGLE_LUT_FULL_SCALE_COUNTS;
  if (corrected < 0) {
    corrected += ESP_ANGLE_LUT_FULL_SCALE_COUNTS;
  }
  return (uint16_t)corrected;
}

esp_err_t esp_angle_lut_install(const int16_t *corrections, size_t bin_count,
                                uint32_t full_scale_counts,
                                uint32_t payload_crc32) {
  return esp_angle_lut_install_detailed(corrections, bin_count,
                                        full_scale_counts, payload_crc32, NULL);
}

esp_err_t esp_angle_lut_install_detailed(
    const int16_t *corrections, size_t bin_count, uint32_t full_scale_counts,
    uint32_t payload_crc32, esp_angle_lut_install_failure_t *failure) {
  if (failure != NULL) {
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;
  }
  if (!initialized) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NOT_INITIALIZED;
    }
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t error = validate_table(corrections, bin_count, full_scale_counts,
                                   payload_crc32, failure);
  if (error != ESP_OK) {
    return error;
  }

  install_blob = (angle_lut_blob_t){
      .magic = LUT_MAGIC,
      .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
      .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
      .full_scale_counts = ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
      .generation = atomic_load(&table_generation) + 1U,
      .payload_crc32 = payload_crc32,
  };
  memcpy(install_blob.corrections, corrections, ESP_ANGLE_LUT_PAYLOAD_BYTES);
  uint8_t next_slot = atomic_load(&table_loaded) ? storage_slot ^ 1U : 0U;

  nvs_handle_t handle = 0;
  error = nvs_open(LUT_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (error != ESP_OK && failure != NULL) {
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN;
  }
  if (error == ESP_OK) {
    error = nvs_set_blob(handle, slot_keys[next_slot], &install_blob,
                         sizeof(install_blob));
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_BLOB_WRITE;
    }
  }
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ACTIVE_KEY, next_slot);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ACTIVE_WRITE;
    }
  }
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ENABLED_KEY, 0U);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE;
    }
  }
  if (error == ESP_OK) {
    error = nvs_commit(handle);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT;
    }
  }
  if (handle != 0) {
    nvs_close(handle);
  }
  if (error != ESP_OK) {
    ESP_LOGE(
        TAG, "installation failed at %s: %s",
        esp_angle_lut_install_failure_name(
            failure != NULL ? *failure : ESP_ANGLE_LUT_INSTALL_FAILURE_NONE),
        esp_err_to_name(error));
    return error;
  }

  atomic_store_explicit(&table_enabled, false, memory_order_release);
  activate_blob(&install_blob, next_slot);
  return ESP_OK;
}

const char *
esp_angle_lut_install_failure_name(esp_angle_lut_install_failure_t failure) {
  switch (failure) {
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NONE:
    return "none";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NOT_INITIALIZED:
    return "not_initialized";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_METADATA:
    return "invalid_metadata";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_CRC:
    return "crc_mismatch";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_CORRECTION_RANGE:
    return "correction_out_of_range";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NON_MONOTONIC:
    return "non_monotonic_lut";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN:
    return "nvs_open_failed";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_BLOB_WRITE:
    return "nvs_blob_write_failed";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ACTIVE_WRITE:
    return "nvs_active_write_failed";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE:
    return "nvs_enabled_write_failed";
  case ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT:
    return "nvs_commit_failed";
  default:
    return "unknown_install_failure";
  }
}

esp_err_t esp_angle_lut_read(int16_t *corrections, size_t bin_count,
                             esp_angle_lut_status_t *status) {
  if (corrections == NULL || bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
      !atomic_load(&table_loaded)) {
    return ESP_ERR_INVALID_STATE;
  }
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &active_table, memory_order_acquire);
  if (table == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  memcpy(corrections, table, ESP_ANGLE_LUT_PAYLOAD_BYTES);
  if (status != NULL) {
    esp_angle_lut_get_status(status);
  }
  return ESP_OK;
}

esp_err_t esp_angle_lut_set_enabled(bool enabled) {
  if (!initialized || (enabled && !atomic_load(&table_loaded))) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(LUT_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ENABLED_KEY, enabled ? 1U : 0U);
  }
  if (error == ESP_OK) {
    error = nvs_commit(handle);
  }
  if (handle != 0) {
    nvs_close(handle);
  }
  if (error == ESP_OK) {
    atomic_store_explicit(&table_enabled, enabled, memory_order_release);
  }
  return error;
}

esp_err_t esp_angle_lut_clear(void) {
  if (!initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  atomic_store_explicit(&table_enabled, false, memory_order_release);
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(LUT_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (error == ESP_OK) {
    (void)nvs_erase_key(handle, slot_keys[0]);
    (void)nvs_erase_key(handle, slot_keys[1]);
    (void)nvs_erase_key(handle, LUT_NVS_ACTIVE_KEY);
    (void)nvs_erase_key(handle, LUT_NVS_ENABLED_KEY);
    error = nvs_commit(handle);
  }
  if (handle != 0) {
    nvs_close(handle);
  }
  if (error == ESP_OK) {
    atomic_store(&table_loaded, false);
    atomic_store(&table_generation, 0U);
    atomic_store(&table_crc32, 0U);
    atomic_store(&active_table, (uintptr_t)NULL);
  }
  return error;
}

void esp_angle_lut_get_status(esp_angle_lut_status_t *status) {
  if (status == NULL) {
    return;
  }
  *status = (esp_angle_lut_status_t){
      .loaded = atomic_load(&table_loaded),
      .enabled = atomic_load(&table_enabled),
      .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
      .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
      .full_scale_counts = ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
      .max_abs_correction_counts = ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS,
      .generation = atomic_load(&table_generation),
      .payload_crc32 = atomic_load(&table_crc32),
  };
}
