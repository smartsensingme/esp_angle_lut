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

/**
 * @brief Calculate the IEEE CRC32 of one complete correction payload.
 *
 * This private helper is called only by validate_table(). The payload length is
 * fixed by the compiled LUT bin count, and the in-memory little-endian int16_t
 * representation is the same byte representation used by the host protocol.
 */
static uint32_t payload_crc(const int16_t *corrections) {
  /* Reinterpret the signed table as bytes without copying or changing order. */
  return esp_crc32_le(0U, (const uint8_t *)corrections,
                      ESP_ANGLE_LUT_PAYLOAD_BYTES);
}

/**
 * @brief Validate table metadata, payload integrity and angular monotonicity.
 *
 * This private helper is called by blob_valid(), read_slot() for legacy blobs,
 * and esp_angle_lut_install_detailed(). It optionally records the precise
 * public failure stage while returning the corresponding esp_err_t value.
 */
static esp_err_t validate_table(const int16_t *corrections, size_t bin_count,
                                uint32_t full_scale_counts,
                                uint32_t expected_crc,
                                esp_angle_lut_install_failure_t *failure) {
  /* The host metadata must exactly match this compiled firmware instance. */
  if (corrections == NULL || bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
      full_scale_counts != ESP_ANGLE_LUT_FULL_SCALE_COUNTS) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_METADATA;
    }
    return ESP_ERR_INVALID_ARG;
  }

  /* Reject corrupted or incorrectly serialized correction payloads. */
  if (payload_crc(corrections) != expected_crc) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_CRC;
    }
    return ESP_ERR_INVALID_CRC;
  }

  const int32_t bin_width =
      ESP_ANGLE_LUT_FULL_SCALE_COUNTS / ESP_ANGLE_LUT_BIN_COUNT;

  /* Validate every correction and every cyclic step to the following bin. */
  for (size_t index = 0; index < ESP_ANGLE_LUT_BIN_COUNT; index++) {
    int32_t correction = corrections[index];

    /* Bound the possible angular displacement of every table entry. */
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

    /*
     * Add the difference between adjacent corrections to the nominal bin
     * width. A nonpositive result would reverse or collapse the angle map.
     * The upper bound rejects implausibly abrupt local stretching.
     */
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

/**
 * @brief Check a current-format persistent blob and its correction table.
 *
 * This private helper is called only by read_slot() when the stored blob size
 * identifies the current format. It calls validate_table() after checking the
 * blob envelope fields.
 */
static bool blob_valid(const angle_lut_blob_t *blob) {
  /* Short-circuit before reading the payload when its envelope is incompatible.
   */
  return blob->magic == LUT_MAGIC &&
         blob->format_version == ESP_ANGLE_LUT_FORMAT_VERSION &&
         blob->bin_count == ESP_ANGLE_LUT_BIN_COUNT &&
         blob->full_scale_counts == ESP_ANGLE_LUT_FULL_SCALE_COUNTS &&
         validate_table(blob->corrections, blob->bin_count,
                        blob->full_scale_counts, blob->payload_crc32,
                        NULL) == ESP_OK;
}

/**
 * @brief Read and validate one persistent NVS slot.
 *
 * This private helper is called twice by esp_angle_lut_init(), once for the
 * preferred slot and once for the fallback. It calls blob_valid() for current
 * blobs and validate_table() while migrating a compatible version-1 blob.
 */
static esp_err_t read_slot(nvs_handle_t handle, uint8_t slot,
                           angle_lut_blob_t *blob) {
  /* Query the stored size first so the on-flash format can be selected safely.
   */
  size_t size = 0U;
  esp_err_t error = nvs_get_blob(handle, slot_keys[slot], NULL, &size);
  if (error != ESP_OK) {
    return error;
  }

  /* Read a current-format blob directly and validate its envelope and table. */
  if (size == sizeof(*blob)) {
    error = nvs_get_blob(handle, slot_keys[slot], blob, &size);
    if (error != ESP_OK) {
      return error;
    }
    return blob_valid(blob) ? ESP_OK : ESP_ERR_INVALID_CRC;
  }

  /*
   * Accept version 1 only at its historical fixed 16384-count resolution, then
   * convert its envelope to version 2 while preserving payload and generation.
   */
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

  /* Any other byte length is neither a supported current nor legacy blob. */
  return ESP_ERR_INVALID_SIZE;
}

/**
 * @brief Publish a validated blob to readers without exposing partial data.
 *
 * This private helper is called by esp_angle_lut_init() after loading NVS and
 * by esp_angle_lut_install_detailed() after committing a new table. It writes
 * the inactive runtime buffer first and publishes its pointer with release
 * ordering only after the complete table copy.
 */
static void activate_blob(const angle_lut_blob_t *blob, uint8_t slot) {
  /* Prepare the buffer that cannot currently be observed by real-time readers.
   */
  uint8_t next_runtime = runtime_table_index ^ 1U;
  memcpy(runtime_tables[next_runtime], blob->corrections,
         ESP_ANGLE_LUT_PAYLOAD_BYTES);

  /* Atomically publish the completed table before updating bookkeeping. */
  atomic_store_explicit(&active_table, (uintptr_t)runtime_tables[next_runtime],
                        memory_order_release);
  runtime_table_index = next_runtime;
  storage_slot = slot;

  /* Publish metadata corresponding to the newly active correction payload. */
  atomic_store(&table_generation, blob->generation);
  atomic_store(&table_crc32, blob->payload_crc32);
  atomic_store(&table_loaded, true);
}

esp_err_t esp_angle_lut_init(void) {
  /* Enforce the component's one-time initialization contract. */
  if (initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Establish a known lock-free state before any persistent data is inspected.
   */
  atomic_init(&active_table, (uintptr_t)NULL);
  atomic_init(&table_loaded, false);
  atomic_init(&table_enabled, false);
  atomic_init(&table_generation, 0U);
  atomic_init(&table_crc32, 0U);

  /* Open the namespace shared by both redundant slots and control keys. */
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(LUT_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (error != ESP_OK) {
    return error;
  }

  /*
   * Load both slots. Prefer the slot named by the active key, but recover from
   * an interrupted/corrupt write by accepting the other valid slot.
   */
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

  /* Restore enablement only when a valid table was successfully activated. */
  uint8_t enabled = 0U;
  (void)nvs_get_u8(handle, LUT_NVS_ENABLED_KEY, &enabled);
  atomic_store(&table_enabled, atomic_load(&table_loaded) && enabled != 0U);
  nvs_close(handle);
  initialized = true;

  /* Log the same lock-free status snapshot exposed to application code. */
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
  /* Normalize arbitrary uint16_t input efficiently using the power-of-two
   * scale. */
  angle_counts &= ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1U;

  /* Bypass correction with minimal latency when the persisted state is off. */
  if (!atomic_load_explicit(&table_enabled, memory_order_acquire)) {
    return angle_counts;
  }

  /* Acquire the fully published immutable runtime buffer for this call. */
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &active_table, memory_order_acquire);
  if (table == NULL) {
    return angle_counts;
  }

  /* Locate the lower LUT bin and the fractional position within that bin. */
  const uint32_t bin_width =
      ESP_ANGLE_LUT_FULL_SCALE_COUNTS / ESP_ANGLE_LUT_BIN_COUNT;
  uint32_t index = angle_counts / bin_width;
  uint32_t fraction = angle_counts % bin_width;
  int32_t first = table[index];
  int32_t second = table[(index + 1U) % ESP_ANGLE_LUT_BIN_COUNT];

  /* Linearly interpolate adjacent signed corrections using integer arithmetic.
   */
  int32_t interpolated =
      first + ((second - first) * (int32_t)fraction) / (int32_t)bin_width;

  /* Add the correction and wrap the result back into one full revolution. */
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
  /* Delegate to the detailed API when the caller does not need a failure stage.
   */
  return esp_angle_lut_install_detailed(corrections, bin_count,
                                        full_scale_counts, payload_crc32, NULL);
}

esp_err_t esp_angle_lut_install_detailed(
    const int16_t *corrections, size_t bin_count, uint32_t full_scale_counts,
    uint32_t payload_crc32, esp_angle_lut_install_failure_t *failure) {
  /* Initialize optional diagnostic output before any operation can fail. */
  if (failure != NULL) {
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;
  }
  if (!initialized) {
    if (failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NOT_INITIALIZED;
    }
    return ESP_ERR_INVALID_STATE;
  }

  /* Validate all host-provided data before modifying persistent state. */
  esp_err_t error = validate_table(corrections, bin_count, full_scale_counts,
                                   payload_crc32, failure);
  if (error != ESP_OK) {
    return error;
  }

  /* Build the next versioned blob in static staging storage. */
  install_blob = (angle_lut_blob_t){
      .magic = LUT_MAGIC,
      .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
      .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
      .full_scale_counts = ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
      .generation = atomic_load(&table_generation) + 1U,
      .payload_crc32 = payload_crc32,
  };
  memcpy(install_blob.corrections, corrections, ESP_ANGLE_LUT_PAYLOAD_BYTES);

  /* Preserve the active slot until the replacement has been written. */
  uint8_t next_slot = atomic_load(&table_loaded) ? storage_slot ^ 1U : 0U;

  /* Open NVS and write the complete replacement blob to the inactive slot. */
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

  /* Select the new slot only after its full blob write succeeded. */
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ACTIVE_KEY, next_slot);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ACTIVE_WRITE;
    }
  }

  /* Every new calibration starts disabled and requires explicit validation. */
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ENABLED_KEY, 0U);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE;
    }
  }

  /* Make the blob, selected-slot key, and disabled state durable together. */
  if (error == ESP_OK) {
    error = nvs_commit(handle);
    if (error != ESP_OK && failure != NULL) {
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT;
    }
  }
  if (handle != 0) {
    nvs_close(handle);
  }

  /* Report the exact failed persistence stage without changing the runtime LUT.
   */
  if (error != ESP_OK) {
    ESP_LOGE(
        TAG, "installation failed at %s: %s",
        esp_angle_lut_install_failure_name(
            failure != NULL ? *failure : ESP_ANGLE_LUT_INSTALL_FAILURE_NONE),
        esp_err_to_name(error));
    return error;
  }

  /* Publish the committed table atomically, while leaving correction disabled.
   */
  atomic_store_explicit(&table_enabled, false, memory_order_release);
  activate_blob(&install_blob, next_slot);
  return ESP_OK;
}

const char *
esp_angle_lut_install_failure_name(esp_angle_lut_install_failure_t failure) {
  /* Keep these names stable because transports expose them as protocol values.
   */
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
  /* Validate destination capacity and require a published table. */
  if (corrections == NULL || bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
      !atomic_load(&table_loaded)) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Acquire the immutable table selected by the latest successful publication.
   */
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &active_table, memory_order_acquire);
  if (table == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Copy the complete payload before taking the optional metadata snapshot. */
  memcpy(corrections, table, ESP_ANGLE_LUT_PAYLOAD_BYTES);
  if (status != NULL) {
    esp_angle_lut_get_status(status);
  }
  return ESP_OK;
}

esp_err_t esp_angle_lut_set_enabled(bool enabled) {
  /* Enabling is meaningful only after initialization and a valid installation.
   */
  if (!initialized || (enabled && !atomic_load(&table_loaded))) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Persist the requested state before exposing it to real-time readers. */
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

  /* Publish only a state that NVS has committed successfully. */
  if (error == ESP_OK) {
    atomic_store_explicit(&table_enabled, enabled, memory_order_release);
  }
  return error;
}

esp_err_t esp_angle_lut_clear(void) {
  /* Reject calls made before the component owns a valid runtime state. */
  if (!initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Stop correction immediately so readers cannot use data being erased. */
  atomic_store_explicit(&table_enabled, false, memory_order_release);

  /* Erase both redundant blobs and their selector/control metadata. */
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

  /* Forget the runtime table only after the persistent erase is durable. */
  if (error == ESP_OK) {
    atomic_store(&table_loaded, false);
    atomic_store(&table_generation, 0U);
    atomic_store(&table_crc32, 0U);
    atomic_store(&active_table, (uintptr_t)NULL);
  }
  return error;
}

void esp_angle_lut_get_status(esp_angle_lut_status_t *status) {
  /* A null optional destination is intentionally a no-op. */
  if (status == NULL) {
    return;
  }

  /* Combine immutable build metadata with atomically published runtime state.
   */
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
