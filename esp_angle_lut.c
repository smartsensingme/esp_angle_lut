#include "esp_angle_lut.h"

#include "esp_attr.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs.h"
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
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
/* Each instance owns runtime buffers, staging and a distinct NVS namespace.
 * Management is serialized by the application; stop readers before mutation.
 */
struct esp_angle_lut {
  int16_t runtime_tables[2][ESP_ANGLE_LUT_BIN_COUNT];
  angle_lut_blob_t install_blob;
  atomic_uintptr_t active_table;
  atomic_bool table_loaded;
  atomic_bool table_enabled;
  atomic_uint_least32_t table_generation;
  atomic_uint_least32_t table_crc32;
  uint8_t runtime_table_index;
  uint8_t storage_slot;
  bool initialized;
  char nvs_namespace[16];
  struct esp_angle_lut *next;
};
static DRAM_ATTR struct esp_angle_lut default_instance = {
    .nvs_namespace = LUT_NVS_NAMESPACE};
static esp_angle_lut_handle_t instances;

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
 * and esp_angle_lut_instance_install_detailed(). It optionally records the
 * precise public failure stage while returning the corresponding esp_err_t
 * value.
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
 * This private helper is called twice by load_instance(), once for the
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
 * This private helper is called by load_instance() after loading NVS and
 * by esp_angle_lut_instance_install_detailed() after committing a new table. It
 * writes the inactive runtime buffer first and publishes its pointer with
 * release ordering only after the complete table copy.
 */
static void activate_blob(esp_angle_lut_handle_t lut,
                          const angle_lut_blob_t *blob, uint8_t slot) {
  /* Readers are stopped by the maintenance contract before buffer reuse. */
  uint8_t next_runtime = lut->runtime_table_index ^ 1U;
  memcpy(lut->runtime_tables[next_runtime], blob->corrections,
         ESP_ANGLE_LUT_PAYLOAD_BYTES);

  /* Atomically publish the completed table before updating bookkeeping. */
  atomic_store_explicit(&lut->active_table,
                        (uintptr_t)lut->runtime_tables[next_runtime],
                        memory_order_release);
  lut->runtime_table_index = next_runtime;
  lut->storage_slot = slot;

  /* Publish metadata corresponding to the newly active correction payload. */
  atomic_store(&lut->table_generation, blob->generation);
  atomic_store(&lut->table_crc32, blob->payload_crc32);
  atomic_store(&lut->table_loaded, true);
}

static esp_err_t load_instance(esp_angle_lut_handle_t lut) {
  /* Enforce the component's one-time initialization contract. */
  if (lut->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Establish a known lock-free state before any persistent data is inspected.
   */
  atomic_init(&lut->active_table, (uintptr_t)NULL);
  atomic_init(&lut->table_loaded, false);
  atomic_init(&lut->table_enabled, false);
  atomic_init(&lut->table_generation, 0U);
  atomic_init(&lut->table_crc32, 0U);

  /* Open the namespace shared by both redundant slots and control keys. */
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(lut->nvs_namespace, NVS_READWRITE, &handle);
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
    activate_blob(lut, &preferred, preferred_slot);
  } else if (fallback_error == ESP_OK) {
    activate_blob(lut, &fallback, preferred_slot ^ 1U);
  }

  /* Restore enablement only when a valid table was successfully activated. */
  uint8_t enabled = 0U;
  (void)nvs_get_u8(handle, LUT_NVS_ENABLED_KEY, &enabled);
  atomic_store(&lut->table_enabled,
               atomic_load(&lut->table_loaded) && enabled != 0U);
  nvs_close(handle);
  lut->initialized = true;

  /* Log the same lock-free status snapshot exposed to application code. */
  esp_angle_lut_status_t status;
  esp_angle_lut_instance_get_status(lut, &status);
  ESP_LOGI(
      TAG,
      "initialized: loaded=%d enabled=%d generation=%lu bins=%u counts=%lu",
      status.loaded, status.enabled, (unsigned long)status.generation,
      status.bin_count, (unsigned long)status.full_scale_counts);
  return ESP_OK;
}

uint16_t IRAM_ATTR esp_angle_lut_instance_apply(esp_angle_lut_handle_t lut,
                                                uint16_t angle_counts) {
  /* Normalize arbitrary uint16_t input efficiently using the power-of-two
   * scale. */
  angle_counts &= ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1U;

  /* Bypass correction with minimal latency when the persisted state is off. */
  if (!lut ||
      !atomic_load_explicit(&lut->table_enabled, memory_order_acquire)) {
    return angle_counts;
  }

  /* Acquire the fully published immutable runtime buffer for this call. */
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &lut->active_table, memory_order_acquire);
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

/* Maintenance-only enable persistence shared by installation and explicit
 * enable changes. Retains exact NVS failure-stage reporting. */
static esp_err_t persist_enabled(esp_angle_lut_handle_t lut, bool enabled,
                                 esp_angle_lut_install_failure_t *failure) {
  /* Enabling is meaningful only after initialization and a valid installation.
   */
  if (!lut || !lut->initialized ||
      (enabled && !atomic_load(&lut->table_loaded))) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Persist the requested state before exposing it to real-time readers. */
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(lut->nvs_namespace, NVS_READWRITE, &handle);
  if (error != ESP_OK && failure)
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN;
  if (error == ESP_OK) {
    error = nvs_set_u8(handle, LUT_NVS_ENABLED_KEY, enabled ? 1U : 0U);
    if (error != ESP_OK && failure)
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE;
  }
  if (error == ESP_OK) {
    error = nvs_commit(handle);
    if (error != ESP_OK && failure)
      *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT;
  }
  if (handle != 0) {
    nvs_close(handle);
  }

  /* Publish only a state that NVS has committed successfully. */
  if (error == ESP_OK) {
    atomic_store_explicit(&lut->table_enabled, enabled, memory_order_release);
  }
  return error;
}

esp_err_t esp_angle_lut_instance_install(esp_angle_lut_handle_t lut,
                                         const int16_t *corrections,
                                         size_t bin_count,
                                         uint32_t full_scale_counts,
                                         uint32_t payload_crc32) {
  /* Delegate to the detailed API when the caller does not need a failure stage.
   */
  return esp_angle_lut_instance_install_detailed(
      lut, corrections, bin_count, full_scale_counts, payload_crc32, NULL);
}

esp_err_t esp_angle_lut_instance_install_detailed(
    esp_angle_lut_handle_t lut, const int16_t *corrections, size_t bin_count,
    uint32_t full_scale_counts, uint32_t payload_crc32,
    esp_angle_lut_install_failure_t *failure) {
  /* Initialize optional diagnostic output before any operation can fail. */
  if (failure != NULL) {
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;
  }
  if (!lut || !lut->initialized) {
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

  /* Disable durably before touching the inactive slot or selector. NVS set
   * operations are not a multi-key atomic transaction. An interrupted install
   * may retain either validated table, but must never auto-enable a new one.
   */
  error = persist_enabled(lut, false, failure);
  if (error != ESP_OK)
    return error;

  /* Build the next versioned blob in per-instance staging storage. */
  lut->install_blob = (angle_lut_blob_t){
      .magic = LUT_MAGIC,
      .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
      .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
      .full_scale_counts = ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
      .generation = atomic_load(&lut->table_generation) + 1U,
      .payload_crc32 = payload_crc32,
  };
  memcpy(lut->install_blob.corrections, corrections,
         ESP_ANGLE_LUT_PAYLOAD_BYTES);

  /* Preserve the active slot until the replacement has been written. */
  uint8_t next_slot =
      atomic_load(&lut->table_loaded) ? lut->storage_slot ^ 1U : 0U;

  /* Open NVS and write the complete replacement blob to the inactive slot. */
  nvs_handle_t handle = 0;
  error = nvs_open(lut->nvs_namespace, NVS_READWRITE, &handle);
  if (error != ESP_OK && failure != NULL) {
    *failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN;
  }
  if (error == ESP_OK) {
    error = nvs_set_blob(handle, slot_keys[next_slot], &lut->install_blob,
                         sizeof(lut->install_blob));
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

  /* Commit pending storage operations; the disabled flag is already durable. */
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
  atomic_store_explicit(&lut->table_enabled, false, memory_order_release);
  activate_blob(lut, &lut->install_blob, next_slot);
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

esp_err_t esp_angle_lut_instance_read(esp_angle_lut_handle_t lut,
                                      int16_t *corrections, size_t bin_count,
                                      esp_angle_lut_status_t *status) {
  /* Validate destination capacity and require a published table. */
  if (!lut || corrections == NULL || bin_count != ESP_ANGLE_LUT_BIN_COUNT ||
      !atomic_load(&lut->table_loaded)) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Acquire the immutable table selected by the latest successful publication.
   */
  const int16_t *table = (const int16_t *)atomic_load_explicit(
      &lut->active_table, memory_order_acquire);
  if (table == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Copy the complete payload before taking the optional metadata snapshot. */
  memcpy(corrections, table, ESP_ANGLE_LUT_PAYLOAD_BYTES);
  if (status != NULL) {
    esp_angle_lut_instance_get_status(lut, status);
  }
  return ESP_OK;
}

esp_err_t esp_angle_lut_instance_set_enabled(esp_angle_lut_handle_t lut,
                                             bool enabled) {
  return persist_enabled(lut, enabled, NULL);
}

esp_err_t esp_angle_lut_instance_clear(esp_angle_lut_handle_t lut) {
  /* Reject calls made before the component owns a valid runtime state. */
  if (!lut || !lut->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Stop correction immediately so readers cannot use data being erased. */
  atomic_store_explicit(&lut->table_enabled, false, memory_order_release);

  /* Erase both redundant blobs and their selector/control metadata. */
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(lut->nvs_namespace, NVS_READWRITE, &handle);
  if (error == ESP_OK) {
    const char *keys[] = {slot_keys[0], slot_keys[1], LUT_NVS_ACTIVE_KEY,
                          LUT_NVS_ENABLED_KEY};
    for (unsigned i = 0; i < 4 && error == ESP_OK; ++i) {
      error = nvs_erase_key(handle, keys[i]);
      if (error == ESP_ERR_NVS_NOT_FOUND)
        error = ESP_OK;
    }
    if (error == ESP_OK)
      error = nvs_commit(handle);
  }
  if (handle != 0) {
    nvs_close(handle);
  }

  /* Forget the runtime table only after the persistent erase is durable. */
  if (error == ESP_OK) {
    atomic_store(&lut->table_loaded, false);
    atomic_store(&lut->table_generation, 0U);
    atomic_store(&lut->table_crc32, 0U);
    atomic_store(&lut->active_table, (uintptr_t)NULL);
  }
  return error;
}

void esp_angle_lut_instance_get_status(esp_angle_lut_handle_t lut,
                                       esp_angle_lut_status_t *status) {
  /* A null optional destination is intentionally a no-op. */
  if (!lut || status == NULL) {
    return;
  }

  /* Combine immutable build metadata with atomically published runtime state.
   */
  *status = (esp_angle_lut_status_t){
      .loaded = atomic_load(&lut->table_loaded),
      .enabled = atomic_load(&lut->table_enabled),
      .format_version = ESP_ANGLE_LUT_FORMAT_VERSION,
      .bin_count = ESP_ANGLE_LUT_BIN_COUNT,
      .full_scale_counts = ESP_ANGLE_LUT_FULL_SCALE_COUNTS,
      .max_abs_correction_counts = ESP_ANGLE_LUT_MAX_ABS_CORRECTION_COUNTS,
      .generation = atomic_load(&lut->table_generation),
      .payload_crc32 = atomic_load(&lut->table_crc32),
  };
}

/* Lifecycle calls run only in the application's serialized maintenance context.
 * No live readers may retain a handle being destroyed. */
esp_err_t esp_angle_lut_create(const char *nvs_namespace,
                               esp_angle_lut_handle_t *out) {
  if (!out)
    return ESP_ERR_INVALID_ARG;
  *out = NULL;
  if (!nvs_namespace || !nvs_namespace[0] || strlen(nvs_namespace) > 15 ||
      strcmp(nvs_namespace, LUT_NVS_NAMESPACE) == 0)
    return ESP_ERR_INVALID_ARG;
  for (esp_angle_lut_handle_t p = instances; p; p = p->next)
    if (strcmp(p->nvs_namespace, nvs_namespace) == 0)
      return ESP_ERR_INVALID_STATE;
  esp_angle_lut_handle_t lut =
      heap_caps_calloc(1, sizeof(*lut), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!lut)
    return ESP_ERR_NO_MEM;
  memcpy(lut->nvs_namespace, nvs_namespace, strlen(nvs_namespace) + 1);
  esp_err_t error = load_instance(lut);
  if (error != ESP_OK) {
    free(lut);
    return error;
  }
  lut->next = instances;
  instances = lut;
  *out = lut;
  return ESP_OK;
}

esp_err_t esp_angle_lut_destroy(esp_angle_lut_handle_t lut) {
  if (!lut || lut == &default_instance)
    return ESP_ERR_INVALID_ARG;
  esp_angle_lut_handle_t *link = &instances;
  while (*link && *link != lut)
    link = &(*link)->next;
  if (!*link)
    return ESP_ERR_INVALID_ARG;
  *link = lut->next;
  free(lut);
  return ESP_OK;
}

/* Legacy entry points preserve signatures and the historical NVS namespace. */
esp_err_t esp_angle_lut_init(void) { return load_instance(&default_instance); }
uint16_t IRAM_ATTR esp_angle_lut_apply(uint16_t angle) {
  return esp_angle_lut_instance_apply(&default_instance, angle);
}
esp_err_t esp_angle_lut_install(const int16_t *c, size_t n, uint32_t scale,
                                uint32_t crc) {
  return esp_angle_lut_instance_install(&default_instance, c, n, scale, crc);
}
esp_err_t
esp_angle_lut_install_detailed(const int16_t *c, size_t n, uint32_t scale,
                               uint32_t crc,
                               esp_angle_lut_install_failure_t *failure) {
  return esp_angle_lut_instance_install_detailed(&default_instance, c, n, scale,
                                                 crc, failure);
}
esp_err_t esp_angle_lut_read(int16_t *c, size_t n,
                             esp_angle_lut_status_t *status) {
  return esp_angle_lut_instance_read(&default_instance, c, n, status);
}
esp_err_t esp_angle_lut_set_enabled(bool enabled) {
  return esp_angle_lut_instance_set_enabled(&default_instance, enabled);
}
esp_err_t esp_angle_lut_clear(void) {
  return esp_angle_lut_instance_clear(&default_instance);
}
void esp_angle_lut_get_status(esp_angle_lut_status_t *status) {
  esp_angle_lut_instance_get_status(&default_instance, status);
}
