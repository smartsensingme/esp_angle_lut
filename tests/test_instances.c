#include "../esp_angle_lut.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Model individual NVS writes becoming visible immediately, not a fictitious
 * multi-key transaction. Inject a failure at each write/commit position. */
static struct {
  char name[16];
  struct {
    bool used;
    size_t n;
    unsigned char bytes[1024];
  } key[4];
} store[8];
static int fail_after = -1;
static bool failing(void) {
  if (fail_after < 0)
    return false;
  if (fail_after-- == 0) {
    fail_after = -1;
    return true;
  }
  return false;
}
static unsigned key_index(const char *k) {
  if (!strcmp(k, "slot0"))
    return 0;
  if (!strcmp(k, "slot1"))
    return 1;
  if (!strcmp(k, "active"))
    return 2;
  assert(!strcmp(k, "enabled"));
  return 3;
}
esp_err_t nvs_open(const char *n, int mode, nvs_handle_t *h) {
  (void)mode;
  if (failing())
    return ESP_FAIL;
  for (unsigned i = 0; i < 8; i++)
    if (!strcmp(store[i].name, n) || !store[i].name[0]) {
      strcpy(store[i].name, n);
      *h = i + 1;
      return ESP_OK;
    }
  return ESP_FAIL;
}
esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *p, size_t *n) {
  unsigned j = key_index(k);
  if (!store[h - 1].key[j].used)
    return ESP_ERR_NVS_NOT_FOUND;
  size_t len = store[h - 1].key[j].n;
  if (p) {
    if (*n < len)
      return ESP_ERR_INVALID_SIZE;
    memcpy(p, store[h - 1].key[j].bytes, len);
  }
  *n = len;
  return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *p, size_t n) {
  if (failing())
    return ESP_FAIL;
  unsigned j = key_index(k);
  assert(n <= 1024);
  memcpy(store[h - 1].key[j].bytes, p, n);
  store[h - 1].key[j].n = n;
  store[h - 1].key[j].used = true;
  return ESP_OK;
}
esp_err_t nvs_get_u8(nvs_handle_t h, const char *k, uint8_t *v) {
  size_t n = 1;
  return nvs_get_blob(h, k, v, &n);
}
esp_err_t nvs_set_u8(nvs_handle_t h, const char *k, uint8_t v) {
  return nvs_set_blob(h, k, &v, 1);
}
esp_err_t nvs_erase_key(nvs_handle_t h, const char *k) {
  if (failing())
    return ESP_FAIL;
  unsigned j = key_index(k);
  if (!store[h - 1].key[j].used)
    return ESP_ERR_NVS_NOT_FOUND;
  store[h - 1].key[j].used = false;
  return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) {
  (void)h;
  return failing() ? ESP_FAIL : ESP_OK;
}
void nvs_close(nvs_handle_t h) { (void)h; }

static void install(esp_angle_lut_handle_t h, int value) {
  int16_t c[ESP_ANGLE_LUT_BIN_COUNT];
  for (unsigned i = 0; i < ESP_ANGLE_LUT_BIN_COUNT; i++)
    c[i] = value;
  assert(esp_angle_lut_instance_install(h, c, ESP_ANGLE_LUT_BIN_COUNT, 16384,
                                        payload_crc(c)) == ESP_OK);
  assert(esp_angle_lut_instance_set_enabled(h, true) == ESP_OK);
}
int main(void) {
  esp_angle_lut_handle_t l, r, dup;
  assert(esp_crc32_le(0, (const uint8_t *)"123456789", 9) == 0xcbf43926);
  assert(esp_angle_lut_create(NULL, &dup) == ESP_ERR_INVALID_ARG && !dup);
  assert(esp_angle_lut_create("angle_lut", &dup) == ESP_ERR_INVALID_ARG);
  assert(esp_angle_lut_create("robot_left", &l) == ESP_OK);
  assert(esp_angle_lut_create("robot_right", &r) == ESP_OK);
  assert(esp_angle_lut_create("robot_left", &dup) == ESP_ERR_INVALID_STATE &&
         !dup);
  assert(esp_angle_lut_instance_apply(NULL, 65535) == 16383);
  assert(esp_angle_lut_instance_apply(l, 65535) == 16383);
  assert(esp_angle_lut_instance_set_enabled(l, true) == ESP_ERR_INVALID_STATE);
  install(l, 12);
  install(r, -9);
  for (unsigned i = 0; i < 16384; i++) {
    assert(esp_angle_lut_instance_apply(l, i) == ((i + 12) & 16383));
    assert(esp_angle_lut_instance_apply(r, i) == ((i + 16384 - 9) & 16383));
  }
  assert(esp_angle_lut_destroy(l) == ESP_OK);
  assert(esp_angle_lut_create("robot_left", &l) == ESP_OK);
  assert(esp_angle_lut_instance_apply(l, 16380) == 8);
  int16_t table[256] = {0};
  esp_angle_lut_install_failure_t reason;
  assert(esp_angle_lut_instance_install_detailed(l, table, 256, 4096,
                                                 payload_crc(table), &reason) ==
         ESP_ERR_INVALID_ARG);
  assert(reason == ESP_ANGLE_LUT_INSTALL_FAILURE_METADATA);
  assert(esp_angle_lut_instance_install(l, table, 256, 16384, 123) ==
         ESP_ERR_INVALID_CRC);
  table[0] = 1025;
  assert(esp_angle_lut_instance_install(
             l, table, 256, 16384, payload_crc(table)) == ESP_ERR_INVALID_ARG);
  table[0] = 100;
  assert(esp_angle_lut_instance_install(
             l, table, 256, 16384, payload_crc(table)) == ESP_ERR_INVALID_ARG);
  memset(table, 0, sizeof(table));
  /* Restart after failure at every operation. A selected new table must be
   * disabled, including selector-success / final-commit-failure scenarios. */
  for (int stage = 0; stage < 9; stage++) {
    install(l, 12);
    fail_after = stage;
    esp_err_t e = esp_angle_lut_instance_install(l, table, 256, 16384,
                                                 payload_crc(table));
    fail_after = -1;
    assert(esp_angle_lut_destroy(l) == ESP_OK);
    assert(esp_angle_lut_create("robot_left", &l) == ESP_OK);
    esp_angle_lut_status_t status;
    int16_t readback[256];
    assert(esp_angle_lut_instance_read(l, readback, 256, &status) == ESP_OK);
    if (readback[0] == 0)
      assert(!status.enabled);
    assert(readback[0] == 0 || readback[0] == 12);
    assert(esp_angle_lut_instance_apply(r, 0) == 16375);
    (void)e;
  }
  /* Disable persistence exposes the exact open/write/commit failure stage. */
  const esp_angle_lut_install_failure_t stages[] = {
      ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_OPEN,
      ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_ENABLED_WRITE,
      ESP_ANGLE_LUT_INSTALL_FAILURE_NVS_COMMIT};
  for (int i = 0; i < 3; ++i) {
    fail_after = i;
    assert(esp_angle_lut_instance_install_detailed(
               l, table, 256, 16384, payload_crc(table), &reason) == ESP_FAIL);
    assert(reason == stages[i]);
    fail_after = -1;
  }
  /* Corrupt selected slot: restore the other validated table. */
  install(l, 12);
  install(l, 20);
  nvs_handle_t n;
  assert(nvs_open("robot_left", 1, &n) == ESP_OK);
  store[n - 1].key[l->storage_slot].bytes[0] ^= 1;
  assert(esp_angle_lut_destroy(l) == ESP_OK);
  assert(esp_angle_lut_create("robot_left", &l) == ESP_OK);
  assert(esp_angle_lut_instance_apply(l, 0) == 12);
  assert(esp_angle_lut_instance_clear(l) == ESP_OK);
  assert(esp_angle_lut_instance_apply(l, 20) == 20);
  assert(esp_angle_lut_instance_apply(r, 0) == 16375);
  esp_angle_lut_handle_t bad;
  assert(esp_angle_lut_create("bad", &bad) == ESP_OK);
  install(bad, 5);
  install(bad, 8);
  assert(nvs_open("bad", 1, &n) == ESP_OK);
  store[n - 1].key[0].bytes[0] ^= 1;
  store[n - 1].key[1].bytes[0] ^= 1;
  assert(esp_angle_lut_destroy(bad) == ESP_OK);
  assert(esp_angle_lut_create("bad", &bad) == ESP_OK);
  esp_angle_lut_status_t bypass;
  esp_angle_lut_instance_get_status(bad, &bypass);
  assert(!bypass.loaded && !bypass.enabled);
  assert(esp_angle_lut_instance_apply(bad, 16383) == 16383);
  assert(esp_angle_lut_destroy(bad) == ESP_OK);
  assert(esp_angle_lut_init() == ESP_OK);
  assert(esp_angle_lut_init() == ESP_ERR_INVALID_STATE);
  assert(esp_angle_lut_install(table, 256, 16384, payload_crc(table)) ==
         ESP_OK);
  assert(esp_angle_lut_set_enabled(true) == ESP_OK);
  assert(esp_angle_lut_apply(123) == 123);
  assert(esp_angle_lut_instance_apply(r, 123) == 114);
  assert(esp_angle_lut_destroy(l) == ESP_OK);
  assert(esp_angle_lut_destroy(r) == ESP_OK);
  puts("LUT instances/persistence/failure/legacy tests passed");
}
