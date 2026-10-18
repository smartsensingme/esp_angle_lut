#include <stdlib.h>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
static inline void *heap_caps_calloc(size_t n, size_t s, int caps) {
  (void)caps;
  return calloc(n, s);
}
