#include <emscripten.h>
#include <stdlib.h>

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
  return malloc(size);
}
EMSCRIPTEN_KEEPALIVE void doFree(void *p) {
  free(p);
}