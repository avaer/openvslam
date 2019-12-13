#include <emscripten.h>
#include <stdlib.h>

#include <spdlog/spdlog.h>

EMSCRIPTEN_KEEPALIVE void doInit() {
  spdlog::set_level(spdlog::level::off);
}

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
  return malloc(size);
}
EMSCRIPTEN_KEEPALIVE void doFree(void *p) {
  free(p);
}