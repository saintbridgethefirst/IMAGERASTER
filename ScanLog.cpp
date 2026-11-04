#include "ScanLog.h"

ScanLog_c::ScanLog_c() : samples(nullptr), cap(0), n(0) {
}

uint32_t ScanLog_c::begin(uint32_t requested_capacity, uint32_t keep_free_bytes) {
  release();

  // Take the largest log that fits in one block while still leaving
  // keep_free_bytes of heap for WiFi and the web server. Step down in small
  // amounts so a slightly-too-big request doesn't lose half the log.
  uint32_t want = requested_capacity;
  while (want >= 250) {
    size_t bytes = (size_t)want * sizeof(ScanSample_t);
    if (ESP.getMaxAllocHeap() >= bytes && ESP.getFreeHeap() >= bytes + keep_free_bytes) {
      samples = (ScanSample_t *)malloc(bytes);
      if (samples != nullptr) {
        cap = want;
        break;
      }
    }
    want -= 250;
  }
  return cap;
}

void ScanLog_c::release() {
  if (samples != nullptr) {
    free(samples);
    samples = nullptr;
  }
  cap = 0;
  n = 0;
}

void ScanLog_c::clear() {
  n = 0;
}

bool ScanLog_c::add(const ScanSample_t &sample) {
  if (samples == nullptr || n >= cap) {
    return false;
  }
  samples[n++] = sample;
  return true;
}

uint32_t ScanLog_c::count() const {
  return n;
}

uint32_t ScanLog_c::capacity() const {
  return cap;
}

bool ScanLog_c::isFull() const {
  return n >= cap;
}

const ScanSample_t &ScanLog_c::get(uint32_t i) const {
  return samples[i];
}
