#ifndef SCAN_LOG_H
#define SCAN_LOG_H

#include <Arduino.h>

/**
 * @brief One logged sample: where the robot thought it was, and what the five
 * surface sensors read at that moment. The dashboard turns these into an image.
 *
 * x, y and theta come from the StampC3's own OdometryModel_c (mm, mm, rad).
 * dn[0..4] are DN1..DN5 exactly as the robot reported them (higher = darker).
 */
struct ScanSample_t {
  uint32_t t_ms;     // StampC3 millis() when the sample was logged
  float x;           // mm
  float y;           // mm
  float theta;       // rad
  uint16_t dn[5];    // DN1..DN5 raw readings
  uint8_t state;     // Controller_c::Mode value at the time (see Controller.h)
  uint8_t row;       // raster row index (0 = first row)
};

/**
 * @brief Fixed-size in-memory store of one scan run.
 *
 * Samples are kept until the next run starts so the dashboard can download a
 * complete run even if it was opened late or the WiFi dropped for a while.
 */
class ScanLog_c {
  public:
    ScanLog_c();

    /**
     * @brief Allocates storage, trying smaller sizes if memory is short.
     * @param requested_capacity Samples wanted.
     * @param keep_free_bytes Heap that must remain free afterwards.
     * @return The number of samples that fit (0 if allocation failed).
     */
    uint32_t begin(uint32_t requested_capacity, uint32_t keep_free_bytes);

    /** @brief Frees the storage (capacity becomes 0). */
    void release();

    /** @brief Forgets all samples (storage is kept). */
    void clear();

    /** @brief Appends one sample. Returns false when the log is full. */
    bool add(const ScanSample_t &sample);

    uint32_t count() const;
    uint32_t capacity() const;
    bool isFull() const;

    /** @brief Returns sample i (0 <= i < count()). */
    const ScanSample_t &get(uint32_t i) const;

  private:
    ScanSample_t *samples;
    uint32_t cap;
    uint32_t n;
};

#endif
