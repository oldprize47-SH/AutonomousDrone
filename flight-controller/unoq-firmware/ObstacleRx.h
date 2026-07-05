//
// ObstacleRx - UNO Q I2C master reader for the H7 obstacle slave (0x20)
//
// Polls the H7 over the QWIIC bus (Wire1), validates the 32-byte ObstaclePacket
// (length + CRC8), and decodes it into engineering units. Coexists with the
// TF-Nova altimeter on the same bus (different address, 0x10).
//
// Decoded units:
//   center_deg : 0..360 (forward = 0; converted back from signed 0.1deg)
//   half_span_deg : 0..90
//   dist_mm    : min distance to the obstacle
//

#ifndef OBSTACLE_RX_H
#define OBSTACLE_RX_H

#include <Arduino.h>
#include <Wire.h>
#include "ObstacleI2C.h"

// One decoded obstacle in engineering units.
//
// The H7 already converts the raw LiDAR angle into a drone-body bearing:
//   forward = 0 deg, right = +, left = - (range -180..180).
// So center_deg here is the collision-cone beta directly (no further mapping).
//
// Angles are kept BOTH as float (for guidance math) and as integer tenths of a
// degree (for printing). The UNO Q's Zephyr nano libc cannot print float (shows
// "ovf"), so debug output uses the *_deg10 integer fields. Guidance code should
// use the float fields.
struct ObstacleDec {
    uint8_t  id;             // track_id (0 = empty)
    float    center_deg;     // body bearing [deg], forward=0, right+, left- (-180..180)
    float    half_span_deg;  // half angular width [deg]
    int16_t  center_deg10;   // center_deg * 10 (signed integer, for printing)
    int16_t  half_span_deg10;// half_span_deg * 10 (integer, for printing)
    uint16_t dist_mm;        // min distance [mm]
};

class ObstacleRx {
public:
    // Bind to an I2C bus + H7 slave address. Caller owns bus.begin()/setClock().
    bool begin(TwoWire& bus, uint8_t addr = H7_OBSTACLE_I2C_ADDR);

    // Poll the H7: requestFrom(addr, 32), validate length + CRC, decode. Returns
    // true on a fresh, valid packet (also updates count()/obstacle()/lastSeq()).
    // On any failure (short read, CRC mismatch) returns false and leaves the
    // previous decoded obstacles untouched; the failure counters are bumped.
    bool poll();

    // Last successfully decoded packet.
    uint8_t      count()       const { return _count; }            // 0..MAX_OBS
    const ObstacleDec& obstacle(uint8_t i) const { return _obs[i]; } // i < count()
    uint8_t      lastSeq()     const { return _last_seq; }
    uint32_t     lastOkMs()    const { return _last_ok_ms; }

    // Diagnostics counters (since begin()).
    uint32_t okCount()    const { return _ok_count; }
    uint32_t shortCount() const { return _short_count; }  // requestFrom short read
    uint32_t crcCount()   const { return _crc_count; }    // CRC mismatch
    void     resetStats();

private:
    TwoWire* _bus  = nullptr;
    uint8_t  _addr = H7_OBSTACLE_I2C_ADDR;

    ObstacleDec _obs[MAX_OBS] = {};
    uint8_t     _count   = 0;
    uint8_t     _last_seq = 0;
    uint32_t    _last_ok_ms = 0;

    uint32_t _ok_count    = 0;
    uint32_t _short_count = 0;
    uint32_t _crc_count   = 0;
};

#endif // OBSTACLE_RX_H
