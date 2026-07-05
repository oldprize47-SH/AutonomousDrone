//
// LidarTF - Benewake TF-Nova LiDAR altimeter driver (I2C)
//
// The TF-Nova is configured in I2C mode (7-bit addr 0x10) and wired to the
// UNO Q QWIIC connector (= Wire1, I2C4: PD13 SDA / PD12 SCL).
//
// Distance register 0x00 (low) / 0x01 (high), little-endian, units cm.
// Invalid measurement returns 6554 (0x199A) or 65535 (0xFFFF); these and
// out-of-range values are gated out by read().
//

#ifndef LIDARTF_H
#define LIDARTF_H

#include <Arduino.h>
#include <Wire.h>

class LidarTF {
public:
    // Bind to an I2C bus. Caller is responsible for bus.begin()/setClock().
    bool begin(TwoWire& bus, uint8_t addr = 0x10);

    // Read one distance sample. Returns true and sets range_m on a valid
    // measurement (also updates lastRangeM()/lastMs()). On an invalid/stale
    // sample returns false and leaves the cached last-valid values untouched.
    // range_m has the ground-bias offset subtracted (see calibrateGround()).
    bool read(float& range_m);

    // Ground-bias zeroing: average `samples` valid reads taken `interval_ms`
    // apart and store the result as the zero offset, so a stationary craft on
    // the ground reports ~0 m. Blocking. Returns true if enough valid samples
    // were captured. Call once at setup, with the craft on the ground.
    bool calibrateGround(uint16_t samples = 20, uint16_t interval_ms = 20);

    void  setGroundOffset(float m) { _offset_m = m; }
    float groundOffset() const     { return _offset_m; }
    bool  calibrated()    const     { return _calibrated; }

    float    lastRangeM() const { return _last_m; }   // last valid range [m], offset applied
    uint32_t lastMs()     const { return _last_ms; }  // millis() of last valid

private:
    TwoWire* _bus       = nullptr;
    uint8_t  _addr      = 0x10;
    float    _last_m    = 0.0f;
    uint32_t _last_ms   = 0;
    float    _offset_m  = 0.0f;   // ground bias subtracted from every read
    bool     _calibrated = false;
};

#endif // LIDARTF_H
