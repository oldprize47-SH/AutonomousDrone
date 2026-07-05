//
// LidarTF - Benewake TF-Nova LiDAR altimeter driver (I2C) implementation
//

#include "LidarTF.h"
#include "hw_config.h"

bool LidarTF::begin(TwoWire& bus, uint8_t addr)
{
    _bus        = &bus;
    _addr       = addr;
    _last_m     = 0.0f;
    _last_ms    = 0;
    _offset_m   = 0.0f;
    _calibrated = false;
    return true;
}

// Read one raw distance sample [m] (no ground offset, no range gating, only
// the sensor's invalid-value gating). Returns true on a valid measurement.
static bool readRaw(TwoWire* bus, uint8_t addr, float& raw_m)
{
    if (bus == nullptr) return false;

    // Set register pointer to the distance low byte, then read 2 bytes.
    // (A bare read without the pointer write returns 0xFFFF on the TF-Nova.)
    bus->beginTransmission(addr);
    bus->write((uint8_t)LIDAR_REG_DIST);
    if (bus->endTransmission() != 0) return false;

    if (bus->requestFrom((int)addr, 2) != 2) return false;
    uint8_t lo = bus->read();
    uint8_t hi = bus->read();
    uint16_t raw_cm = (uint16_t)((hi << 8) | lo);

    // Gate out the sensor's "no valid measurement" sentinels.
    if (raw_cm == LIDAR_INVALID_FFFF || raw_cm == LIDAR_INVALID_6554)
        return false;

    raw_m = raw_cm / 100.0f;
    return true;
}

bool LidarTF::read(float& range_m)
{
    float raw_m;
    if (!readRaw(_bus, _addr, raw_m)) return false;

    // Range gating on the raw (unbiased) distance, so the configured window
    // matches the physical sensor range regardless of the ground offset.
    if (raw_m < LIDAR_ALT_MIN_M || raw_m > LIDAR_ALT_MAX_M)
        return false;

    // Subtract the ground bias so a craft sitting on the ground reads ~0 m.
    float m = raw_m - _offset_m;

    _last_m  = m;
    _last_ms = millis();
    range_m  = m;
    return true;
}

bool LidarTF::calibrateGround(uint16_t samples, uint16_t interval_ms)
{
    if (_bus == nullptr) return false;

    float    sum   = 0.0f;
    uint16_t count = 0;

    // Take up to (samples * 3) attempts so a few invalid reads don't starve us.
    uint16_t attempts = (uint16_t)(samples * 3);
    for (uint16_t i = 0; i < attempts && count < samples; i++) {
        float raw_m;
        if (readRaw(_bus, _addr, raw_m) &&
            raw_m >= LIDAR_ALT_MIN_M && raw_m <= LIDAR_ALT_MAX_M) {
            sum += raw_m;
            count++;
        }
        delay(interval_ms);
    }

    if (count == 0) {
        _calibrated = false;
        return false;
    }

    _offset_m   = sum / (float)count;   // average ground distance = zero point
    _calibrated = true;
    return true;
}
