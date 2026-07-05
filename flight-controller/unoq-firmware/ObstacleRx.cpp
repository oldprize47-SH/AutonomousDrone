//
// ObstacleRx - UNO Q I2C master reader for the H7 obstacle slave implementation
//

#include "ObstacleRx.h"
#include "hw_config.h"   // OBSTACLE_MAX_DIST_MM (distance gate)

bool ObstacleRx::begin(TwoWire& bus, uint8_t addr)
{
    _bus  = &bus;
    _addr = addr;
    _count = 0;
    _last_seq = 0;
    _last_ok_ms = 0;
    resetStats();
    return true;
}

void ObstacleRx::resetStats()
{
    _ok_count    = 0;
    _short_count = 0;
    _crc_count   = 0;
}

bool ObstacleRx::poll()
{
    if (_bus == nullptr) return false;

    // Request the full fixed-size packet. The H7 onRequest ISR always writes a
    // whole ObstaclePacket, so a short read means a bus/transfer problem.
    uint8_t buf[sizeof(ObstaclePacket)];
    int n = _bus->requestFrom((int)_addr, (int)sizeof(ObstaclePacket));
    if (n != (int)sizeof(ObstaclePacket)) {
        // Drain whatever partial bytes arrived so the next poll starts clean.
        while (_bus->available()) (void)_bus->read();
        _short_count++;
        return false;
    }

    for (int i = 0; i < (int)sizeof(ObstaclePacket); ++i) {
        buf[i] = (uint8_t)_bus->read();
    }

    const ObstaclePacket* pkt = (const ObstaclePacket*)buf;

    // CRC over hdr..obs[] (everything except the trailing crc8 byte).
    uint8_t crc = i2cObstacleCrc8(buf, sizeof(ObstaclePacket) - 1);
    if (crc != pkt->crc8) {
        _crc_count++;
        return false;
    }

    uint8_t raw_cnt = i2cObsHdrCount(pkt->hdr);
    if (raw_cnt > MAX_OBS) raw_cnt = MAX_OBS;   // defensive clamp

    // Decode into engineering units, dropping anything beyond the distance gate.
    // The H7 sends obstacles already converted to a body bearing (forward=0,
    // right+, left-), so center_cdeg maps straight to center_deg (no 0..360
    // wrap). out tracks how many survive the distance filter.
    uint8_t out = 0;
    for (uint8_t i = 0; i < raw_cnt; ++i) {
        const ObstacleMsg& m = pkt->obs[i];
        if (m.dist_mm > OBSTACLE_MAX_DIST_MM) continue;   // distance gate

        ObstacleDec& d = _obs[out];
        d.id = m.id;
        // center_cdeg is signed 0.1deg body bearing (-1800..1800 = -180..180).
        d.center_deg    = (float)m.center_cdeg * 0.1f;
        d.half_span_deg = (float)m.half_span_hdeg * 0.5f;
        // Integer tenths-of-degree mirrors for printing (Zephyr can't print float).
        d.center_deg10    = m.center_cdeg;                  // already 0.1deg, signed
        d.half_span_deg10 = (int16_t)((int)m.half_span_hdeg * 5);  // 0.5deg -> *10 = *5
        d.dist_mm       = m.dist_mm;
        out++;
    }

    _count      = out;
    _last_seq   = i2cObsHdrSeq(pkt->hdr);
    _last_ok_ms = millis();
    _ok_count++;
    return true;
}
