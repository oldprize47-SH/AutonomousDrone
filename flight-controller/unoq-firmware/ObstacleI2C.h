//
// ObstacleI2C - shared H7 <-> UNO Q obstacle packet layout (32 bytes)
//
// This is the byte-for-byte shared section copied from the H7 firmware's
// i2c_obstacle.h. BOTH sides must use identical struct layout / CRC so the
// 32-byte frame decodes correctly. Only the H7-specific publisher API lives
// on the H7 side; the UNO Q only needs the layout + decode helpers below.
//
// Bus: UNO Q QWIIC = Wire1 (I2C4). UNO Q is master and polls:
//   - TF-Nova  0x10  (altitude, existing)
//   - H7       0x20  (obstacles, this packet)
//

#ifndef OBSTACLE_I2C_H
#define OBSTACLE_I2C_H

#include <stdint.h>

// Max obstacles per packet (count field is 3 bits -> <= 7; 32-byte cap -> 5).
#ifndef MAX_OBS
#define MAX_OBS 5
#endif

// H7 obstacle slave address on the QWIIC bus (must match the H7 firmware).
#ifndef H7_OBSTACLE_I2C_ADDR
#define H7_OBSTACLE_I2C_ADDR 0x20
#endif

// One obstacle = 6 bytes.
struct __attribute__((packed)) ObstacleMsg {
    uint8_t  id;             // track_id (0 = empty slot)
    int16_t  center_cdeg;    // center angle, 0.1deg units, signed about forward [-1800,1800]
    uint8_t  half_span_hdeg; // half-width, 0.5deg units (0..180 => 0..90deg)
    uint16_t dist_mm;        // min distance, mm
};

// Packet = 1 (hdr) + 6*MAX_OBS + 1 (crc8) = 32 bytes for MAX_OBS=5.
//   hdr : [7:3] seq(0..31, stale detect), [2:0] count(0..MAX_OBS)
//   crc8: CRC8 over hdr..obs[] (poly 0x07, init 0x00)
struct __attribute__((packed)) ObstaclePacket {
    uint8_t     hdr;
    ObstacleMsg obs[MAX_OBS];
    uint8_t     crc8;
};

static_assert(MAX_OBS <= 7, "count field is 3 bits: MAX_OBS must be <= 7");
static_assert(sizeof(ObstaclePacket) <= 32,
              "I2C packet exceeds 32-byte Wire buffer; lower MAX_OBS");

// hdr helpers (shared)
static inline uint8_t i2cObsHdr(uint8_t seq, uint8_t count) {
    return (uint8_t)(((seq & 0x1F) << 3) | (count & 0x07));
}
static inline uint8_t i2cObsHdrSeq(uint8_t hdr)   { return (uint8_t)(hdr >> 3); }
static inline uint8_t i2cObsHdrCount(uint8_t hdr) { return (uint8_t)(hdr & 0x07); }

// CRC8 (poly 0x07, init 0x00) — shared.
static inline uint8_t i2cObstacleCrc8(const uint8_t* d, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; ++i) {
        c ^= d[i];
        for (int b = 0; b < 8; ++b) {
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
    }
    return c;
}

#endif // OBSTACLE_I2C_H
