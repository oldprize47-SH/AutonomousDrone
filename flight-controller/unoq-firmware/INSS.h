//
// INSS - INS Sensor Driver (MTi-680G IMU/GNSS)
// Unified module: XBUS protocol, data structures, GNSS types, driver class
//

#ifndef INSS_H
#define INSS_H

#include <Arduino.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// GNSS Data Structures
// ============================================================

typedef struct {
    uint32_t nano;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  flags;
} UTCTime;

typedef struct {
    uint32_t itow;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t t_acc;
    int32_t  nano;
    uint8_t  fix_type;
    uint8_t  flags;
    uint8_t  num_sv;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  h_msl;
    uint32_t h_acc;
    uint32_t v_acc;
    int32_t  vel_n;
    int32_t  vel_e;
    int32_t  vel_d;
    int32_t  g_speed;
    int32_t  head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t gdop;
    uint16_t pdop;
    uint16_t tdop;
    uint16_t vdop;
    uint16_t hdop;
    uint16_t ndop;
    uint16_t edop;
} GnssPvtData;

// ============================================================
// XBUS Protocol Constants
// ============================================================

#define MID_GOTO_CONFIG            0x30
#define MID_GOTO_CONFIG_ACK        0x31
#define MID_GOTO_MEASUREMENT       0x10
#define MID_GOTO_MEASURE_ACK       0x11
#define MID_SET_OUTPUT_CONFIG      0xC0
#define MID_SET_OUTPUT_ACK         0xC1
#define MID_SET_BAUDRATE           0x18
#define MID_SET_BAUDRATE_ACK       0x19
#define MID_MTDATA2                0x36
#define MID_SET_GNSS_LEVER_ARM     0x68
#define MID_SET_GNSS_LEVER_ARM_ACK 0x69
#define MID_SET_PORT_CONFIG        0xC4
#define MID_SET_PORT_CONFIG_ACK    0xC5
#define MID_FORWARD_GNSS_DATA      0xE2
#define MID_FORWARD_GNSS_DATA_ACK  0xE3

#define XDI_TEMPERATURE        ((uint16_t)0x0810)
#define XDI_UTC_TIME           ((uint16_t)0x1010)
#define XDI_PACKET_COUNTER     ((uint16_t)0x1020)
#define XDI_SAMPLE_TIME_FINE   ((uint16_t)0x1060)
#define XDI_EULER_ANGLES       ((uint16_t)0x2030)
#define XDI_BARO_PRESSURE      ((uint16_t)0x3010)
#define XDI_ACCELERATION       ((uint16_t)0x4020)
#define XDI_FREE_ACCELERATION  ((uint16_t)0x4030)
#define XDI_LATLON             ((uint16_t)0x5040)
#define XDI_ALT_ELLIPSOID      ((uint16_t)0x5020)
#define XDI_ALT_MSL            ((uint16_t)0x5010)
#define XDI_GNSS_PVT_DATA      ((uint16_t)0x7010)
#define XDI_GNSS_SAT_INFO      ((uint16_t)0x7020)
#define XDI_RATE_OF_TURN       ((uint16_t)0x8020)
#define XDI_MAGNETIC_FIELD     ((uint16_t)0xC020)
#define XDI_VELOCITY_XYZ       ((uint16_t)0xD010)
#define XDI_STATUS_WORD        ((uint16_t)0xE020)

#define STATUS_FILTER_VALID         (1u << 1)
#define STATUS_GNSS_FIX             (1u << 2)
#define STATUS_ROTATION_STATE_MASK  (0x3u << 3)
#define STATUS_ROTATION_STATE_SHIFT 3
#define STATUS_RTK_MASK             (0x3u << 27)
#define STATUS_RTK_SHIFT            27

#define RTK_STATUS_NONE     0
#define RTK_STATUS_FLOATING 1
#define RTK_STATUS_FIXED    2

#define MDATA2_MAX_PAYLOAD 2048

typedef struct { uint8_t BID; uint8_t MID; uint8_t LEN; } XbusHeader;

typedef union {
    float f; double d; uint32_t u32; int32_t i32; uint16_t u16; uint8_t byte[8];
} ByteData680G;

typedef enum { MTI_OK, MTI_SERIAL_FAIL, MTI_TIMEOUT, MTI_CONFIG_ERROR } MtiError;

typedef enum {
    PARSE_WAIT_PREAMBLE, PARSE_WAIT_BID, PARSE_WAIT_MID, PARSE_WAIT_LEN,
    PARSE_WAIT_EXT_LEN_H, PARSE_WAIT_EXT_LEN_L, PARSE_WAIT_PAYLOAD, PARSE_WAIT_CHECKSUM
} XbusParseState;

typedef struct {
    XbusParseState state;
    uint8_t  bid, mid, len_byte;
    uint16_t payload_len, payload_idx;
    uint8_t  payload[MDATA2_MAX_PAYLOAD];
    uint8_t  checksum;
} XbusParser;

// ============================================================
// MTi Data Structure
// ============================================================

// All sensor outputs are converted to NED frame at parse time.
// Euler: Roll(+right), Pitch(+nose down), Yaw(+CW from North, 0-360)
// Acc/FreeAcc/Gyro: X=forward, Y=right, Z=down
// Velocity: X=North, Y=East, Z=Down
enum MtiDataIndex {
    ROLL = 0, PITCH, YAW,
    ACC_X, ACC_Y, ACC_Z,
    FREE_ACC_X, FREE_ACC_Y, FREE_ACC_Z,
    GYR_X, GYR_Y, GYR_Z,
    MAG_X, MAG_Y, MAG_Z,
    TEMPERATURE, PRESSURE, SAMPLE_TIME,
    LATITUDE, LONGITUDE, ALT_ELLIPSOID, ALT_MSL,
    VEL_X, VEL_Y, VEL_Z,
    STATUS_WORD, GNSS_FIX, RTK_STATUS, GNSS_NUM_SV,
    DATA_COUNT
};

#define HAS_PACKET_COUNT      (1u << 0)
#define HAS_UTC_TIME          (1u << 1)
#define HAS_EULER             (1u << 2)
#define HAS_ACCELERATION      (1u << 3)
#define HAS_FREE_ACCELERATION (1u << 4)
#define HAS_GYRO              (1u << 6)
#define HAS_MAGNETIC          (1u << 7)
#define HAS_PRESSURE          (1u << 8)
#define HAS_LATLON            (1u << 9)
#define HAS_ALT_ELLIPSOID     (1u << 10)
#define HAS_ALT_MSL           (1u << 11)
#define HAS_VELOCITY_XYZ      (1u << 12)
#define HAS_GNSS_PVT          (1u << 13)
#define HAS_STATUS_WORD       (1u << 14)
#define HAS_SAMPLE_TIME       (1u << 15)
#define HAS_TEMPERATURE       (1u << 16)

typedef struct {
    double      data[DATA_COUNT];
    uint32_t    data_available;
    uint16_t    packet_count;
    UTCTime     utc_time;
    GnssPvtData gnss_pvt;
} MtiData;

#ifdef __cplusplus
}
#endif

// ============================================================
// INSS Driver Class
// ============================================================

#ifdef __cplusplus

class Usart1Serial;

class INSS {
public:
    INSS();

    bool begin(Usart1Serial &serial, uint32_t baud = 115200);
    void update();
    MtiData getData();
    bool available();
    bool isRunning() const { return _running; }
    MtiError forwardRtcm(const uint8_t *data, uint16_t len);
    Usart1Serial* getSerial() { return _serial; }
    void end();

private:
    Usart1Serial *_serial;
    bool _running;
public:
    int  _flush_count;   // bytes flushed during begin()
    int  _fail_step;     // 0=ok, 1=GoToConfig, 2=SetOutput, 3=GoToMeasure
private:

    XbusParser _parser;
    MtiData _data;
    volatile bool _data_new;

    void parserReset();
    bool parserFeedByte(uint8_t b);
    void processPacket();

    bool sendCommand(const uint8_t *cmd, int len);
    bool waitForAck(uint8_t expected_mid, uint32_t timeout_ms = 2000);
    bool waitForAckWithPayload(uint8_t expected_mid, uint8_t *out_payload,
                               uint16_t *out_len, uint32_t timeout_ms = 2000);
    static uint8_t calcChecksum(const uint8_t *data, int len);

    static void decodeMData2(const uint8_t *payload, uint16_t len, MtiData *out);
    static void decodeField(uint16_t xdi, const uint8_t *payload, uint8_t len, MtiData *out);

    static float    bytesToFloat(const uint8_t b[4]);
    static double   bytesToDouble(const uint8_t b[8]);
    static double   bytesToFP1632(const uint8_t b[6]);
    static uint16_t bytesToU16(const uint8_t b[2]);
    static uint32_t bytesToU32(const uint8_t b[4]);
    static int32_t  bytesToI32(const uint8_t b[4]);
};

#endif // __cplusplus

#endif // INSS_H
