//
// INSS - INS Sensor Driver Implementation (MTi-680G)
//

#include "INSS.h"
#include "Usart1Serial.h"
#include "hw_config.h"
#include <string.h>

float INSS::bytesToFloat(const uint8_t b[4])
{
    ByteData680G d;
    d.byte[3] = b[0]; d.byte[2] = b[1];
    d.byte[1] = b[2]; d.byte[0] = b[3];
    return d.f;
}

double INSS::bytesToDouble(const uint8_t b[8])
{
    ByteData680G d;
    d.byte[7] = b[0]; d.byte[6] = b[1];
    d.byte[5] = b[2]; d.byte[4] = b[3];
    d.byte[3] = b[4]; d.byte[2] = b[5];
    d.byte[1] = b[6]; d.byte[0] = b[7];
    return d.d;
}

double INSS::bytesToFP1632(const uint8_t b[6])
{
    uint32_t fracPart = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                        ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    int16_t  intPart  = (int16_t)(((uint16_t)b[4] << 8) | (uint16_t)b[5]);
    return (double)intPart + (double)fracPart / 4294967296.0;
}

uint16_t INSS::bytesToU16(const uint8_t b[2])
{
    return ((uint16_t)b[0] << 8) | (uint16_t)b[1];
}

uint32_t INSS::bytesToU32(const uint8_t b[4])
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
}

int32_t INSS::bytesToI32(const uint8_t b[4])
{
    return (int32_t)bytesToU32(b);
}

INSS::INSS()
    : _serial(nullptr), _running(false), _data_new(false)
{
    memset(&_data, 0, sizeof(_data));
    parserReset();
}

void INSS::parserReset()
{
    _parser.state = PARSE_WAIT_PREAMBLE;
    _parser.payload_idx = 0;
    _parser.checksum = 0;
}

bool INSS::parserFeedByte(uint8_t b)
{
    switch (_parser.state)
    {
    case PARSE_WAIT_PREAMBLE:
        if (b == 0xFA) _parser.state = PARSE_WAIT_BID;
        return false;
    case PARSE_WAIT_BID:
        if (b == 0xFF) {
            _parser.bid = b; _parser.checksum = b;
            _parser.state = PARSE_WAIT_MID;
        } else { parserReset(); }
        return false;
    case PARSE_WAIT_MID:
        _parser.mid = b; _parser.checksum += b;
        _parser.state = PARSE_WAIT_LEN;
        return false;
    case PARSE_WAIT_LEN:
        _parser.len_byte = b; _parser.checksum += b;
        if (b == 0xFF) {
            _parser.state = PARSE_WAIT_EXT_LEN_H;
        } else {
            _parser.payload_len = b; _parser.payload_idx = 0;
            _parser.state = (b == 0) ? PARSE_WAIT_CHECKSUM : PARSE_WAIT_PAYLOAD;
        }
        return false;
    case PARSE_WAIT_EXT_LEN_H:
        _parser.payload_len = (uint16_t)b << 8;
        _parser.checksum += b;
        _parser.state = PARSE_WAIT_EXT_LEN_L;
        return false;
    case PARSE_WAIT_EXT_LEN_L:
        _parser.payload_len |= b; _parser.checksum += b;
        _parser.payload_idx = 0;
        if (_parser.payload_len > MDATA2_MAX_PAYLOAD) { parserReset(); return false; }
        _parser.state = (_parser.payload_len == 0) ? PARSE_WAIT_CHECKSUM : PARSE_WAIT_PAYLOAD;
        return false;
    case PARSE_WAIT_PAYLOAD:
        _parser.payload[_parser.payload_idx++] = b;
        _parser.checksum += b;
        if (_parser.payload_idx >= _parser.payload_len) _parser.state = PARSE_WAIT_CHECKSUM;
        return false;
    case PARSE_WAIT_CHECKSUM:
        _parser.checksum += b;
        if (_parser.checksum == 0) { processPacket(); parserReset(); return true; }
        parserReset();
        return false;
    }
    parserReset();
    return false;
}

void INSS::decodeField(uint16_t xdi, const uint8_t *payload,
                       uint8_t len, MtiData *out)
{
    uint16_t xdi_base = xdi & 0xFFF0;
    switch (xdi_base)
    {
    case 0x0810:
        if (len >= 4) { out->data[TEMPERATURE] = (double)bytesToFloat(payload); out->data_available |= HAS_TEMPERATURE; }
        break;
    case 0x1020:
        if (len >= 2) { out->packet_count = bytesToU16(payload); out->data_available |= HAS_PACKET_COUNT; }
        break;
    case 0x1060:
        if (len >= 4) { out->data[SAMPLE_TIME] = (double)bytesToU32(payload); out->data_available |= HAS_SAMPLE_TIME; }
        break;
    case 0x1010:
        if (len >= 12) {
            out->utc_time.nano = bytesToU32(&payload[0]); out->utc_time.year = bytesToU16(&payload[4]);
            out->utc_time.month = payload[6]; out->utc_time.day = payload[7];
            out->utc_time.hour = payload[8]; out->utc_time.minute = payload[9];
            out->utc_time.second = payload[10]; out->utc_time.flags = payload[11];
            out->data_available |= HAS_UTC_TIME;
        }
        break;
    case 0x2030:  // Euler Angles (sign matched to NED by flight test)
        if (len >= 12) {
            out->data[ROLL]  =  (double)bytesToFloat(&payload[0]);
            out->data[PITCH] = -(double)bytesToFloat(&payload[4]);
            double yaw_enu   =  (double)bytesToFloat(&payload[8]);
            double yaw_ned   = 90.0 - yaw_enu;
            if (yaw_ned < 0.0)   yaw_ned += 360.0;
            if (yaw_ned >= 360.0) yaw_ned -= 360.0;
            out->data[YAW] = yaw_ned;
            out->data_available |= HAS_EULER;
        }
        break;
    case 0x4020:  // Acceleration: SENSOR(body) frame, NOT earth-frame.
        // 0x4020 is body-fixed (includes gravity). This negates Y,Z to align the
        // Xsens sensor axes with the vehicle body frame (FRD: Forward-Right-Down).
        // It is NOT an ENU->NED earth-frame conversion - ACC_X/Y/Z are body axes,
        // not North/East/Down. (Contrast 0x4030 Free Acc / 0xD010 Velocity, which
        // ARE earth-frame and get a true N/E swap.)
        if (len >= 12) {
            out->data[ACC_X] =  (double)bytesToFloat(&payload[0]);
            out->data[ACC_Y] = -(double)bytesToFloat(&payload[4]);
            out->data[ACC_Z] = -(double)bytesToFloat(&payload[8]);
            out->data_available |= HAS_ACCELERATION;
        }
        break;
    case 0x4030:  // Free Acceleration: ENU(E,N,U) -> NED(N,E,D)
        // payload is [X=East, Y=North, Z=Up] per Xsens output spec (default ENU),
        // identical axis order to Velocity 0xD010. Convert to true NED so the
        // horizontal components match VEL_X/Y (used together by the velocity
        // controller's yaw rotation). Z stays down+ (-Up), as before.
        if (len >= 12) {
            double fa_e = (double)bytesToFloat(&payload[0]);  // East
            double fa_n = (double)bytesToFloat(&payload[4]);  // North
            double fa_u = (double)bytesToFloat(&payload[8]);  // Up
            out->data[FREE_ACC_X] =  fa_n;   // North
            out->data[FREE_ACC_Y] =  fa_e;   // East
            out->data[FREE_ACC_Z] = -fa_u;   // Down
            out->data_available |= HAS_FREE_ACCELERATION;
        }
        break;
    case 0x8020:  // Rate of Turn (gyro): SENSOR(body) frame, NOT earth-frame.
        // Body angular rate. Y,Z negated to align Xsens sensor axes with the
        // vehicle body frame (FRD). GYR_X/Y/Z are body roll/pitch/yaw rates,
        // not earth-frame - no N/E/D meaning. Used directly by the attitude loop.
        if (len >= 12) {
            out->data[GYR_X] =  (double)bytesToFloat(&payload[0]);
            out->data[GYR_Y] = -(double)bytesToFloat(&payload[4]);
            out->data[GYR_Z] = -(double)bytesToFloat(&payload[8]);
            out->data_available |= HAS_GYRO;
        }
        break;
    case 0xC020:  // Magnetic field: SENSOR(body) frame, stored raw (no negate).
        // Body-frame magnetic field, normalized units. Kept as raw sensor axes
        // (currently unused by the flight code). NOT earth-frame.
        if (len >= 12) {
            out->data[MAG_X] = (double)bytesToFloat(&payload[0]);
            out->data[MAG_Y] = (double)bytesToFloat(&payload[4]);
            out->data[MAG_Z] = (double)bytesToFloat(&payload[8]);
            out->data_available |= HAS_MAGNETIC;
        }
        break;
    case 0x3010:
        if (len >= 4) { out->data[PRESSURE] = (double)bytesToU32(payload); out->data_available |= HAS_PRESSURE; }
        break;
    case 0x5040:
        if ((xdi & 0x0003) == 0x0000 && len >= 8) {
            out->data[LATITUDE] = (double)bytesToFloat(&payload[0]);
            out->data[LONGITUDE] = (double)bytesToFloat(&payload[4]);
            out->data_available |= HAS_LATLON;
        } else if ((xdi & 0x0003) == 0x0002 && len >= 12) {
            out->data[LATITUDE] = bytesToFP1632(&payload[0]);
            out->data[LONGITUDE] = bytesToFP1632(&payload[6]);
            out->data_available |= HAS_LATLON;
        } else if ((xdi & 0x0003) == 0x0003 && len >= 16) {
            out->data[LATITUDE] = bytesToDouble(&payload[0]);
            out->data[LONGITUDE] = bytesToDouble(&payload[8]);
            out->data_available |= HAS_LATLON;
        }
        break;
    case 0x5020:
        if (len >= 4) { out->data[ALT_ELLIPSOID] = (double)bytesToFloat(payload); out->data_available |= HAS_ALT_ELLIPSOID; }
        break;
    case 0x5010:
        if (len >= 4) { out->data[ALT_MSL] = (double)bytesToFloat(payload); out->data_available |= HAS_ALT_MSL; }
        break;
    case 0xD010:  // Velocity: ENU(E,N,U) -> NED(N,E,D)
        if (len >= 12) {
            double vel_e = (double)bytesToFloat(&payload[0]);
            double vel_n = (double)bytesToFloat(&payload[4]);
            double vel_u = (double)bytesToFloat(&payload[8]);
            out->data[VEL_X] = vel_n;   // North
            out->data[VEL_Y] = vel_e;   // East
            out->data[VEL_Z] = -vel_u;  // Down
            out->data_available |= HAS_VELOCITY_XYZ;
        }
        break;
    case 0x7010:
        if (len >= 72) {
            const uint8_t *p = payload;
            out->gnss_pvt.itow = bytesToU32(&p[0]); out->gnss_pvt.year = bytesToU16(&p[4]);
            out->gnss_pvt.month = p[6]; out->gnss_pvt.day = p[7];
            out->gnss_pvt.hour = p[8]; out->gnss_pvt.min = p[9];
            out->gnss_pvt.sec = p[10]; out->gnss_pvt.valid = p[11];
            out->gnss_pvt.t_acc = bytesToU32(&p[12]); out->gnss_pvt.nano = bytesToI32(&p[16]);
            out->gnss_pvt.fix_type = p[20]; out->gnss_pvt.flags = p[21]; out->gnss_pvt.num_sv = p[22];
            out->gnss_pvt.lon = bytesToI32(&p[24]); out->gnss_pvt.lat = bytesToI32(&p[28]);
            out->gnss_pvt.height = bytesToI32(&p[32]); out->gnss_pvt.h_msl = bytesToI32(&p[36]);
            out->gnss_pvt.h_acc = bytesToU32(&p[40]); out->gnss_pvt.v_acc = bytesToU32(&p[44]);
            out->gnss_pvt.vel_n = bytesToI32(&p[48]); out->gnss_pvt.vel_e = bytesToI32(&p[52]);
            out->gnss_pvt.vel_d = bytesToI32(&p[56]); out->gnss_pvt.g_speed = bytesToI32(&p[60]);
            out->gnss_pvt.head_mot = bytesToI32(&p[64]); out->gnss_pvt.s_acc = bytesToU32(&p[68]);
            if (len >= 76) out->gnss_pvt.head_acc = bytesToU32(&p[72]);
            if (len >= 78) out->gnss_pvt.gdop = bytesToU16(&p[76]);
            if (len >= 80) out->gnss_pvt.pdop = bytesToU16(&p[78]);
            if (len >= 82) out->gnss_pvt.tdop = bytesToU16(&p[80]);
            if (len >= 84) out->gnss_pvt.vdop = bytesToU16(&p[82]);
            if (len >= 86) out->gnss_pvt.hdop = bytesToU16(&p[84]);
            if (len >= 88) out->gnss_pvt.ndop = bytesToU16(&p[86]);
            if (len >= 90) out->gnss_pvt.edop = bytesToU16(&p[88]);
            out->data[GNSS_NUM_SV] = (double)out->gnss_pvt.num_sv;
            out->data_available |= HAS_GNSS_PVT;
        }
        break;
    case 0xE020:
        if (len >= 4) {
            uint32_t sw = bytesToU32(payload);
            out->data[STATUS_WORD] = (double)sw;
            out->data[GNSS_FIX] = (double)((sw & STATUS_GNSS_FIX) ? 1 : 0);
            out->data[RTK_STATUS] = (double)((sw & STATUS_RTK_MASK) >> STATUS_RTK_SHIFT);
            out->data_available |= HAS_STATUS_WORD;
        }
        break;
    default: break;
    }
}

void INSS::decodeMData2(const uint8_t *payload, uint16_t total_len, MtiData *out)
{
    uint16_t offset = 0;
    while (offset + 3 <= total_len) {
        uint16_t xdi = bytesToU16(&payload[offset]);
        uint8_t field_len = payload[offset + 2];
        offset += 3;
        if (offset + field_len > total_len) break;
        decodeField(xdi, &payload[offset], field_len, out);
        offset += field_len;
    }
}

void INSS::processPacket()
{
    if (_parser.mid == MID_MTDATA2) {
        // Reset RTK status before decode so it reflects current packet
        _data.data[RTK_STATUS] = 0;
        _data.data[GNSS_FIX] = 0;
        decodeMData2(_parser.payload, _parser.payload_len, &_data);
        _data_new = true;
    }
}

uint8_t INSS::calcChecksum(const uint8_t *data, int len)
{
    uint8_t cs = 0;
    for (int i = 1; i < len - 1; i++) cs -= data[i];
    return cs;
}

bool INSS::sendCommand(const uint8_t *cmd, int len)
{
    return _serial->write(cmd, len) == (size_t)len;
}

bool INSS::waitForAck(uint8_t expected_mid, uint32_t timeout_ms)
{
    uint8_t dummy_payload[256]; uint16_t dummy_len;
    return waitForAckWithPayload(expected_mid, dummy_payload, &dummy_len, timeout_ms);
}

bool INSS::waitForAckWithPayload(uint8_t expected_mid, uint8_t *out_payload,
                                 uint16_t *out_len, uint32_t timeout_ms)
{
    XbusParser ack_parser;
    memset(&ack_parser, 0, sizeof(ack_parser));
    ack_parser.state = PARSE_WAIT_PREAMBLE;
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        if (_serial->available()) {
            uint8_t b = _serial->read();
            bool packet_complete = false;
            switch (ack_parser.state) {
            case PARSE_WAIT_PREAMBLE: if (b == 0xFA) ack_parser.state = PARSE_WAIT_BID; break;
            case PARSE_WAIT_BID:
                if (b == 0xFF) { ack_parser.bid = b; ack_parser.checksum = b; ack_parser.state = PARSE_WAIT_MID; }
                else ack_parser.state = PARSE_WAIT_PREAMBLE;
                break;
            case PARSE_WAIT_MID: ack_parser.mid = b; ack_parser.checksum += b; ack_parser.state = PARSE_WAIT_LEN; break;
            case PARSE_WAIT_LEN:
                ack_parser.len_byte = b; ack_parser.checksum += b;
                if (b == 0xFF) { ack_parser.state = PARSE_WAIT_EXT_LEN_H; }
                else { ack_parser.payload_len = b; ack_parser.payload_idx = 0;
                       ack_parser.state = (b == 0) ? PARSE_WAIT_CHECKSUM : PARSE_WAIT_PAYLOAD; }
                break;
            case PARSE_WAIT_EXT_LEN_H: ack_parser.payload_len = (uint16_t)b << 8; ack_parser.checksum += b; ack_parser.state = PARSE_WAIT_EXT_LEN_L; break;
            case PARSE_WAIT_EXT_LEN_L:
                ack_parser.payload_len |= b; ack_parser.checksum += b; ack_parser.payload_idx = 0;
                if (ack_parser.payload_len > sizeof(ack_parser.payload)) ack_parser.state = PARSE_WAIT_PREAMBLE;
                else ack_parser.state = (ack_parser.payload_len == 0) ? PARSE_WAIT_CHECKSUM : PARSE_WAIT_PAYLOAD;
                break;
            case PARSE_WAIT_PAYLOAD:
                ack_parser.payload[ack_parser.payload_idx++] = b; ack_parser.checksum += b;
                if (ack_parser.payload_idx >= ack_parser.payload_len) ack_parser.state = PARSE_WAIT_CHECKSUM;
                break;
            case PARSE_WAIT_CHECKSUM:
                ack_parser.checksum += b;
                if (ack_parser.checksum == 0) packet_complete = true;
                ack_parser.state = PARSE_WAIT_PREAMBLE; ack_parser.payload_idx = 0; ack_parser.checksum = 0;
                break;
            }
            if (packet_complete && ack_parser.mid == expected_mid) {
                if (out_payload && out_len) { memcpy(out_payload, ack_parser.payload, ack_parser.payload_len); *out_len = ack_parser.payload_len; }
                return true;
            }
            if (packet_complete) ack_parser.state = PARSE_WAIT_PREAMBLE;
        }
    }
    return false;
}

static const uint8_t CMD_GOTO_CONFIG[5] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
static const uint8_t CMD_GOTO_MEASUREMENT[5] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};

//#define NUM_OUTPUT_ENTRIES  16
//#define CONFIG_PAYLOAD_SIZE (NUM_OUTPUT_ENTRIES * 4)
//#define CONFIG_CMD_SIZE     (4 + CONFIG_PAYLOAD_SIZE + 1)

#define NUM_OUTPUT_ENTRIES  9
#define CONFIG_PAYLOAD_SIZE (NUM_OUTPUT_ENTRIES * 4)
#define CONFIG_CMD_SIZE     (4 + CONFIG_PAYLOAD_SIZE + 1)

static uint8_t build_set_output_config(uint8_t *cmd)
{
    //static const uint8_t template_cmd[CONFIG_CMD_SIZE] = {
    //    0xFA, 0xFF, 0xC0, CONFIG_PAYLOAD_SIZE,
    //    0x08, 0x10, 0x00, 0x64, 0x10, 0x20, 0xFF, 0xFF,
    //    0x10, 0x60, 0xFF, 0xFF, 0x10, 0x10, 0xFF, 0xFF,
    //    0x20, 0x30, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64,
    //    0x40, 0x30, 0x00, 0x64, 0x40, 0x10, 0x00, 0x64,
    //    0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64,
    //    0x30, 0x10, 0x00, 0x64, 0x50, 0x42, 0x00, 0x04,
    //    0x50, 0x20, 0x00, 0x04, 0xD0, 0x10, 0x00, 0x64,
    //    0x70, 0x10, 0x00, 0x04, 0xE0, 0x20, 0xFF, 0xFF,
    //    0x00
    //};
	static const uint8_t template_cmd[] = {

    	0xFA, 0xFF, 0xC0, CONFIG_PAYLOAD_SIZE,

    	// Euler 200Hz
    	0x20, 0x30, 0x00, 0xC8,

    	// Acceleration 200Hz
    	0x40, 0x20, 0x00, 0xC8,

    	// Free Acceleration 200Hz
    	0x40, 0x30, 0x00, 0xC8,

    	// Gyroscope 200Hz
    	0x80, 0x20, 0x00, 0xC8,

    	// Pressure 50Hz
    	0x30, 0x10, 0x00, 0x32,

    	// Altitude Ellipsoid (XKF3 fusion, RTK-precise) 50Hz
    	0x50, 0x20, 0x00, 0x32,

    	// Velocity XYZ 50Hz
    	0xD0, 0x10, 0x00, 0x32,

    	// GNSS PVT 4Hz
    	0x70, 0x10, 0x00, 0x04,

    	// Status Word
   		0xE0, 0x20, 0xFF, 0xFF,

    	0x00
	};

    memcpy(cmd, template_cmd, CONFIG_CMD_SIZE);
    uint8_t cs = 0;
    for (int i = 1; i < CONFIG_CMD_SIZE - 1; i++) cs -= cmd[i];
    cmd[CONFIG_CMD_SIZE - 1] = cs;
    return CONFIG_CMD_SIZE;
}

bool INSS::begin(Usart1Serial &serial, uint32_t baud)
{
    _serial = &serial;
    delay(100);

    uint32_t flush_start = millis();
    _flush_count = 0;
    while (millis() - flush_start < 500) {
        while (_serial->available()) { _serial->read(); _flush_count++; }
        delay(10);
    }

    bool config_ok = false;
    _fail_step = 1;  // GoToConfig
    for (int attempt = 0; attempt < 5; attempt++) {
        while (_serial->available()) { _serial->read(); }
        if (!sendCommand(CMD_GOTO_CONFIG, 5)) continue;
        if (waitForAck(MID_GOTO_CONFIG_ACK, 1000)) { config_ok = true; break; }
        delay(50);
    }
    if (!config_ok) return false;

    _fail_step = 2;  // SetOutputConfig
    uint8_t config_cmd[CONFIG_CMD_SIZE];
    build_set_output_config(config_cmd);
    if (!sendCommand(config_cmd, CONFIG_CMD_SIZE)) return false;
    { uint8_t resp[CONFIG_PAYLOAD_SIZE]; uint16_t resp_len;
      if (!waitForAckWithPayload(MID_SET_OUTPUT_ACK, resp, &resp_len, 3000)) return false;
    }
    _fail_step = 3;  // GoToMeasurement
    if (!sendCommand(CMD_GOTO_MEASUREMENT, 5)) return false;
    if (!waitForAck(MID_GOTO_MEASURE_ACK, 3000)) return false;

    _fail_step = 0;  // success
    _running = true;
    memset(&_data, 0, sizeof(_data));
    parserReset();
    return true;
}

void INSS::update()
{
    if (!_serial) return;
    while (_serial->available()) { uint8_t b = _serial->read(); parserFeedByte(b); }
}

MtiError INSS::forwardRtcm(const uint8_t *data, uint16_t len)
{
    if (!_serial || !data || len == 0) return MTI_CONFIG_ERROR;
    if (len <= 254) {
        uint8_t header[4] = { 0xFA, 0xFF, MID_FORWARD_GNSS_DATA, (uint8_t)len };
        uint8_t cs = 0xFF + MID_FORWARD_GNSS_DATA + (uint8_t)len;
        for (uint16_t i = 0; i < len; i++) cs += data[i];
        cs = (uint8_t)(-(int8_t)cs);
        _serial->write(header, 4); _serial->write(data, len); _serial->write(&cs, 1);
    } else {
        uint8_t header[6] = { 0xFA, 0xFF, MID_FORWARD_GNSS_DATA, 0xFF, (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
        uint8_t cs = 0xFF + MID_FORWARD_GNSS_DATA + 0xFF + (uint8_t)(len >> 8) + (uint8_t)(len & 0xFF);
        for (uint16_t i = 0; i < len; i++) cs += data[i];
        cs = (uint8_t)(-(int8_t)cs);
        _serial->write(header, 6); _serial->write(data, len); _serial->write(&cs, 1);
    }
    return MTI_OK;
}

MtiData INSS::getData() { _data_new = false; return _data; }
bool INSS::available() { return _data_new; }

void INSS::end()
{
    _running = false;
    _serial = nullptr;
}
