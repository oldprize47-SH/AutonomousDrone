//
// Telem - Telemetry Implementation
//

#include "Telem.h"
#include "INSS.h"
#include "QuadPWM.h"
#include "RCInput.h"
#include "hw_config.h"
#include <string.h>

// External references (defined in sketch.ino)
extern FlightMode current_mode;
extern Usart3Serial P8Serial;
extern INSS    ins;
extern QuadPWM motors;
extern bool     ins_active;
extern bool     motors_active;
extern uint16_t batt_main_mv;
extern uint16_t batt_sub_mv;
extern bool     rc_auto_inhibit;
extern bool     debug_output_enabled;
extern volatile bool emland_active;
extern volatile bool atl_takeoff_request;
extern volatile bool atl_land_request;
extern volatile bool atl_mission_request;

// RTCM forward counter
static uint16_t rtcm_fwd_count = 0;

// External references (defined in Mode.cpp)
extern int      pwm_selected_motor;
extern uint16_t pwm_motor_us[4];

// ============================================================
// TelemParser
// ============================================================

TelemParser::TelemParser() { reset(); }

void TelemParser::reset()
{
    _state = S_SYNC1;
    _idx = 0;
    _crc = 0;
    _cmd.cmd_id = 0;
    _cmd.payload_len = 0;
}

bool TelemParser::feedByte(uint8_t b)
{
    switch (_state) {
    case S_SYNC1:
        if (b == TELEM_SYNC1) _state = S_SYNC2;
        return false;
    case S_SYNC2:
        _state = (b == TELEM_SYNC2) ? S_CMD_ID : S_SYNC1;
        return false;
    case S_CMD_ID:
        _cmd.cmd_id = b;
        _crc = b;
        _state = S_LEN_H;
        return false;
    case S_LEN_H:
        _cmd.payload_len = (uint16_t)b << 8;
        _crc ^= b;
        _state = S_LEN_L;
        return false;
    case S_LEN_L:
        _cmd.payload_len |= b;
        _crc ^= b;
        _idx = 0;
        if (_cmd.payload_len > TELEM_MAX_PAYLOAD) { reset(); return false; }
        _state = (_cmd.payload_len == 0) ? S_CRC : S_PAYLOAD;
        return false;
    case S_PAYLOAD:
        _cmd.payload[_idx++] = b;
        _crc ^= b;
        if (_idx >= _cmd.payload_len) _state = S_CRC;
        return false;
    case S_CRC:
        if (b == _crc) {
            _state = S_SYNC1;
            return true;
        }
        reset();
        return false;
    }
    reset();
    return false;
}

// ============================================================
// ACK sender
// ============================================================

void telem_send_ack(Stream &serial, uint8_t cmd_id, uint8_t result)
{
    uint8_t frame[8];
    frame[0] = TELEM_SYNC1;
    frame[1] = TELEM_SYNC2;
    frame[2] = MSG_ACK;
    frame[3] = 0x00;  // LEN_H
    frame[4] = 0x02;  // LEN_L
    frame[5] = cmd_id;
    frame[6] = result;
    frame[7] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]; // CRC
    serial.write(frame, 8);
}

void telem_send_ack_ext(Stream &serial, uint8_t cmd_id, uint8_t result,
                        const uint8_t *extra, uint8_t extra_len)
{
    uint16_t payload_len = 2 + extra_len;
    uint8_t header[5] = { TELEM_SYNC1, TELEM_SYNC2, MSG_ACK,
                          (uint8_t)(payload_len >> 8), (uint8_t)(payload_len & 0xFF) };
    uint8_t crc = header[2] ^ header[3] ^ header[4] ^ cmd_id ^ result;
    for (uint8_t i = 0; i < extra_len; i++) crc ^= extra[i];
    serial.write(header, 5);
    serial.write(&cmd_id, 1);
    serial.write(&result, 1);
    serial.write(extra, extra_len);
    serial.write(&crc, 1);
}

// ============================================================
// Downlink: send TelemFrame
// ============================================================

void telem_send_frame(Stream &serial)
{
    TelemFrame tf;
    memset(&tf, 0, sizeof(tf));

    // System
    tf.mode = (uint8_t)current_mode;

    uint8_t flags = 0;
    if (ins_active)    flags |= STATUS_BIT_IMU_OK;
    if (motors_active) flags |= STATUS_BIT_MOTOR_ARMED;
    // RC: check if any channel has valid data
    if (rc_pulse_us[RC_CH_THROTTLE] >= 900 && rc_pulse_us[RC_CH_THROTTLE] <= 2200) flags |= STATUS_BIT_RC_OK;

    // Control source (bits 4-5)
    flags |= ((uint8_t)get_control_source() & 0x03) << STATUS_BIT_CTRL_SHIFT;

    // Fill from INS data
    if (ins_active) {
        MtiData d = ins.getData();

        // Attitude
        tf.roll_deg  = (float)d.data[ROLL];
        tf.pitch_deg = (float)d.data[PITCH];
        tf.yaw_deg   = (float)d.data[YAW];

        // Position (INS LatLon + GNSS PVT fallback)
        if (d.data_available & HAS_LATLON) {
            tf.lat_deg = d.data[LATITUDE];
            tf.lon_deg = d.data[LONGITUDE];
        }
        // Override with GNSS PVT raw if INS lat is integer (filter not converged)
        if ((d.data_available & HAS_GNSS_PVT) &&
            tf.lat_deg == (double)(int)tf.lat_deg) {
            tf.lat_deg = d.gnss_pvt.lat / 1e7;
            tf.lon_deg = d.gnss_pvt.lon / 1e7;
        }
        // Altitude: ellipsoid (XKF3 fusion, RTK-precise) is what MTi is configured to output
        if (d.data_available & HAS_ALT_ELLIPSOID) {
            tf.alt_m = (float)d.data[ALT_ELLIPSOID];
        } else if (d.data_available & HAS_ALT_MSL) {
            tf.alt_m = (float)d.data[ALT_MSL];
        }

        // Velocity NED (VEL_X=North, VEL_Y=East, VEL_Z=Down)
        if (d.data_available & HAS_VELOCITY_XYZ) {
            tf.vel_n_ms = (float)d.data[VEL_X];
            tf.vel_e_ms = (float)d.data[VEL_Y];
            tf.vel_u_ms = -(float)d.data[VEL_Z];  // Down -> Up for display
        }

        // GNSS
        if (d.data_available & HAS_STATUS_WORD) {
            if ((int)d.data[GNSS_FIX]) flags |= STATUS_BIT_GNSS_FIX;
            uint8_t rtk = (uint8_t)d.data[RTK_STATUS];
            flags |= (rtk & 0x03) << STATUS_RTK_SHIFT_TELEM;
        }
        if (d.data_available & HAS_GNSS_PVT) {
            tf.gnss_num_sv   = d.gnss_pvt.num_sv;
            tf.gnss_fix_type = d.gnss_pvt.fix_type;
        }

        // Yaw rate (gyro Z, NED frame, rad/s -> deg/s)
        if (d.data_available & HAS_GYRO)
            tf.yaw_rate_dps = (float)(d.data[GYR_Z] * RAD_TO_DEG);
    }

    tf.status_flags = flags;

    // Battery
    tf.batt1_mv = batt_main_mv;
    tf.batt2_mv = batt_sub_mv;

    // RC
    for (int i = 0; i < 5; i++) tf.rc_ch[i] = rc_pulse_us[i];

    // Motor
    if (motors_active) {
        for (int i = 0; i < 4; i++) tf.motor_us[i] = motors.getPulseWidth(i);
    }

    // Timestamp
    tf.uptime_ms = millis();

    // RTCM forward count (debug)
    tf.reserved[0] = (uint8_t)(rtcm_fwd_count >> 8);
    tf.reserved[1] = (uint8_t)(rtcm_fwd_count & 0xFF);

    // Send: [SYNC1][SYNC2][payload...][CRC]
    uint8_t header[2] = { TELEM_SYNC1, TELEM_SYNC2 };
    serial.write(header, 2);

    const uint8_t *p = (const uint8_t *)&tf;
    uint8_t crc = 0;
    for (uint16_t i = 0; i < sizeof(TelemFrame); i++) crc ^= p[i];
    serial.write(p, sizeof(TelemFrame));
    serial.write(&crc, 1);
}

// ============================================================
// Debug frame sender
// ============================================================

void telem_send_debug(Stream &serial, const DebugFrame &df)
{
    uint8_t header[3] = { TELEM_SYNC1, TELEM_SYNC2, MSG_DEBUG };
    serial.write(header, 3);

    const uint8_t *p = (const uint8_t *)&df;
    uint8_t crc = MSG_DEBUG;
    for (uint16_t i = 0; i < sizeof(DebugFrame); i++) crc ^= p[i];
    serial.write(p, sizeof(DebugFrame));
    serial.write(&crc, 1);
}

// ============================================================
// Uplink: handle command
// ============================================================

void telem_handle_cmd(const TelemCmd &cmd)
{
    switch (cmd.cmd_id) {
    case CMD_SET_MODE:
        if (cmd.payload_len >= 1 && cmd.payload[0] < MODE_COUNT) {
            FlightMode req = (FlightMode)cmd.payload[0];
            // RC controls mode when signal is present and in RC_CONTROL
            if (current_mode == MODE_RC_CONTROL &&
   				 rc_pulse_us[RC_CH_THROTTLE] > 1100) {
    			telem_send_ack(P8Serial,
                 			  	CMD_SET_MODE,
                   				ACK_ERR_WRONG_MODE);
			}
			else {
                // GCS explicitly selected a non-RC mode: this takes priority over
                // RC auto-entry. A stuck/hold RC signal (receiver still outputs the
                // last value after the TX is off) must not pull us back into
                // RC_CONTROL. The inhibit is cleared automatically when the RC
                // signal actually drops out (sketch.ino loop()).
                if (req != MODE_RC_CONTROL) {
                    rc_auto_inhibit = true;
                } else {
                    // GCS requested RC_CONTROL itself: honor it normally.
                    rc_auto_inhibit = false;
                }
    			enter_mode(req);
			}
        }
		else {
            telem_send_ack(P8Serial, CMD_SET_MODE, ACK_ERR_BAD_PARAM);
        }
        break;

    case CMD_ABORT:
        rc_auto_inhibit = true;  // prevent RC auto re-enter until RC cycles off/on
        exit_current_mode();
        telem_send_ack(P8Serial, CMD_ABORT, ACK_OK);
        #if USE_DEBUG_SERIAL
        Serial.println("[MODE] ABORT -> IDLE (RC inhibited)");
        #endif
        break;

    case CMD_PWM_SET:
        if (current_mode != MODE_PWM_TEST && current_mode != MODE_RC_CONTROL) {
            telem_send_ack(P8Serial, CMD_PWM_SET, ACK_ERR_WRONG_MODE);
        } else if (cmd.payload_len >= 3) {
            uint8_t ch = cmd.payload[0];
            uint16_t us = ((uint16_t)cmd.payload[1] << 8) | cmd.payload[2];
            if (ch < 4 && motors_active) {
                pwm_motor_us[ch] = us;
                // In PWM_TEST mode, apply immediately via set_motor_output
                if (current_mode == MODE_PWM_TEST) {
                    set_motor_output(pwm_motor_us[0], pwm_motor_us[1],
                                     pwm_motor_us[2], pwm_motor_us[3]);
                }
                // In RC_CONTROL AUTO mode, loop_rc_control() will pick up pwm_motor_us[]
                telem_send_ack(P8Serial, CMD_PWM_SET, ACK_OK);
            } else {
                telem_send_ack(P8Serial, CMD_PWM_SET, ACK_ERR_BAD_PARAM);
            }
        }
        break;

    case CMD_PWM_DISARM:
        if (current_mode == MODE_PWM_TEST && motors_active) {
            motors.disarm();
            for (int i = 0; i < 4; i++) pwm_motor_us[i] = motors.getMinPulse();
            telem_send_ack(P8Serial, CMD_PWM_DISARM, ACK_OK);
        } else {
            telem_send_ack(P8Serial, CMD_PWM_DISARM, ACK_ERR_WRONG_MODE);
        }
        break;

    case CMD_RTCM_DATA:
        if (ins_active && cmd.payload_len > 0) {
          ins.forwardRtcm(cmd.payload, cmd.payload_len);
          rtcm_fwd_count++;
        }
        break;

    case CMD_SET_WAYPOINTS: {
        // PNG mission upload. Refuse while PNG is active (entry-time upload only).
        if (current_mode == MODE_PNG_GUIDANCE) {
            telem_send_ack(P8Serial, CMD_SET_WAYPOINTS, ACK_ERR_WRONG_MODE);
            break;
        }
        // payload: [count][int32 lat, int32 lon, float alt, uint8 flags] x count
        //          (LE; lat/lon 1e-7 deg, alt m ground-relative). 13 bytes per WP.
        if (cmd.payload_len < 1) {
            telem_send_ack(P8Serial, CMD_SET_WAYPOINTS, ACK_ERR_BAD_PARAM);
            break;
        }
        uint8_t count = cmd.payload[0];
        if (count > PNG_MAX_WAYPOINTS ||
            cmd.payload_len != (uint16_t)(1 + (uint16_t)count * 13)) {
            telem_send_ack(P8Serial, CMD_SET_WAYPOINTS, ACK_ERR_BAD_PARAM);
            break;
        }
        png_set_waypoints(cmd.payload + 1, count);
        telem_send_ack(P8Serial, CMD_SET_WAYPOINTS, ACK_OK);
        #if USE_DEBUG_SERIAL
        Serial.print("[TELEM] Waypoints set: "); Serial.println(count);
        #endif
        break;
    }

    case CMD_EMERGENCY_LAND:
        // Autonomous modes (VEL/PNG/POS/ATL): latch a gentle fixed-thrust descent
        // (EMLAND_THRUST_FRAC*mg, level/station-keep). Each loop handles emland_active:
        //   PNG/POS via (CTRL_AUTO || emland_active); ATL via the emland_active &&
        //   atl_airborne guard; VEL via the forced descent thrust in ctrl_compute_thrust.
        // Cleared only by RC EMERGENCY / mode change, not by re-sending. Re-sending
        // while already latched is a harmless no-op (still ACK_OK).
        if (current_mode == MODE_VEL_CONTROL  ||
            current_mode == MODE_PNG_GUIDANCE ||
            current_mode == MODE_POS_CONTROL  ||
            current_mode == MODE_AUTO_TKO_LAND) {
            emland_active = true;
            telem_send_ack(P8Serial, CMD_EMERGENCY_LAND, ACK_OK);
            #if USE_DEBUG_SERIAL
            Serial.println("[EMLAND] Emergency landing armed");
            #endif
        } else {
            telem_send_ack(P8Serial, CMD_EMERGENCY_LAND, ACK_ERR_WRONG_MODE);
        }
        break;

    case CMD_AUTO_TAKEOFF:
        // AUTO_TKO_LAND / MISSION: request a takeoff. The loop's sequencer starts
        // it only when on the ground and ready (fix3D + IMU + low LiDAR + AUTO),
        // reported by atl_can_takeoff(). Latched flag consumed by the loop.
        if ((current_mode == MODE_AUTO_TKO_LAND || current_mode == MODE_MISSION) &&
            atl_can_takeoff()) {
            atl_takeoff_request = true;
            telem_send_ack(P8Serial, CMD_AUTO_TAKEOFF, ACK_OK);
            #if USE_DEBUG_SERIAL
            Serial.println("[ATL] Auto takeoff requested");
            #endif
        } else {
            telem_send_ack(P8Serial, CMD_AUTO_TAKEOFF, ACK_ERR_WRONG_MODE);
        }
        break;

    case CMD_MISSION_GUIDE:
        // MISSION only: hand off from HOLD to PNG guidance. Valid only while
        // holding altitude after a completed takeoff (atl_in_hold()).
        if (current_mode == MODE_MISSION && atl_in_hold()) {
            atl_mission_request = true;
            telem_send_ack(P8Serial, CMD_MISSION_GUIDE, ACK_OK);
            #if USE_DEBUG_SERIAL
            Serial.println("[MISSION] Guidance start requested");
            #endif
        } else {
            telem_send_ack(P8Serial, CMD_MISSION_GUIDE, ACK_ERR_WRONG_MODE);
        }
        break;

    case CMD_AUTO_LAND:
        // AUTO_TKO_LAND / MISSION: request a landing. AUTO_TKO_LAND requires HOLD
        // (atl_in_hold()); MISSION also allows landing directly (HOLD or, as an
        // abort, during guidance) -> any airborne sequence state via atl_in_hold()
        // OR mission guidance. The sequencer consumes req_land in HOLD/GUIDANCE.
        if ((current_mode == MODE_AUTO_TKO_LAND && atl_in_hold()) ||
            (current_mode == MODE_MISSION)) {
            atl_land_request = true;
            telem_send_ack(P8Serial, CMD_AUTO_LAND, ACK_OK);
            #if USE_DEBUG_SERIAL
            Serial.println("[ATL] Auto landing requested");
            #endif
        } else {
            telem_send_ack(P8Serial, CMD_AUTO_LAND, ACK_ERR_WRONG_MODE);
        }
        break;

    case CMD_DEBUG_TOGGLE:
        debug_output_enabled = !debug_output_enabled;
        telem_send_ack(P8Serial, CMD_DEBUG_TOGGLE, ACK_OK);
        #if USE_DEBUG_SERIAL
        Serial.print("[TELEM] Debug output: ");
        Serial.println(debug_output_enabled ? "ON" : "OFF");
        #endif
        break;

    case CMD_HEARTBEAT:
        telem_send_ack(P8Serial, CMD_HEARTBEAT, ACK_OK);
        break;

    default:
        telem_send_ack(P8Serial, cmd.cmd_id, ACK_ERR_UNKNOWN_CMD);
        break;
    }
}
