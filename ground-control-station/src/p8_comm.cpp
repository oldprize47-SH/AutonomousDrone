#include "p8_comm.h"
#include "serial_port.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

const char* MODE_NAMES[MODE_COUNT] = {
    "IDLE", "RC_CONTROL", "RTK_MAIN", "PWM_TEST", "MTI_TEST", "ESC_CAL",
    "VEL_CONTROL", "PNG_GUIDANCE", "POS_CONTROL", "AUTO_TKO_LAND", "MISSION"
};

// AtlState names (must match MCU Mode.cpp AtlState enum order).
static const char* ATL_STATE_NAMES[] = {
    "GROUND", "SPOOLUP", "TAKEOFF", "HOLD", "GUIDANCE",
    "LAND", "LAND_SLOW", "SPOOLDOWN", "DISARMED"
};
const char* atl_state_name(uint8_t s)
{
    if (s < (sizeof(ATL_STATE_NAMES) / sizeof(ATL_STATE_NAMES[0])))
        return ATL_STATE_NAMES[s];
    return "?";
}

// ============================================================
// Frame sanity validation
// ------------------------------------------------------------
// The link CRC is a single XOR byte (1/256 chance a corrupted frame passes).
// When the MCU runs a tight control loop while NTRIP RTCM streams in the
// opposite direction, an RX buffer overrun can drop a byte, slide the framing,
// and let a garbage DebugFrame slip through CRC. Such frames showed up in logs
// as e.g. avoid=60, fix=211, lon=1e148. These post-CRC checks reject frames
// whose values are physically impossible, so they never reach the log/UI.
// ============================================================

static bool finite_f(float v)  { return v == v && v > -1e30f && v < 1e30f; }
static bool finite_d(double v) { return v == v && v > -1e30  && v < 1e30; }

static bool debug_frame_sane(const DebugFrame* d)
{
    // Enum / boolean fields must hold their declared small ranges.
    if (d->lidar_valid  > 1) return false;
    if (d->alt_hold     > 1) return false;
    if (d->obs_avoid    > 1) return false;
    if (d->rtk_status   > 2) return false;
    if (d->gnss_fix     > 1) return false;
    if (d->vision_use   > 1) return false;
    if (d->vlatch_set   > 1) return false;
    if (d->vision_reject > 1) return false;
    if (d->atl_state    > 8) return false;   // GROUND..DISARMED
    if (d->wp_idx       > PNG_MAX_WAYPOINTS) return false;

    // GNSS lat/lon must be on the globe (0 is the valid "no fix" sentinel).
    if (d->png_lat < -90.0  || d->png_lat > 90.0)  return false;
    if (d->png_lon < -180.0 || d->png_lon > 180.0) return false;

    // A few wide-but-bounded floats catch e+18/e+148 style garbage.
    if (!finite_f(d->throttle_N)) return false;
    if (!finite_f(d->obs_Ro) || !finite_f(d->obs_eo) || !finite_f(d->obs_gamma)) return false;
    if (!finite_f(d->wp_alt_cmd) || !finite_f(d->vlatch_n) || !finite_f(d->vlatch_e)) return false;
    if (!finite_f(d->roll) || !finite_f(d->pitch) || !finite_f(d->yaw)) return false;
    if (!finite_d(d->png_lat) || !finite_d(d->png_lon)) return false;

    return true;
}

static bool telem_frame_sane(const TelemFrame* t)
{
    if (t->mode >= MODE_COUNT) return false;
    if (t->lat_deg < -90.0  || t->lat_deg > 90.0)  return false;
    if (t->lon_deg < -180.0 || t->lon_deg > 180.0) return false;
    if (!finite_f(t->roll_deg) || !finite_f(t->pitch_deg) || !finite_f(t->yaw_deg)) return false;
    if (!finite_f(t->alt_m)) return false;
    return true;
}

// ============================================================
// Parser
// ============================================================

void p8_parser_init(P8Parser* p)
{
    memset(p, 0, sizeof(P8Parser));
    p->state = P8_SYNC1_WAIT;
    p->result_type = P8_MSG_NONE;
    p->rebuf_len = 0;
    p->replaying = false;
}

// Core byte-at-a-time state machine. Returns the message type completed by this
// byte (or P8_MSG_NONE). On a CRC/sanity rejection it sets p->result_type to a
// private sentinel via the REJECT macro so the wrapper can trigger reframing.
static P8MsgType p8_parser_step(P8Parser* p, uint8_t b);

// Wrapper: capture every byte of the in-progress frame so a rejected frame can
// be re-scanned for a real boundary instead of skipped over. While SYNC-hunting
// (no frame started yet) there is nothing to buffer, so this is essentially free
// on a clean link; the replay path only runs on an actual rejection.
P8MsgType p8_parser_feed(P8Parser* p, uint8_t b)
{
    // Capture into the reframing buffer unless we are mid-replay (avoid
    // re-capturing bytes we are already replaying).
    if (!p->replaying && p->state != P8_SYNC1_WAIT) {
        if (p->rebuf_len < sizeof(p->rebuf))
            p->rebuf[p->rebuf_len++] = b;
        else
            p->rebuf_len = 0; // overflow guard: drop the runaway window
    } else if (!p->replaying && p->state == P8_SYNC1_WAIT) {
        p->rebuf_len = 0; // no frame in progress; start fresh on next SYNC1
    }

    P8MsgType r = p8_parser_step(p, b);

    // Success: clear the window, the frame was consumed cleanly.
    if (r == P8_MSG_TELEM || r == P8_MSG_ACK || r == P8_MSG_DEBUG) {
        p->rebuf_len = 0;
        return r;
    }

    // Rejection (CRC or sanity fail): step() reset state to P8_SYNC1_WAIT and
    // returned NONE. Reframe: drop the leading SYNC1 byte of the bad window and
    // replay the rest so a real frame start hidden inside it is recovered.
    if (p->state == P8_SYNC1_WAIT && p->result_type == P8_MSG_REJECT && !p->replaying) {
        uint8_t saved[sizeof(p->rebuf)];
        uint16_t n = p->rebuf_len;
        if (n > 1) {
            memcpy(saved, p->rebuf + 1, n - 1); // skip the consumed SYNC1
            uint16_t m = n - 1;
            p->rebuf_len = 0;
            p->replaying = true;
            P8MsgType replay_result = P8_MSG_NONE;
            for (uint16_t i = 0; i < m; i++) {
                P8MsgType rr = p8_parser_feed(p, saved[i]);
                if (rr != P8_MSG_NONE) replay_result = rr;
            }
            p->replaying = false;
            return replay_result;
        }
        p->rebuf_len = 0;
    }

    p->result_type = P8_MSG_NONE;
    return P8_MSG_NONE;
}

static P8MsgType p8_parser_step(P8Parser* p, uint8_t b)
{
    p->result_type = P8_MSG_NONE;

    switch (p->state) {
    case P8_SYNC1_WAIT:
        if (b == P8_SYNC1) p->state = P8_SYNC2_WAIT;
        return P8_MSG_NONE;

    case P8_SYNC2_WAIT:
        if (b == P8_SYNC2) {
            p->state = P8_ACK_ID;
            p->idx = 0;
            p->crc = 0;
        } else {
            p->state = P8_SYNC1_WAIT;
        }
        return P8_MSG_NONE;

    case P8_ACK_ID:
        // Dispatch: MSG_ACK(0xF1) -> ACK, MSG_DEBUG(0xDB) -> Debug, else -> TelemFrame
        if (b == MSG_ACK) {
            p->ack_msg_id = b;
            p->crc = b;
            p->state = P8_ACK_LEN_H;
        } else if (b == MSG_DEBUG) {
            p->crc = b;
            p->idx = 0;
            p->state = P8_DEBUG_PAYLOAD;
        } else {
            p->telem_buf[0] = b;
            p->crc = b;
            p->idx = 1;
            p->state = P8_TELEM_PAYLOAD;
        }
        return P8_MSG_NONE;

    case P8_TELEM_PAYLOAD:
        p->telem_buf[p->idx++] = b;
        p->crc ^= b;
        if (p->idx >= sizeof(TelemFrame))
            p->state = P8_TELEM_CRC;
        return P8_MSG_NONE;

    case P8_TELEM_CRC:
        if (b == p->crc && telem_frame_sane((const TelemFrame*)p->telem_buf)) {
            p->result_type = P8_MSG_TELEM;
            p->state = P8_SYNC1_WAIT;
            return P8_MSG_TELEM;
        }
        // Bad CRC or impossible values: reject this completed frame and reframe.
        p->state = P8_SYNC1_WAIT;
        p->result_type = P8_MSG_REJECT;
        return P8_MSG_NONE;

    case P8_ACK_LEN_H:
        p->ack_payload_len = (uint16_t)b << 8;
        p->crc ^= b;
        p->state = P8_ACK_LEN_L;
        return P8_MSG_NONE;

    case P8_ACK_LEN_L:
        p->ack_payload_len |= b;
        p->crc ^= b;
        p->idx = 0;
        if (p->ack_payload_len > sizeof(p->ack_payload)) {
            // Implausible length -> this was not really an ACK frame; reframe.
            p->state = P8_SYNC1_WAIT;
            p->result_type = P8_MSG_REJECT;
            return P8_MSG_NONE;
        }
        p->state = (p->ack_payload_len == 0) ? P8_ACK_CRC : P8_ACK_PAYLOAD;
        return P8_MSG_NONE;

    case P8_ACK_PAYLOAD:
        p->ack_payload[p->idx++] = b;
        p->crc ^= b;
        if (p->idx >= p->ack_payload_len) p->state = P8_ACK_CRC;
        return P8_MSG_NONE;

    case P8_ACK_CRC:
        if (b == p->crc) {
            p->result_type = P8_MSG_ACK;
            p->state = P8_SYNC1_WAIT;
            return P8_MSG_ACK;
        }
        // Bad ACK CRC: reject and reframe.
        p->state = P8_SYNC1_WAIT;
        p->result_type = P8_MSG_REJECT;
        return P8_MSG_NONE;

    case P8_DEBUG_PAYLOAD:
        p->debug_buf[p->idx++] = b;
        p->crc ^= b;
        if (p->idx >= sizeof(DebugFrame))
            p->state = P8_DEBUG_CRC;
        return P8_MSG_NONE;

    case P8_DEBUG_CRC:
        if (b == p->crc && debug_frame_sane((const DebugFrame*)p->debug_buf)) {
            p->result_type = P8_MSG_DEBUG;
            p->state = P8_SYNC1_WAIT;
            return P8_MSG_DEBUG;
        }
        // Bad CRC or impossible values (framing slid under RTCM load): reject
        // and reframe so a real frame start hidden in this window is recovered
        // instead of printing garbage or skipping past the next frame.
        p->state = P8_SYNC1_WAIT;
        p->result_type = P8_MSG_REJECT;
        return P8_MSG_NONE;
    }

    p->state = P8_SYNC1_WAIT;
    return P8_MSG_NONE;
}

const TelemFrame* p8_get_telem(const P8Parser* p)
{
    return (const TelemFrame*)p->telem_buf;
}

const DebugFrame* p8_get_debug(const P8Parser* p)
{
    return (const DebugFrame*)p->debug_buf;
}

// ============================================================
// Uplink command senders
// ============================================================

bool p8_send_cmd(uint8_t cmd_id, const uint8_t* payload, uint16_t len)
{
    uint8_t buf[P8_MAX_PAYLOAD + 6];
    buf[0] = P8_SYNC1;
    buf[1] = P8_SYNC2;
    buf[2] = cmd_id;
    buf[3] = (uint8_t)(len >> 8);
    buf[4] = (uint8_t)(len & 0xFF);

    uint8_t crc = cmd_id ^ buf[3] ^ buf[4];
    for (uint16_t i = 0; i < len; i++) {
        buf[5 + i] = payload[i];
        crc ^= payload[i];
    }
    buf[5 + len] = crc;

    return serial_write(buf, 6 + len);
}

bool p8_send_set_mode(uint8_t mode_id)    { return p8_send_cmd(CMD_SET_MODE, &mode_id, 1); }
bool p8_send_abort()                       { return p8_send_cmd(CMD_ABORT); }
bool p8_send_heartbeat()                   { return p8_send_cmd(CMD_HEARTBEAT); }
bool p8_send_pwm_disarm()                  { return p8_send_cmd(CMD_PWM_DISARM); }

bool p8_send_pwm_set(uint8_t ch, uint16_t us)
{
    uint8_t payload[3] = { ch, (uint8_t)(us >> 8), (uint8_t)(us & 0xFF) };
    return p8_send_cmd(CMD_PWM_SET, payload, 3);
}

bool p8_send_rtcm(const uint8_t* data, uint16_t len)
{
    return p8_send_cmd(CMD_RTCM_DATA, data, len);
}

bool p8_send_debug_toggle()
{
    return p8_send_cmd(CMD_DEBUG_TOGGLE);
}

bool p8_send_emergency_land()
{
    return p8_send_cmd(CMD_EMERGENCY_LAND);
}

bool p8_send_auto_takeoff()
{
    return p8_send_cmd(CMD_AUTO_TAKEOFF);
}

bool p8_send_auto_land()
{
    return p8_send_cmd(CMD_AUTO_LAND);
}

bool p8_send_mission_start()
{
    return p8_send_cmd(CMD_MISSION_GUIDE);
}

bool p8_send_waypoints(const double* lat, const double* lon,
                       const float* alt, const uint8_t* flags, uint8_t count)
{
    if (count > PNG_MAX_WAYPOINTS) return false;

    // payload: [count] + count * { int32 lat, int32 lon, float alt, uint8 flags }
    //          all little-endian; lat/lon in 1e-7 deg, alt in m (ground-relative).
    uint8_t payload[1 + PNG_MAX_WAYPOINTS * 13];
    payload[0] = count;
    for (uint8_t i = 0; i < count; i++) {
        int32_t la = (int32_t)lround(lat[i] * 1e7);
        int32_t lo = (int32_t)lround(lon[i] * 1e7);
        // float alt -> 32-bit LE pattern (x86 + Cortex-M33 both LE IEEE-754).
        union { float f; uint32_t u; } alt_cvt;
        alt_cvt.f = alt[i];
        uint32_t ab = alt_cvt.u;
        uint8_t* p = payload + 1 + (size_t)i * 13;
        p[0]  = (uint8_t)(la);       p[1]  = (uint8_t)(la >> 8);
        p[2]  = (uint8_t)(la >> 16); p[3]  = (uint8_t)(la >> 24);
        p[4]  = (uint8_t)(lo);       p[5]  = (uint8_t)(lo >> 8);
        p[6]  = (uint8_t)(lo >> 16); p[7]  = (uint8_t)(lo >> 24);
        p[8]  = (uint8_t)(ab);       p[9]  = (uint8_t)(ab >> 8);
        p[10] = (uint8_t)(ab >> 16); p[11] = (uint8_t)(ab >> 24);
        p[12] = flags ? flags[i] : 0;
    }
    return p8_send_cmd(CMD_SET_WAYPOINTS, payload, (uint16_t)(1 + (uint16_t)count * 13));
}

// ============================================================
// Display helpers
// ============================================================

const char* ack_result_str(uint8_t r)
{
    switch (r) {
    case ACK_OK:              return "OK";
    case ACK_ERR_UNKNOWN_CMD: return "UNKNOWN_CMD";
    case ACK_ERR_BAD_PARAM:   return "BAD_PARAM";
    case ACK_ERR_WRONG_MODE:  return "WRONG_MODE";
    case ACK_ERR_HW_FAIL:     return "HW_FAIL";
    default:                  return "???";
    }
}

const char* rtk_status_str(uint8_t flags)
{
    uint8_t rtk = (flags >> STATUS_RTK_SHIFT) & 0x03;
    switch (rtk) {
    case 2:  return "Fixed";
    case 1:  return "Float";
    default: return "None";
    }
}

const char* ctrl_source_str(uint8_t flags)
{
    uint8_t ctrl = (flags >> STATUS_BIT_CTRL_SHIFT) & 0x03;
    switch (ctrl) {
    case 0:  return "EMERG";
    case 1:  return "MANUAL";
    case 2:  return "AUTO";
    default: return "???";
    }
}

void print_telem(const TelemFrame* tf)
{
    const char* mode_str = (tf->mode < MODE_COUNT) ? MODE_NAMES[tf->mode] : "???";
    uint8_t f = tf->status_flags;

    uint16_t rtcm_cnt = ((uint16_t)tf->reserved[0] << 8) | tf->reserved[1];

    char line[512];
    int len = snprintf(line, sizeof(line),
           "[TELEM] %s | CTRL:%-6s | IMU:%c MTR:%c RC:%c GNSS:%c RTK:%-5s | "
           "R=%7.2f P=%7.2f Y=%7.2f | "
           "Lat=%.7f Lon=%.7f Alt(ell)=%.2f | "
           "Vn=%6.2f Ve=%6.2f Vu=%6.2f | "
           "SV=%2d | B1=%.2fV B2=%.2fV | RTCM=%u | %u.%03us",
           mode_str,
           ctrl_source_str(f),
           (f & STATUS_BIT_IMU_OK)      ? 'O' : 'X',
           (f & STATUS_BIT_MOTOR_ARMED) ? 'O' : 'X',
           (f & STATUS_BIT_RC_OK)       ? 'O' : 'X',
           (f & STATUS_BIT_GNSS_FIX)    ? 'O' : 'X',
           rtk_status_str(f),
           tf->roll_deg, tf->pitch_deg, tf->yaw_deg,
           tf->lat_deg, tf->lon_deg, tf->alt_m,
           tf->vel_n_ms, tf->vel_e_ms, tf->vel_u_ms,
           tf->gnss_num_sv,
           tf->batt1_mv / 1000.0f, tf->batt2_mv / 1000.0f,
           rtcm_cnt,
           tf->uptime_ms / 1000, tf->uptime_ms % 1000);

    // 이전 출력 잔여 문자를 지우기 위해 빈 공간으로 패딩
    static int prev_len = 0;
    int pad = (prev_len > len) ? prev_len - len : 0;
    prev_len = len;

    printf("\r%s%*s", line, pad, "");
    fflush(stdout);
}

void print_debug(const DebugFrame* df)
{
    printf("\n[DEBUG] t=%.3f | R=%.2f P=%.2f Y=%.2f | "
           "p=%.2f q=%.2f r=%.2f | "
           "cmd=[%.3f %.3f %.3f] | "
           "rate_cmd=[%.3f %.3f] pf=%.3f qf=%.3f | "
           "e_rp=[%.3f %.3f] int_rp=[%.3f %.3f] ff=[%.3f %.3f] | "
           "yawPID e=%.3f int=%.3f I=%.3f | "
           "U1=%.1f U2=%.3f U3=%.3f U4=%.3f | "
           "thr=%.1fN | "
           "lidar=%.2f(%d) | "
           "hold=%d cmd=%.2f e=%.2f hdot=%.2f dT=%.1f | "
           "fmf h=%.2f v=%.2f a=%.2f h_used=%.2f v_used=%.2f | "
           "velX cmd=%.2f fb=%.2f acc_f=%.2f err=%.2f | "
           "velY cmd=%.2f fb=%.2f acc_f=%.2f err=%.2f | "
           "png NE=[%.2f %.2f] rng=%.2f eta=%.2f yr=%.3f wp=%d "
           "ll=[%.7f %.7f] alt=%.1f | "
           "obs r_ca=%.3f Ro=%.2f eo=%.3f g=%.3f th_d=%.3f avoid=%d | "
           "ATL=%s climb=%.2f hold=%.2f phase=%ums wp_alt=%.2f | "
           "LAND raw[v=%d xb=%.3f yb=%.3f xy=%.3f gnd=%.2f] "
           "dcm[dn=%.3f de=%.3f] "
           "latch[set=%d N=%.2f E=%.2f] use=%d rej=%d | "
           "rt=%u dtmax=%.1fms B=%.2fV | "
           "rtk=%d fix=%d NE=[%.2f %.2f] uv=[%.2f %.2f]",
           df->uptime_ms / 1000.0,
           df->roll * 57.2958f, df->pitch * 57.2958f, df->yaw * 57.2958f,
           df->p, df->q, df->r,
           df->roll_cmd, df->pitch_cmd, df->r_cmd,
           df->roll_rate_cmd, df->pitch_rate_cmd, df->p_f, df->q_f,
           df->e_roll, df->e_pitch, df->int_e_roll, df->int_e_pitch,
           df->roll_ff, df->pitch_ff,
           df->e_r, df->int_e_r, df->I_r,
           df->U1, df->U2, df->U3, df->U4,
           df->throttle_N,
           df->lidar_alt, (int)df->lidar_valid,
           (int)df->alt_hold, df->alt_cmd, df->alt_error,
           df->hdot_cmd, df->DeltaT_cmd,
           df->h_fmf, df->v_fmf, df->a_fmf, df->h_used, df->v_used,
           df->u_cmd, df->u_fb, df->u_acc, df->u_err,
           df->v_cmd, df->v_fb, df->v_acc, df->v_err,
           df->png_px, df->png_py, df->png_rng, df->png_eta,
           df->png_yaw_rate, (int)df->wp_idx,
           df->png_lat, df->png_lon, df->png_alt,
           df->obs_r_ca, df->obs_Ro, df->obs_eo, df->obs_gamma,
           df->obs_theta_dot, (int)df->obs_avoid,
           atl_state_name(df->atl_state), df->atl_climb_cmd, df->atl_hold_alt,
           (unsigned)df->atl_phase_ms, df->wp_alt_cmd,
           (int)df->helipad_valid, df->helipad_x, df->helipad_y,
           df->helipad_xy, df->helipad_gnd,
           df->helipad_dn, df->helipad_de,
           (int)df->vlatch_set, df->vlatch_n, df->vlatch_e,
           (int)df->vision_use, (int)df->vision_reject,
           df->loop_count_50hz, df->loop_dt_max_ms, df->batt_mv / 1000.0f,
           (int)df->rtk_status, (int)df->gnss_fix,
           df->vel_n, df->vel_e, df->vel_u, df->vel_v);
    // Group 9: mission WP local-NED cache (static; print a compact summary only,
    // the full 32-pair array is logged to the debug CSV). Show count + WP0/last.
    if (df->wp_ned_count > 0) {
        int last = (df->wp_ned_count <= PNG_MAX_WAYPOINTS)
                     ? df->wp_ned_count - 1 : PNG_MAX_WAYPOINTS - 1;
        printf("  WP_NED n=%d  [0]=(%.2f,%.2f)  [%d]=(%.2f,%.2f)\n",
               (int)df->wp_ned_count, df->wp_north[0], df->wp_east[0],
               last, df->wp_north[last], df->wp_east[last]);
    }
    fflush(stdout);
}

void print_ack(uint8_t cmd_id, uint8_t result)
{
    printf("[ACK] cmd=0x%02X result=%s\n", cmd_id, ack_result_str(result));
}
