#pragma once
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// P8 Communication Module
// Mirrors UnoQ Telem.h protocol definitions
// ============================================================

#define P8_SYNC1     0xAA
#define P8_SYNC2     0x55
#define P8_MAX_PAYLOAD 2048

// Uplink CMD_ID (GCS -> MCU)
#define CMD_SET_MODE    0x01
#define CMD_ABORT       0x02
#define CMD_PWM_SET     0x03
#define CMD_PWM_DISARM  0x04
#define CMD_RTCM_DATA   0x10
#define CMD_DEBUG_TOGGLE 0x20
#define CMD_SET_WAYPOINTS 0x30  // PNG mission: [count][int32 lat,int32 lon]... (1e-7 deg)
#define CMD_EMERGENCY_LAND 0x40 // PNG: latch U1 to EMLAND_THRUST_FRAC*mg, gentle descent (no payload)
#define CMD_AUTO_TAKEOFF 0x41   // AUTO_TKO_LAND/MISSION: start auto takeoff (no payload)
#define CMD_AUTO_LAND    0x42   // AUTO_TKO_LAND/MISSION: start auto landing from HOLD (no payload)
#define CMD_MISSION_GUIDE 0x43  // MISSION mode: start PNG guidance from HOLD (no payload)
#define CMD_HEARTBEAT   0xF0

// Sub-PC mission upload protocol (mav_bridge.py -> GCS over UDP). GCS parses
// these into a WayPoint/*.txt file; it does NOT forward them to the MCU. The
// user later picks a file with 'w' and uploads it via CMD_SET_WAYPOINTS.
#define CMD_MISSION_COUNT 0x11  // payload: [count(u8)] start of mission, N waypoints
#define CMD_MISSION_ITEM  0x12  // payload: <B i i i> = idx, lat_1e7, lon_1e7, alt_mm
#define CMD_MISSION_START 0x13  // no payload: mission complete -> save txt
#define CMD_MISSION_CLEAR 0x14  // no payload: clear pending mission

#define PNG_MAX_WAYPOINTS 32    // must match UnoQ Mode.h

// Downlink
#define MSG_ACK         0xF1
#define MSG_DEBUG       0xDB

// ACK results
#define ACK_OK              0x00
#define ACK_ERR_UNKNOWN_CMD 0x01
#define ACK_ERR_BAD_PARAM   0x02
#define ACK_ERR_WRONG_MODE  0x03
#define ACK_ERR_HW_FAIL     0x04

// Flight modes (must match UnoQ FlightMode enum order)
//   0=IDLE 1=RC_CONTROL 2=RTK_MAIN 3=PWM_TEST 4=MTI_TEST 5=ESC_CAL
//   6=VEL_CONTROL 7=PNG_GUIDANCE 8=POS_CONTROL 9=AUTO_TKO_LAND 10=MISSION
#define MODE_COUNT 11
extern const char* MODE_NAMES[MODE_COUNT];

// Status flag bits
#define STATUS_BIT_IMU_OK       (1u << 0)
#define STATUS_BIT_MOTOR_ARMED  (1u << 1)
#define STATUS_BIT_RC_OK        (1u << 2)
#define STATUS_BIT_GNSS_FIX     (1u << 3)
#define STATUS_BIT_CTRL_SHIFT   4       // bits 4-5: ControlSource
#define STATUS_BIT_CTRL_MASK   0x30    // bit4-5
#define STATUS_RTK_SHIFT        6

// ============================================================
// TelemFrame (76 bytes, packed) - matches MCU definition
// ============================================================
#pragma pack(push, 1)
typedef struct {
    uint8_t  mode;
    uint8_t  status_flags;
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    float    vel_e_ms;
    float    vel_n_ms;
    float    vel_u_ms;
    uint8_t  gnss_num_sv;
    uint8_t  gnss_fix_type;
    uint16_t batt1_mv;
    uint16_t batt2_mv;
    uint16_t rc_ch[5];
    uint16_t motor_us[4];
    uint32_t uptime_ms;
    float    yaw_rate_dps;
    uint8_t  reserved[2];
} TelemFrame;

// ============================================================
// DebugFrame (control loop logging) - matches MCU Telem.h byte-for-byte
// ============================================================
typedef struct {
    // ---- Group 1: attitude controller --------------------------------------
    float    roll;
    float    pitch;
    float    yaw;
    float    p;
    float    q;
    float    r;
    float    roll_cmd;          // target roll  [rad] (applied setpoint)
    float    pitch_cmd;         // target pitch [rad] (applied setpoint)
    float    r_cmd;             // target yaw rate [rad/s]
    float    roll_rate_cmd;     // outer Angle PI output (roll)  [rad/s]
    float    pitch_rate_cmd;    // outer Angle PI output (pitch) [rad/s]
    float    p_f;               // 35Hz-LPF roll rate feedback   [rad/s]
    float    q_f;               // 35Hz-LPF pitch rate feedback  [rad/s]
    float    e_roll;            // outer PI roll error  [rad]
    float    e_pitch;           // outer PI pitch error [rad]
    float    int_e_roll;        // outer PI roll integrator state  [rad*s]
    float    int_e_pitch;       // outer PI pitch integrator state [rad*s]
    float    roll_ff;           // takeoff-trim feed-forward (roll)  [rad]
    float    pitch_ff;          // takeoff-trim feed-forward (pitch) [rad]
    float    e_r;               // yaw rate error [rad/s]
    float    int_e_r;           // yaw rate integrator state
    float    I_r;               // yaw rate I term
    float    U1, U2, U3, U4;    // torques [Nm]
    // F1..F4 (per-motor thrust) removed to shrink the frame. Mirrors UnoQ Telem.h.

    // ---- Group 2: altitude controller --------------------------------------
    // Barometer/complementary-filter debug fields (alt_est, alt_baro,
    // free_acc_z, pressure_pa, w_down) were removed to shrink the frame.
    // Mirrors UnoQ Telem.h byte-for-byte.
    float    throttle_N;        // total thrust U1 command [N]
    float    lidar_alt;         // body-tilt-comp LiDAR altitude [m] (0 if invalid)
    uint8_t  lidar_valid;       // 1 = LiDAR used as aiding source
    uint8_t  alt_hold;          // 1 = altitude-hold active (RC_CH_MODE AUTO)
    float    alt_cmd;           // hold setpoint [m] (up+)
    float    alt_error;         // alt error [m] (NED down+, h_cmd-h)
    float    hdot_cmd;          // commanded vertical speed [m/s] (NED down+)
    float    DeltaT_cmd;        // thrust correction [N] (T_cmd = mg - DeltaT)
    float    h_fmf;             // FMF height   [m]   (up+)
    float    v_fmf;             // FMF velocity [m/s] (up+)
    float    a_fmf;             // FMF accel    [m/s^2] (Constant_Acc only; else 0)
    float    h_used;            // altitude feedback used  [m]   (up+)
    float    v_used;            // vertical-speed feedback [m/s] (up+)

    // ---- Group 3: velocity controller (body frame, X->u, Y->v) -------------
    float    u_cmd;             // X velocity command  [m/s] (body forward)
    float    u_fb;              // X velocity feedback [m/s] (body forward, GNSS)
    float    u_acc;             // X free acceleration [m/s^2] (body forward)
    float    u_err;             // X velocity error    [m/s] (u_cmd - u_fb)
    float    v_cmd;             // Y velocity command  [m/s] (body right)
    float    v_fb;              // Y velocity feedback [m/s] (body right, GNSS)
    float    v_acc;             // Y free acceleration [m/s^2] (body right)
    float    v_err;             // Y velocity error    [m/s] (v_cmd - v_fb)

    // ---- Group 4: PNG guidance ---------------------------------------------
    float    png_px;            // local NED North position (origin-ref) [m]
    float    png_py;            // local NED East  position (origin-ref) [m]
    float    png_rng;           // horizontal distance to current waypoint [m]
    float    png_eta;           // heading error (LOS - yaw) [rad]
    float    png_los;           // line-of-sight angle to target (NED, North=0, East+) [rad]
    float    png_yaw_rate;      // commanded yaw rate [rad/s]
    uint8_t  wp_idx;            // current waypoint index
    double   png_lat;           // GNSS latitude  [deg] (0 outside PNG/fix loss)
    double   png_lon;           // GNSS longitude [deg]
    float    png_alt;           // GNSS height    [m] (ellipsoid)

    // ---- Group 4b: collision-cone obstacle avoidance -----------------------
    // 0 outside MODE_PNG_GUIDANCE / when no fresh H7 obstacle.
    float    obs_r_ca;          // avoidance yaw-rate command [rad/s]
    float    obs_Ro;            // nearest obstacle range [m] (0 = none)
    float    obs_eo;            // obstacle heading error [rad]
    float    obs_gamma;         // collision-cone half angle [rad]
    float    obs_theta_dot;     // cone-boundary angle rate [rad/s]
    uint8_t  obs_avoid;         // 1 = avoidance latch active

    // ---- Group 5: misc (timing, battery, GNSS/RTK) -------------------------
    uint32_t uptime_ms;
    uint16_t batt_mv;
    uint16_t loop_count_50hz;   // # of 200Hz iterations in the 50Hz window (~4)
    float    loop_dt_max_ms;    // max 200Hz loop period in window [ms] (~5)
    uint8_t  rtk_status;        // 0=none, 1=float, 2=fixed
    uint8_t  gnss_fix;          // 1 = GNSS fix valid
    float    vel_n;             // GNSS/RTK fusion velocity, NED North [m/s]
    float    vel_e;             // GNSS/RTK fusion velocity, NED East  [m/s]
    float    vel_u;             // GNSS velocity rotated to body forward (u) [m/s]
    float    vel_v;             // GNSS velocity rotated to body right   (v) [m/s]

    // ---- Group 6: auto takeoff/landing (MODE_AUTO_TKO_LAND) -----------------
    uint8_t  atl_state;         // AtlState: 0=GROUND 1=SPOOLUP 2=TAKEOFF 3=HOLD
                                // 4=LAND 5=LAND_SLOW 6=SPOOLDOWN 7=DISARMED
    float    atl_climb_cmd;     // commanded climb rate [m/s, up+] (+climb,-descend)
    float    atl_hold_alt;      // captured altitude-hold setpoint [m, up+]
    uint16_t atl_phase_ms;      // ms remaining in spool-up/down phase (0 otherwise)

    // ---- Group 7: vision precision landing (RealSense helipad, ATL only) ----
    // (a) RAW camera input (latest received, even if not steering)
    float    helipad_x;         // raw body forward offset from camera [m] (+fwd)
    float    helipad_y;         // raw body right   offset from camera [m] (+right)
    float    helipad_xy;        // raw horizontal error sqrt(x^2+y^2) [m]
    float    helipad_gnd;       // raw drone-centre ground distance [m]
    uint8_t  helipad_valid;     // raw vision valid (and fresh) flag (1=valid)
    // (b) attitude-corrected NED offset (full roll/pitch/yaw DCM, pre-add)
    float    helipad_dn;        // DCM-corrected NED north offset [m] (vs raw helipad_x)
    float    helipad_de;        // DCM-corrected NED east  offset [m] (vs raw helipad_y)
    // (c) NED latch actually used by the position PID
    float    vlatch_n;          // latched helipad target, NED north [m] (0 if no lock)
    float    vlatch_e;          // latched helipad target, NED east  [m] (0 if no lock)
    // (d) controller status
    uint8_t  vision_use;        // 1 = steering to the latched helipad point
    uint8_t  vlatch_set;        // 1 = a helipad point is latched (lock acquired)
    uint8_t  vision_reject;     // 1 = this step's fix rejected as a spike (jump)

    // ---- Group 8: MISSION waypoint altitude command (MODE_MISSION only) -------
    // Ramped target altitude the vertical alt-hold tracks toward the active WP
    // (linear interpolation across the segment). Equals atl_hold_alt outside
    // GUIDANCE. Appended last to match the MCU DebugFrame byte layout.
    float    wp_alt_cmd;        // ramped WP altitude setpoint [m, takeoff-ground-relative]

    // ---- Group 9: MISSION waypoint local-NED cache (MODE_MISSION only) --------
    // Uploaded waypoints converted to local NED at mode entry (origin = takeoff
    // GNSS fix). Static for the whole flight, so values repeat every frame; only
    // the first wp_ned_count entries are valid, the rest are 0. Mirrors the MCU
    // DebugFrame Group 9 byte layout exactly.
    uint8_t  wp_ned_count;                 // # of valid waypoints (= png_wp_size)
    float    wp_north[PNG_MAX_WAYPOINTS];  // local-NED North per WP [m] (origin-ref)
    float    wp_east [PNG_MAX_WAYPOINTS];  // local-NED East  per WP [m] (origin-ref)
} DebugFrame;
#pragma pack(pop)

// ============================================================
// Downlink parser
// ============================================================
enum P8ParseState {
    P8_SYNC1_WAIT, P8_SYNC2_WAIT,
    P8_TELEM_PAYLOAD, P8_TELEM_CRC,
    P8_ACK_ID, P8_ACK_LEN_H, P8_ACK_LEN_L, P8_ACK_PAYLOAD, P8_ACK_CRC,
    P8_DEBUG_PAYLOAD, P8_DEBUG_CRC
};

enum P8MsgType {
    P8_MSG_NONE = 0, P8_MSG_TELEM, P8_MSG_ACK, P8_MSG_DEBUG,
    // Private: a frame passed framing but failed CRC/sanity. Never surfaced to
    // callers (the wrapper converts it back to P8_MSG_NONE); used internally to
    // distinguish "rejected a completed frame" from "still hunting for SYNC".
    P8_MSG_REJECT
};

struct P8Parser {
    P8ParseState state;
    uint16_t     idx;
    uint8_t      crc;
    uint8_t      telem_buf[sizeof(TelemFrame)];
    uint8_t      debug_buf[sizeof(DebugFrame)];

    uint8_t      ack_msg_id;
    uint16_t     ack_payload_len;
    uint8_t      ack_payload[16];
    P8MsgType    result_type;

    // Reframing buffer. The link has no length-prefixed framing, so a byte lost
    // mid-DebugFrame (RX overrun under load) slides the parser, and the 622-byte
    // payload almost always contains a spurious 0xAA 0x55 that traps re-sync at
    // the wrong offset. To recover, every byte consumed since the SYNC that
    // started the current frame is captured here; on a CRC/sanity failure the
    // frame is rejected, the leading SYNC1 is dropped, and the remaining bytes
    // are replayed so a real frame boundary hidden inside the bad window is
    // found instead of being skipped past.
    uint8_t      rebuf[8 + sizeof(DebugFrame)]; // largest frame + header/crc slack
    uint16_t     rebuf_len;
    bool         replaying;                      // guard: no capture during replay
};

void      p8_parser_init(P8Parser* p);
P8MsgType p8_parser_feed(P8Parser* p, uint8_t b);
const TelemFrame* p8_get_telem(const P8Parser* p);
const DebugFrame* p8_get_debug(const P8Parser* p);

// ============================================================
// Uplink command senders
// ============================================================
bool p8_send_cmd(uint8_t cmd_id, const uint8_t* payload = nullptr, uint16_t len = 0);
bool p8_send_set_mode(uint8_t mode_id);
bool p8_send_abort();
bool p8_send_heartbeat();
bool p8_send_pwm_set(uint8_t ch, uint16_t us);
bool p8_send_pwm_disarm();
bool p8_send_rtcm(const uint8_t* data, uint16_t len);
bool p8_send_debug_toggle();
// Emergency landing: PNG mode latches a fixed gentle-descent thrust. No payload.
bool p8_send_emergency_land();
// Auto takeoff: AUTO_TKO_LAND mode starts the climb sequence. No payload.
bool p8_send_auto_takeoff();
// Auto landing: AUTO_TKO_LAND mode starts the descent sequence from HOLD. No payload.
bool p8_send_auto_land();
// Mission start: MISSION mode hands off from HOLD to PNG guidance. No payload.
bool p8_send_mission_start();

// Upload a PNG/MISSION waypoint mission (absolute GPS). lat/lon are degrees,
// serialized as int32 1e-7 deg (LE) to match the MCU GNSS PVT scale. alt is the
// per-WP target altitude [m, takeoff-ground-relative], serialized as 32-bit LE
// float. flags is per-WP (reserved; pass nullptr to send all-zero). Each WP is
// 13 bytes. count <= PNG_MAX_WAYPOINTS.
bool p8_send_waypoints(const double* lat, const double* lon,
                       const float* alt, const uint8_t* flags, uint8_t count);

// ============================================================
// Display helpers
// ============================================================
const char* ack_result_str(uint8_t r);
const char* rtk_status_str(uint8_t flags);
const char* ctrl_source_str(uint8_t flags);
void print_telem(const TelemFrame* tf);
void print_debug(const DebugFrame* df);
void print_ack(uint8_t cmd_id, uint8_t result);
// AtlState (auto takeoff/landing) name for a state index (0..7).
const char* atl_state_name(uint8_t s);
