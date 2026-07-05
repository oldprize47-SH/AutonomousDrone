//
// Telem - Telemetry Communication (P8 USART3)
// Downlink: Fixed-length TelemFrame (76 bytes)
// Uplink:   Variable-length command frames
//

#ifndef TELEM_H
#define TELEM_H

#include <Arduino.h>
#include <stdint.h>
#include "Mode.h"

// ============================================================
// Frame Constants
// ============================================================

#define TELEM_SYNC1     0xAA
#define TELEM_SYNC2     0x55
#define TELEM_MAX_PAYLOAD 2048

// ============================================================
// Uplink CMD_ID (GCS -> MCU)
// ============================================================

#define CMD_SET_MODE    0x01
#define CMD_ABORT       0x02
#define CMD_PWM_SET     0x03
#define CMD_PWM_DISARM  0x04
#define CMD_RTCM_DATA   0x10
#define CMD_DEBUG_TOGGLE 0x20
#define CMD_SET_WAYPOINTS 0x30  // mission: [count][i32 lat,i32 lon,f32 alt,u8 flags]... (13B/WP; 1e-7 deg, alt m)
#define CMD_EMERGENCY_LAND 0x40 // PNG: latch U1 to EMLAND_THRUST_FRAC*mg, gentle descent (no payload)
#define CMD_AUTO_TAKEOFF 0x41   // AUTO_TKO_LAND/MISSION: start auto takeoff (no payload)
#define CMD_AUTO_LAND    0x42   // AUTO_TKO_LAND/MISSION: start auto landing from HOLD (no payload)
#define CMD_MISSION_GUIDE 0x43  // MISSION: start PNG guidance from HOLD (no payload)
#define CMD_HEARTBEAT   0xF0

// ============================================================
// Downlink MSG_ID (MCU -> GCS)
// ============================================================

#define MSG_ACK         0xF1
#define MSG_DEBUG       0xDB

// ============================================================
// ACK Result Codes
// ============================================================

#define ACK_OK              0x00
#define ACK_ERR_UNKNOWN_CMD 0x01
#define ACK_ERR_BAD_PARAM   0x02
#define ACK_ERR_WRONG_MODE  0x03
#define ACK_ERR_HW_FAIL     0x04

// ============================================================
// Status Flag Bits (for TelemFrame.status_flags)
// ============================================================

#define STATUS_BIT_IMU_OK       (1u << 0)
#define STATUS_BIT_MOTOR_ARMED  (1u << 1)
#define STATUS_BIT_RC_OK        (1u << 2)
#define STATUS_BIT_GNSS_FIX     (1u << 3)
#define STATUS_BIT_CTRL_SHIFT   4       // bits 4-5: ControlSource
#define STATUS_BIT_CTRL_MASK   0x30    // bit4-5
#define STATUS_RTK_SHIFT_TELEM  6
#define STATUS_RTK_MASK_BITS    0xC0  // bit6-7

// ============================================================
// Downlink Frame (fixed 76 bytes)
// ============================================================

typedef struct __attribute__((packed)) {
    // System (2 bytes)
    uint8_t  mode;
    uint8_t  status_flags;

    // Attitude (12 bytes)
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;

    // Position (20 bytes)
    double   lat_deg;
    double   lon_deg;
    float    alt_m;

    // Velocity ENU (12 bytes)
    float    vel_e_ms;
    float    vel_n_ms;
    float    vel_u_ms;

    // GNSS (2 bytes)
    uint8_t  gnss_num_sv;
    uint8_t  gnss_fix_type;

    // Battery (4 bytes, reserved)
    uint16_t batt1_mv;
    uint16_t batt2_mv;

    // RC input (10 bytes)
    uint16_t rc_ch[5];

    // Motor output (8 bytes)
    uint16_t motor_us[4];

    // Timestamp (4 bytes)
    uint32_t uptime_ms;

    // Yaw rate (4 bytes)
    float    yaw_rate_dps;

    // Reserved (2 bytes)
    uint8_t  reserved[2];
} TelemFrame;  // 80 bytes

// ============================================================
// Debug Frame (control loop logging)
// ============================================================

typedef struct __attribute__((packed)) {
    // ---- Group 1: attitude controller --------------------------------------
    // Attitude (rad) + body rates (rad/s) + setpoints.
    float    roll;
    float    pitch;
    float    yaw;
    float    p;
    float    q;
    float    r;
    float    roll_cmd;        // target roll  [rad] (applied setpoint)
    float    pitch_cmd;       // target pitch [rad] (applied setpoint)
    float    r_cmd;           // target yaw rate [rad/s]
    // Outer Angle PI -> inner rate setpoint, and LPF rate feedback used.
    float    roll_rate_cmd;   // outer Angle PI output (roll)  [rad/s]
    float    pitch_rate_cmd;  // outer Angle PI output (pitch) [rad/s]
    float    p_f;             // 35Hz-LPF roll rate feedback   [rad/s]
    float    q_f;             // 35Hz-LPF pitch rate feedback  [rad/s]
    // Outer Angle PI internals (error = (target+ff) - angle; live integrators).
    float    e_roll;          // outer PI roll error  [rad]
    float    e_pitch;         // outer PI pitch error [rad]
    float    int_e_roll;      // outer PI roll integrator state  [rad*s]
    float    int_e_pitch;     // outer PI pitch integrator state [rad*s]
    float    roll_ff;         // takeoff-trim feed-forward (roll)  [rad]
    float    pitch_ff;        // takeoff-trim feed-forward (pitch) [rad]
    // Yaw rate PID internals.
    float    e_r;             // yaw rate error [rad/s]
    float    int_e_r;         // yaw rate integrator state
    float    I_r;             // yaw rate I term
    // Mixer: torques (Nm) + per-motor thrust (N).
    float    U1, U2, U3, U4;

    // ---- Group 2: altitude controller --------------------------------------
    // NOTE: barometer/complementary-filter debug fields (alt_est, alt_baro,
    // free_acc_z, pressure_pa, w_down) were removed to shrink the frame. The
    // altitude estimator still runs on the MCU; only its DebugFrame logging is
    // dropped. GCS DebugFrame must mirror this removal byte-for-byte.
    float    throttle_N;      // total thrust U1 command [N]
    float    lidar_alt;       // body-tilt-comp LiDAR altitude [m] (0 if invalid)
    uint8_t  lidar_valid;     // 1 = LiDAR used as aiding source
    uint8_t  alt_hold;        // 1 = altitude-hold active (RC_CH_MODE AUTO)
    float    alt_cmd;         // hold setpoint [m] (up+, = alt_hold_h)
    float    alt_error;       // alt error [m] (NED down+, h_cmd-h)
    float    hdot_cmd;        // commanded vertical speed [m/s] (NED down+)
    float    DeltaT_cmd;      // thrust correction [N] (T_cmd = mg - DeltaT)
    float    h_fmf;           // FMF height   [m]   (up+)
    float    v_fmf;           // FMF velocity [m/s] (up+)
    float    a_fmf;           // FMF accel    [m/s^2] (Constant_Acc only; else 0)
    float    h_used;          // altitude feedback used  [m]   (up+)
    float    v_used;          // vertical-speed feedback [m/s] (up+)

    // ---- Group 3: velocity controller (body frame, X->u, Y->v) -------------
    // 0 outside the velocity/PNG controllers. fb=feedback, acc=free accel.
    float    u_cmd;           // X velocity command  [m/s] (body forward)
    float    u_fb;            // X velocity feedback [m/s] (body forward, GNSS)
    float    u_acc;           // X free acceleration [m/s^2] (body forward)
    float    u_err;           // X velocity error    [m/s] (u_cmd - u_fb)
    float    v_cmd;           // Y velocity command  [m/s] (body right)
    float    v_fb;            // Y velocity feedback [m/s] (body right, GNSS)
    float    v_acc;           // Y free acceleration [m/s^2] (body right)
    float    v_err;           // Y velocity error    [m/s] (v_cmd - v_fb)

    // ---- Group 4: PNG guidance ---------------------------------------------
    // 0 outside MODE_PNG_GUIDANCE / on GNSS-fix loss.
    float    png_px;          // local NED North position (origin-ref) [m]
    float    png_py;          // local NED East  position (origin-ref) [m]
    float    png_rng;         // horizontal distance to current waypoint [m]
    float    png_eta;         // heading error (LOS - yaw) [rad]
    float    png_los;         // line-of-sight angle to target (NED, North=0, East+) [rad]
    float    png_yaw_rate;    // commanded yaw rate [rad/s]
    uint8_t  wp_idx;          // current waypoint index
    // Raw GNSS PVT fix used by PNG this step (0 outside PNG / on fix loss). Lets
    // the GCS log absolute position so plots auto-align to the mission origin.
    double   png_lat;         // GNSS latitude  [deg]
    double   png_lon;         // GNSS longitude [deg]
    float    png_alt;         // GNSS height    [m] (ellipsoid)

    // ---- Group 4b: collision-cone obstacle avoidance -----------------------
    // 0 outside MODE_PNG_GUIDANCE / when no fresh H7 obstacle. See CollisionCone.
    float    obs_r_ca;        // avoidance yaw-rate command [rad/s]
    float    obs_Ro;          // nearest obstacle range [m] (0 = none)
    float    obs_eo;          // obstacle heading error [rad]
    float    obs_gamma;       // collision-cone half angle [rad]
    float    obs_theta_dot;   // cone-boundary angle rate [rad/s]
    uint8_t  obs_avoid;       // 1 = avoidance latch active

    // ---- Group 5: misc (timing, battery, GNSS/RTK) -------------------------
    uint32_t uptime_ms;
    uint16_t batt_mv;
    uint16_t loop_count_50hz; // # of 200Hz iterations in the 50Hz window (~4)
    float    loop_dt_max_ms;  // max 200Hz loop period in window [ms] (~5)
    uint8_t  rtk_status;      // 0=none, 1=float, 2=fixed
    uint8_t  gnss_fix;        // 1 = GNSS fix valid
    float    vel_n;           // GNSS/RTK fusion velocity, NED North [m/s]
    float    vel_e;           // GNSS/RTK fusion velocity, NED East  [m/s]
    float    vel_u;           // GNSS velocity rotated to body forward (u) [m/s]
    float    vel_v;           // GNSS velocity rotated to body right   (v) [m/s]

    // ---- Group 6: auto takeoff/landing (MODE_AUTO_TKO_LAND) -----------------
    // 0 / ATL_GROUND outside the auto-takeoff/landing mode. Appended at the end
    // so existing field offsets stay fixed (GCS decoder stays byte-compatible).
    uint8_t  atl_state;       // AtlState enum (0=GROUND 1=SPOOLUP 2=TAKEOFF
                              // 3=HOLD 4=LAND 5=LAND_SLOW 6=SPOOLDOWN 7=DISARMED)
    float    atl_climb_cmd;   // commanded climb rate [m/s, up+] (+climb, -descend)
    float    atl_hold_alt;    // captured altitude-hold setpoint [m, up+]
    uint16_t atl_phase_ms;    // ms remaining in spool-up/down phase (0 otherwise)

    // ---- Group 7: vision precision landing (RealSense helipad, ATL only) ----
    // Three layers, all in one place:
    //   (a) RAW camera input  : latest value received from the bridge, regardless
    //       of whether the controller used it (monitoring the camera link itself).
    //   (b) NED latch (used)  : the absolute NED point actually fed to the pos PID.
    //   (c) status flags      : raw valid / steering / lock / spike-reject.
    // Appended at the end so existing field offsets stay fixed (byte-compatible).
    // --- (a) RAW camera input (always the latest received, even if not steering) -
    float    helipad_x;       // raw body forward offset from camera [m] (+fwd)
    float    helipad_y;       // raw body right   offset from camera [m] (+right)
    float    helipad_xy;      // raw horizontal error sqrt(x^2+y^2) [m]
    float    helipad_gnd;     // raw drone-centre ground distance [m]
    uint8_t  helipad_valid;   // raw vision valid flag from the camera (1=valid)
    // --- (b) attitude-corrected NED offset (full roll/pitch/yaw DCM, pre-add) --
    float    helipad_dn;      // DCM-corrected NED north offset [m] (vs raw helipad_x)
    float    helipad_de;      // DCM-corrected NED east  offset [m] (vs raw helipad_y)
    // --- (c) NED latch actually used by the position PID -----------------------
    float    vlatch_n;        // latched helipad target, NED north [m] (0 if no lock)
    float    vlatch_e;        // latched helipad target, NED east  [m] (0 if no lock)
    // --- (d) controller status -------------------------------------------------
    uint8_t  vision_use;      // 1 = steering to the latched helipad point this step
    uint8_t  vlatch_set;      // 1 = a helipad point is latched (lock acquired)
    uint8_t  vision_reject;   // 1 = this step's fix was rejected as a spike (jump)

    // ---- Group 8: MISSION waypoint altitude command (MODE_MISSION only) -------
    // The ramped target altitude the vertical alt-hold is tracking toward the
    // active WP (linear interpolation across the segment). Equals atl_hold_alt
    // outside GUIDANCE. Appended at the end so existing offsets stay fixed.
    float    wp_alt_cmd;      // ramped WP altitude setpoint [m, takeoff-ground-relative]

    // ---- Group 9: MISSION waypoint local-NED cache (MODE_MISSION only) --------
    // The uploaded waypoints converted to local NED at mode entry (origin =
    // takeoff GNSS fix). STATIC for the whole flight (computed once), so the same
    // values repeat every frame; only the first wp_ned_count entries are valid,
    // the rest are 0. Lets the GCS verify/plot the NED mission geometry that the
    // 200Hz guidance loop actually steers to. Appended last (byte-compatible).
    uint8_t  wp_ned_count;                 // # of valid waypoints (= png_wp_size)
    float    wp_north[PNG_MAX_WAYPOINTS];  // local-NED North per WP [m] (origin-ref)
    float    wp_east [PNG_MAX_WAYPOINTS];  // local-NED East  per WP [m] (origin-ref)
} DebugFrame;  // sizeof auto-calculated

// Send debug frame (replaces telem_send_frame for control debugging)
void telem_send_debug(Stream &serial, const DebugFrame &df);

// ============================================================
// Uplink Parsed Message
// ============================================================

typedef struct {
    uint8_t  cmd_id;
    uint16_t payload_len;
    uint8_t  payload[TELEM_MAX_PAYLOAD];
} TelemCmd;

// ============================================================
// Uplink Parser
// ============================================================

class TelemParser {
public:
    TelemParser();
    void reset();
    bool feedByte(uint8_t b);
    const TelemCmd& getCmd() const { return _cmd; }

private:
    enum State { S_SYNC1, S_SYNC2, S_CMD_ID, S_LEN_H, S_LEN_L, S_PAYLOAD, S_CRC };
    State    _state;
    TelemCmd _cmd;
    uint16_t _idx;
    uint8_t  _crc;
};

// ============================================================
// Public API
// ============================================================

// Send ACK response
void telem_send_ack(Stream &serial, uint8_t cmd_id, uint8_t result);
void telem_send_ack_ext(Stream &serial, uint8_t cmd_id, uint8_t result,
                        const uint8_t *extra, uint8_t extra_len);

// Build and send downlink TelemFrame
void telem_send_frame(Stream &serial);

// Handle parsed uplink command
void telem_handle_cmd(const TelemCmd &cmd);

#endif // TELEM_H
