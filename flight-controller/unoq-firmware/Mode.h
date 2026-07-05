//
// Mode - Flight Mode Management and Mode-specific Loops
//

#ifndef MODE_H
#define MODE_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================
// Flight Modes
// ============================================================

enum FlightMode : uint8_t {
    MODE_IDLE       = 0x00,
    MODE_RC_CONTROL = 0x01,
    MODE_RTK_MAIN   = 0x02,
    MODE_PWM_TEST   = 0x03,
    MODE_MTI_TEST   = 0x04,
    MODE_ESC_CAL    = 0x05,
    MODE_VEL_CONTROL = 0x06,  // accel-sensor velocity control (P8 CMD_SET_MODE only)
    MODE_PNG_GUIDANCE = 0x07, // PNG waypoint guidance over RTK-GNSS (P8 CMD_SET_MODE only)
    MODE_POS_CONTROL = 0x08,  // RTK-GNSS position control (moving setpoint, P8 CMD_SET_MODE only)
    MODE_AUTO_TKO_LAND = 0x09, // auto takeoff/landing (climb-rate + takeoff-point hold, P8 only)
    MODE_MISSION    = 0x0A,    // auto takeoff + PNG guidance + auto land (P8 only)
    MODE_COUNT
};

// ============================================================
// Control Source (priority: EMERGENCY > MANUAL > AUTO)
// ============================================================

enum ControlSource : uint8_t {
    CTRL_EMERGENCY = 0,  // RC CH5 >= 1500us: force motor stop
    CTRL_MANUAL    = 1,  // RC stick direct control
    CTRL_AUTO      = 2,  // Telemetry/mission command control
};

// Determine control source from RC inputs
ControlSource get_control_source();

// IMU attitude calibration offsets (read-only access)
float get_roll_offset();
float get_pitch_offset();

// Unified motor output with EMERGENCY override
// All motor writes MUST go through this function.
void set_motor_output(uint16_t us0, uint16_t us1, uint16_t us2, uint16_t us3);

// ============================================================
// Mode Management
// ============================================================

const char* mode_name(FlightMode m);
void enter_mode(FlightMode mode);
void exit_current_mode();
void imu_setup();  // IMU init + attitude/baro calibration (called from setup())

// ============================================================
// PNG Waypoint Mission (uploaded from GCS via CMD_SET_WAYPOINTS)
// ============================================================

#define PNG_MAX_WAYPOINTS  32  // max waypoints per uploaded PNG mission

// Store an uploaded PNG mission. buf holds `count` waypoints, each 8 bytes:
// int32 lat, int32 lon (little-endian, 1e-7 deg). Lat/lon are kept raw and
// converted to local NED at PNG mode entry (origin = entry GNSS fix).
// Resets png_wp_idx/png_is_end. count==0 clears the mission.
void png_set_waypoints(const uint8_t* buf, uint8_t count);

// ============================================================
// Auto takeoff/landing request gate (queried by Telem on GCS command)
// ============================================================
// True only while the AUTO_TKO_LAND state machine is on the ground and ready to
// accept a takeoff (GNSS 3D fix + IMU + low LiDAR alt + RC AUTO). The actual
// trigger flag is atl_takeoff_request (sketch.ino), consumed by the loop.
bool atl_can_takeoff();
// True only while the AUTO_TKO_LAND state machine is holding altitude (after a
// completed takeoff), i.e. a landing command is valid.
bool atl_in_hold();

// ============================================================
// Vision precision-landing target (RealSense helipad)
// ============================================================
// Called from the Bridge "set_helipad_target" RPC (Linux side relays the
// RealSense track_helipad.py output). Copy-only: stores the latest helipad
// offset (body FRD, drone-centre relative) + freshness timestamp. The ATL
// landing loop reads these under a stale/range/valid gate and rotates the
// offset into a NED setpoint for the position PID. All numeric args are double
// (matches the MsgPack RPC decoding used by the existing set_cam_alt handler);
// `valid` is passed as int (0/1) for unambiguous MsgPack typing.
void set_helipad_target(int valid, double x_m, double y_m,
                        double xy_m, double gnd_m);

// ============================================================
// Mode Loops
// ============================================================

void loop_idle();
void loop_rc_control();
void loop_vel_control();
void loop_png_guidance();
void loop_pos_control();
void loop_auto_tko_land();
void loop_mission();
void loop_rtk_main();
void loop_pwm_test();
void loop_mti_test();
void loop_esc_cal();

// ============================================================
// PWM_TEST state (used by Telem and DebugConsole)
// ============================================================

extern int      pwm_selected_motor;
extern uint16_t pwm_motor_us[4];

#endif // MODE_H
