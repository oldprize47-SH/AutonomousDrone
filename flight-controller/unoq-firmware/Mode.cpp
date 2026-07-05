//
// Mode - Flight Mode Management and Mode-specific Loops
//

#include "Mode.h"
#include "INSS.h"
#include "QuadPWM.h"
#include "RCInput.h"
#include "PIDControl.h"
#include "Autopilot.h"
#include "RealTime.h"
#include "Telem.h"
#include "hw_config.h"
#include "filter.h"
#include "LidarTF.h"
#include <string.h>   // memset (DebugFrame zero-init)
#if USE_OBS_AVOID
#include "ObstacleRx.h"
#include "CollisionCone.h"
#endif
#if USE_BRIDGE
#include <Arduino_RouterBridge.h>  // for Bridge.update() in the RC tight loop
#endif

// External references (defined in sketch.ino)
extern FlightMode   current_mode;
extern Usart3Serial P8Serial;
extern INSS         ins;
extern QuadPWM      motors;
extern LidarTF      lidar;
extern bool         ins_active;
extern bool         motors_active;
extern uint16_t    batt_main_mv;
extern bool         debug_output_enabled;
extern TelemParser  p8_parser;
extern uint16_t     batt_sub_mv;
extern bool         rc_auto_inhibit;
extern volatile bool emland_active;

#if USE_OBS_AVOID
extern ObstacleRx   obstacle;   // H7 obstacle slave reader (sketch.ino)
// Collision-cone avoidance state (PNG only). cc owns the avoid-mode latch;
// g_ccd caches the last step's diagnostics for the DebugFrame.
static CollisionCone cc;
static CCDebug       g_ccd = {};
#endif

// ============================================================
// PWM_TEST state
// ============================================================

int      pwm_selected_motor = 0;
uint16_t pwm_motor_us[4];

// IMU watchdog
static uint32_t last_imu_data_ms = 0;
#define IMU_TIMEOUT_MS  500  // no IMU data for 500ms -> motor stop

// IMU level calibration offset (captured at RC_CONTROL entry)
static float roll_offset  = 0.0f;
static float pitch_offset = 0.0f;

// Altitude estimator
static FilterState  alt_filter;
static FilterGains  alt_gains;
static BaroCal      baro_cal;
static float        alt_baro_latest = 0.0f;
static bool         alt_ready = false;

#if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
// Fading Memory Filter on the LiDAR altitude (up-positive). Estimates height
// and vertical speed from the single LiDAR signal; outputs (h_fmf/v_fmf) feed
// the altitude controller. a_fmf is logged only (Constant_Acc analysis).
static FadingMemFilter alt_fmf;
static bool   fmf_ready = false;   // seeded with first valid LiDAR altitude
static float  h_fmf = 0.0f;        // up-positive [m]
static float  v_fmf = 0.0f;        // up-positive [m/s]
static float  a_fmf = 0.0f;        // up-positive [m/s^2] (Constant_Acc only)
#endif

// Active altitude estimate (up+ [m]) for the alt-hold controller feedback.
// Mirrors the compile-time source selection used in the control branch below
// so the alt-hold setpoint is captured from the SAME source as the feedback,
// giving a bump-free hand-off. lidar_alt_m is a per-iteration value, so the
// caller passes it in (used only by the LiDAR-direct path).
static inline float alt_hold_active_h(float lidar_alt_m)
{
#if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
    (void)lidar_alt_m;
    return h_fmf;
#elif (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && ALT_LIDAR_DIRECT
    return lidar_alt_m;
#else
    (void)lidar_alt_m;
    return alt_filter.h;
#endif
}

// MTi XKF3 fusion altitude (ellipsoid, RTK-precise outdoors) @50Hz; ZOH at 200Hz
static float        alt_fusion_latest = 0.0f;  // ALT_ELLIPSOID [m], up+ WGS84

// Altitude-hold (AUTO via RC_CH_MODE): captured hold setpoint (up+ [m])
static bool         alt_hold_active = false;   // edge tracker for toggle ON
static float        alt_hold_h      = 0.0f;    // alt_filter.h captured at toggle ON

// MTi velocity (data[VEL_Z]) is NED down+ from the on-board GNSS/INS fusion
// at 50Hz; the 200Hz control loop keeps the last value (ZOH) between updates.
static float        vel_down_latest = 0.0f;    // w feedback [m/s], NED down+

float get_roll_offset()  { return roll_offset; }
float get_pitch_offset() { return pitch_offset; }

// ============================================================
// PNG Guidance (MODE_PNG_GUIDANCE) state
// ============================================================
//
// Waypoints are hardcoded in the local NED frame (meters), referenced to the
// origin captured at mode entry (current GNSS LLA). X=North, Y=East, Z=Down.
// Only the horizontal (N,E) components are used for PNG; altitude is handled by
// the shared alt-hold logic (RC throttle).

// PNG_MAX_WAYPOINTS is defined in Mode.h (shared with Telem.cpp).

// PNG mission waypoints. Uploaded from the GCS as absolute GPS lat/lon (1e-7 deg)
// via CMD_SET_WAYPOINTS, and stored raw here. They are converted to local NED
// (png_wp_north/east) once at PNG mode entry, after the origin is captured from
// the entry GNSS fix (so the mission stays correct regardless of takeoff point).
static int32_t png_wp_lat[PNG_MAX_WAYPOINTS];  // uploaded latitude  [1e-7 deg]
static int32_t png_wp_lon[PNG_MAX_WAYPOINTS];  // uploaded longitude [1e-7 deg]
static float   png_wp_alt[PNG_MAX_WAYPOINTS];  // uploaded altitude  [m, takeoff-ground-relative]
static uint8_t png_wp_flags[PNG_MAX_WAYPOINTS]; // per-WP flags (reserved, unused for now)
static int     png_wp_size = 0;                // valid waypoints (0 until uploaded)

// Local-NED cache (origin-referenced) built at PNG entry from png_wp_lat/lon.
// The 200Hz guidance loop reads these; never written outside enter_mode().
static float png_wp_north[PNG_MAX_WAYPOINTS];  // [m] North (origin-ref)
static float png_wp_east [PNG_MAX_WAYPOINTS];  // [m] East  (origin-ref)
// Horizontal segment length into each WP (dist from the previous WP, or from the
// takeoff origin for WP0). Used by the MISSION altitude ramp. Precomputed at entry.
static float png_wp_seg_len[PNG_MAX_WAYPOINTS]; // [m] horizontal length of segment ending at WP[i]

// Runtime guidance state
static int   png_wp_idx   = 0;          // current target waypoint index
static int   png_wp_prev_idx = -1;      // last PASSED waypoint index (-1 = none yet)
                                        // gates MISSION avoidance on the post-WP segment
static bool  png_is_end   = false;      // reached final waypoint
static bool  png_origin_set = false;    // LLA origin captured at entry

// ---- Orbit / U-turn guidance state (WP_FLAG_ORBIT, inside PNG) ----
// Set when a WP carrying WP_FLAG_ORBIT is captured: the craft orbits the
// midpoint of that WP and the next WP (png_wp_idx) until it reaches the next
// WP's capture radius, then resumes PNG straight guidance.
static bool  png_orbit_active = false;       // orbit (U-turn) guidance in progress
static float png_orbit_cN     = 0.0f;        // orbit centre North [m] (origin-ref)
static float png_orbit_cE     = 0.0f;        // orbit centre East  [m] (origin-ref)
static float png_orbit_r      = ORBIT_R_MIN; // orbit radius [m]
static float png_orbit_turn   = 0.0f;        // accumulated |yaw turn| since entry [rad]
static float png_orbit_yawprev = 0.0f;       // previous yaw sample [rad] (turn integrator)
// Orbit direction is auto-selected at entry: eta_cmd = +/-90 deg chosen so the
// craft turns toward whichever side the centre is on at the moment of entry
// (the stable equilibrium is eta == eta_cmd, i.e. the centre stays on that side).
//   centre on the LEFT  at entry (eta0 >= 0) -> eta_cmd = +PI/2 -> turns LEFT
//   centre on the RIGHT at entry (eta0 <  0) -> eta_cmd = -PI/2 -> turns RIGHT
// So the WP layout / approach heading chooses the direction. Latched for the turn.
static float png_orbit_eta_cmd = PI * 0.5f;  // eta_cmd [rad], set at orbit entry
// Altitude ramp anchors (MISSION GUIDANCE): the altitude the craft was at when
// it left the previous WP, ramped toward png_wp_alt[png_wp_idx] over the segment.
static float png_alt_prev = 0.0f;       // [m] ramp start altitude for the active segment

// ---- Vision precision-landing target (RealSense helipad) ----
// Written ONLY by set_helipad_target() (Bridge RPC context, ~50Hz). Read by the
// ATL landing loop (200Hz). volatile for the cross-context single-word reads;
// the float words are updated independently so the loop tolerates a torn read
// (next sample corrects it). Body FRD: x=forward, y=right (drone-centre rel).
static volatile bool     helipad_valid_raw = false;
static volatile float    helipad_x_m  = 0.0f;   // forward offset to helipad [m]
static volatile float    helipad_y_m  = 0.0f;   // right   offset to helipad [m]
static volatile float    helipad_xy_m = 0.0f;   // sqrt(x^2+y^2) horiz error [m]
static volatile float    helipad_gnd_m = 0.0f;  // drone-centre ground distance [m]
static volatile uint32_t helipad_ms  = 0;       // millis() of last fresh fix

// Bridge RPC sink: copy-only (no heavy work in the callback). The landing loop
// owns coordinate rotation / gating. See Mode.h for the contract.
void set_helipad_target(int valid, double x_m, double y_m,
                        double xy_m, double gnd_m)
{
    helipad_x_m   = (float)x_m;
    helipad_y_m   = (float)y_m;
    helipad_xy_m  = (float)xy_m;
    helipad_gnd_m = (float)gnd_m;
    helipad_valid_raw = (valid != 0);
    helipad_ms    = millis();
}

// Captured LLA origin (ECEF) + precomputed rotation trig for ECEF->NED.
static double png_X0 = 0.0, png_Y0 = 0.0, png_Z0 = 0.0;  // origin ECEF [m]
static double png_sin_lat0 = 0.0, png_cos_lat0 = 1.0;
static double png_sin_lon0 = 0.0, png_cos_lon0 = 1.0;

// WGS84 ellipsoid constants
#define WGS84_A    6378137.0            // semi-major axis [m]
#define WGS84_E2   6.69437999014e-3     // first eccentricity squared

// Convert geodetic (lat,lon in deg; h ellipsoid [m]) to ECEF [m]. Uses sqrt
// only (Zephyr nano libc safe; no powf/fmodf/floorf).
static void lla_to_ecef(double lat_deg, double lon_deg, double h_m,
                        double &X, double &Y, double &Z)
{
    double lat = lat_deg * (double)DEG_TO_RAD;
    double lon = lon_deg * (double)DEG_TO_RAD;
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double sin_lon = sin(lon), cos_lon = cos(lon);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    X = (N + h_m) * cos_lat * cos_lon;
    Y = (N + h_m) * cos_lat * sin_lon;
    Z = (N * (1.0 - WGS84_E2) + h_m) * sin_lat;
}

// Capture the current LLA as the local-NED origin (called at mode entry).
static void png_capture_origin(double lat_deg, double lon_deg, double h_m)
{
    double lat = lat_deg * (double)DEG_TO_RAD;
    double lon = lon_deg * (double)DEG_TO_RAD;
    png_sin_lat0 = sin(lat); png_cos_lat0 = cos(lat);
    png_sin_lon0 = sin(lon); png_cos_lon0 = cos(lon);
    lla_to_ecef(lat_deg, lon_deg, h_m, png_X0, png_Y0, png_Z0);
    png_origin_set = true;
}

// Convert current LLA to local NED (origin-referenced). Outputs north/east [m].
static void png_lla_to_ned(double lat_deg, double lon_deg, double h_m,
                           float &north, float &east)
{
    double X, Y, Z;
    lla_to_ecef(lat_deg, lon_deg, h_m, X, Y, Z);
    double dX = X - png_X0, dY = Y - png_Y0, dZ = Z - png_Z0;
    north = (float)(-png_sin_lat0 * png_cos_lon0 * dX
                    - png_sin_lat0 * png_sin_lon0 * dY
                    + png_cos_lat0 * dZ);
    east  = (float)(-png_sin_lon0 * dX + png_cos_lon0 * dY);
}

// Rotate a LEVEL-body horizontal offset (helipad vision: xb=forward, yb=right,
// implicit zb=0) into the local-NED frame using the FULL roll/pitch/yaw DCM.
// Single source of truth shared by the ground-test debug path and the flight
// latch path so the two can never diverge.
//
// Sign convention (INSS.h): roll +right-wing-down, pitch +nose-DOWN,
// yaw +CW-from-North. The standard NED body->nav DCM assumes pitch +nose-UP,
// so the pitch input is negated (th = -pitch_rad) before applying the formula.
// Only the N/E rows of v_n = R(phi,theta,psi) * [xb, yb, 0]^T are returned.
//   phi=theta=0 reduces to the yaw-only rotation (dn=xb*cy-yb*sy, de=xb*sy+yb*cy).
static inline void helipad_body_to_ned(float xb, float yb,
                                       float roll_rad, float pitch_rad,
                                       float yaw_rad,
                                       float &dn, float &de)
{
    const float ph = roll_rad;      // +right-wing-down
    const float th = -pitch_rad;    // code +nose-down -> std +nose-up
    const float ps = yaw_rad;       // +CW from North
    const float sph = sinf(ph), cph = cosf(ph);
    const float sth = sinf(th), cth = cosf(th);
    const float sps = sinf(ps), cps = cosf(ps);
    dn = xb * (cth * cps) + yb * (sph * sth * cps - cph * sps);
    de = xb * (cth * sps) + yb * (sph * sth * sps + cph * cps);
}

// Store an uploaded PNG mission (GCS -> CMD_SET_WAYPOINTS). buf holds `count`
// waypoints, each 13 bytes (little-endian):
//   int32 lat (1e-7 deg), int32 lon (1e-7 deg), float alt (m, ground-relative),
//   uint8 flags (reserved). Lat/lon/alt are kept raw; NED conversion happens at
//   PNG/MISSION entry. count==0 clears.
void png_set_waypoints(const uint8_t* buf, uint8_t count)
{
    if (count > PNG_MAX_WAYPOINTS) count = PNG_MAX_WAYPOINTS;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t* p = buf + (size_t)i * 13;
        png_wp_lat[i] = (int32_t)((uint32_t)p[0]        | ((uint32_t)p[1] << 8) |
                                  ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
        png_wp_lon[i] = (int32_t)((uint32_t)p[4]        | ((uint32_t)p[5] << 8) |
                                  ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24));
        // float alt: bytes 8..11, little-endian. MCU (Cortex-M33) and the x86 GCS
        // are both little-endian IEEE-754, so reassemble the 32-bit pattern and
        // bit-cast it to float (no string.h dependency, no aliasing UB).
        uint32_t alt_bits = (uint32_t)p[8]        | ((uint32_t)p[9]  << 8) |
                            ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
        union { uint32_t u; float f; } alt_cvt;
        alt_cvt.u = alt_bits;
        png_wp_alt[i] = alt_cvt.f;
        png_wp_flags[i] = p[12];
    }
    png_wp_size = count;
    png_wp_idx  = 0;
    png_wp_prev_idx = -1;
    png_is_end  = false;
}

// Result of one PNG guidance step (proportional navigation, horizontal plane).
// The control loop feeds yaw_rate + u_cmd into the attitude/velocity controllers;
// the rest are telemetry. All angles [rad], ranges [m], speed [m/s].
struct PngGuidance {
    float yaw_rate;   // commanded yaw rate (PNG_KP*Vx*sin(eta)/rng), 0 at end
    float u_cmd;      // forward velocity command (PNG_VX_MS, 0 at end)
    float px, py;     // current local-NED position (origin-ref) [m]
    float rng;        // horizontal distance to current waypoint [m]
    float eta;        // heading error (LOS - yaw) [rad]
    float los;        // line-of-sight angle to target (NED, North=0, East+) [rad]
    float alt_cmd;    // (MISSION) ramped target altitude for this segment [m, up+]
    bool  orbit_active; // true while orbiting (U-turn); caller disables avoidance
};

// Proportional-navigation guidance for the current waypoint (horizontal only).
//   lat_deg/lon_deg/h_m : current GNSS PVT fix
//   yaw_rad             : current heading (RAW NED, from ctrl_read_sensors)
//   alt_now_m           : current alt-hold altitude [m, takeoff-ground-relative],
//                         used to re-anchor the ramp start on WP capture so the
//                         next segment blends from the ACTUAL altitude (no setpoint
//                         jump if the previous segment's climb/descent lagged).
// Returns yaw_rate / u_cmd to steer toward png_wp_north/east[png_wp_idx], and
// advances png_wp_idx / png_is_end when the waypoint capture radius is reached.
// Pure guidance math (mirrors the reference PNG implementation); the attitude
// and throttle controllers stay in the caller.
//   v_fwd : measured body forward velocity [m/s] (gnss_vel_u). The straight PN
//           law uses this actual speed instead of the fixed command PNG_VX_MS so
//           the yaw-rate matches the real closing speed. The orbit law keeps the
//           command speed (its steady-radius INVARIANT requires V == u_cmd).
static PngGuidance png_compute_guidance(double lat_deg, double lon_deg, double h_m,
                                        float yaw_rad, float alt_now_m, float v_fwd)
{
    PngGuidance g;
    g.orbit_active = png_orbit_active;

    // Current position in local NED (origin = PNG entry fix).
    png_lla_to_ned(lat_deg, lon_deg, h_m, g.px, g.py);

    // ---- Orbit / U-turn guidance (WP_FLAG_ORBIT) ----
    // While orbiting, steer around the captured centre with the orbit yaw-rate law
    // instead of the PN waypoint law. Forward speed stays PNG_VX_MS. The orbit ends
    // when the next straight waypoint (png_wp_idx) is within the capture radius, at
    // which point PNG straight guidance resumes toward that same waypoint.
    if (png_orbit_active) {
        float dcN = png_orbit_cN - g.px;
        float dcE = png_orbit_cE - g.py;
        g.rng     = hypotf(dcN, dcE);            // distance to orbit centre (telemetry)
        float los_c = atan2f(dcE, dcN);          // LOS to centre (NED)
        g.los     = los_c;                        // LOS to orbit centre (telemetry)
        g.eta     = wrap_pi(los_c - yaw_rad);    // heading error vs centre LOS

        g.u_cmd    = PNG_VX_MS;                   // forward speed command (unchanged)
        // (MISSION) Altitude ramp during the orbit: blend png_alt_prev (altitude
        // at orbit entry) toward the next WP's altitude over the half-turn, using
        // the accumulated heading change as progress (0 at entry, 1 at the
        // ORBIT_EXIT_TURN_DEG half-turn). Without this the alt-hold setpoint would
        // step straight to png_wp_alt[png_wp_idx] the instant the orbit begins
        // (a 2->3 m jump when the next WP changes altitude). PNG-standalone mode
        // holds a fixed altitude, so this only affects the MISSION caller that
        // feeds g.alt_cmd into the vertical sequencer.
        {
            float turn_full = ORBIT_EXIT_TURN_DEG * DEG_TO_RAD;
            float oprog = (turn_full > 0.0f) ? (png_orbit_turn / turn_full) : 1.0f;
            if (oprog < 0.0f) oprog = 0.0f;
            if (oprog > 1.0f) oprog = 1.0f;
            float alt_tgt = png_wp_alt[png_wp_idx];
            g.alt_cmd = png_alt_prev + (alt_tgt - png_alt_prev) * oprog;
        }

        float r_den = (png_orbit_r < ORBIT_R_MIN) ? ORBIT_R_MIN : png_orbit_r;
        const float eta_cmd = png_orbit_eta_cmd;   // +/-90 deg, auto-selected at entry
        // psi_dot = -(V/r) sin(eta_cmd) - K2 (V/r) sin(eta - eta_cmd).
        // INVARIANT: the speed term V in the law MUST equal the commanded forward
        // speed (g.u_cmd), otherwise the steady orbit radius (= V/|psi_dot|) no
        // longer converges to r_cmd. Tie them together so changing the forward
        // speed automatically keeps the radius correct.
        const float v_law = g.u_cmd;
        g.yaw_rate = -(v_law / r_den) * sinf(eta_cmd)
                     - ORBIT_K2 * (v_law / r_den) * sinf(g.eta - eta_cmd);

        // Accumulate the heading change since orbit entry (half-turn exit).
        png_orbit_turn += fabsf(wrap_pi(yaw_rad - png_orbit_yawprev));
        png_orbit_yawprev = yaw_rad;

        // Orbit exit: next straight WP within capture radius OR a half-turn done.
        float dnN = png_wp_north[png_wp_idx] - g.px;
        float dnE = png_wp_east [png_wp_idx] - g.py;
        bool wp_near = (hypotf(dnN, dnE) < PNG_WP_CAPTURE_R);
        bool turned  = (png_orbit_turn >= ORBIT_EXIT_TURN_DEG * DEG_TO_RAD);
        if (wp_near || turned) {
            png_orbit_active = false;             // resume PNG straight guidance
            g.orbit_active   = false;
            // Re-anchor the straight-segment altitude ramp to the craft's ACTUAL
            // current altitude on orbit exit. The orbit may end before the alt
            // ramp finished (e.g. still at 2.76 m heading for a 3 m target); the
            // straight ramp below blends png_alt_prev -> png_wp_alt[idx] by
            // (1 - rng/seg). Without re-anchoring, png_alt_prev still holds the
            // pre-orbit altitude (2 m) and a partial progress yields an alt_cmd
            // BELOW the altitude already reached -> the setpoint jumps DOWN at
            // orbit exit (the 3.0 -> 2.5 m drop seen in flight). Anchoring on the
            // real altitude makes the straight ramp continue UP from here toward
            // the same target with no backward step.
            png_alt_prev = alt_now_m;
        }
        return g;
    }

    // Line-of-sight to the active waypoint.
    float dN  = png_wp_north[png_wp_idx] - g.px;
    float dE  = png_wp_east [png_wp_idx] - g.py;
    g.rng     = hypotf(dN, dE);
    float los = atan2f(dE, dN);             // NED: North=0, East=+
    g.los     = los;                        // LOS to active waypoint (telemetry)
    g.eta     = wrap_pi(los - yaw_rad);     // heading error

    // PN law: yaw_rate = KP * V * sin(eta) / range (range floored for no /0).
    // V is the MEASURED body forward speed (v_fwd), so the yaw-rate tracks the
    // real closing speed rather than the fixed command. No low-speed clamp: when
    // v_fwd ~ 0 the yaw-rate goes to 0 (PN-consistent). Forward speed command
    // (g.u_cmd) stays PNG_VX_MS; both zeroed once the final WP is reached.
    float rng_den = (g.rng < PNG_RNG_MIN) ? PNG_RNG_MIN : g.rng;
    g.yaw_rate = png_is_end ? 0.0f : PNG_KP * v_fwd * sinf(g.eta) / rng_den;
    g.u_cmd    = png_is_end ? 0.0f : PNG_VX_MS;

    // (MISSION) Altitude ramp: blend from png_alt_prev (altitude at the last WP /
    // takeoff) toward the active WP's altitude as the craft closes the segment.
    // progress = 1 - rng/seg_len, clamped [0,1] (0 = just left prev WP, 1 = at WP).
    // At the final WP we hold the final altitude (progress pinned to 1).
    {
        float seg = png_wp_seg_len[png_wp_idx];      // >= PNG_RNG_MIN (precomputed)
        float progress = 1.0f - (g.rng / seg);
        if (progress < 0.0f) progress = 0.0f;
        if (progress > 1.0f) progress = 1.0f;
        float alt_tgt = png_wp_alt[png_wp_idx];
        g.alt_cmd = png_is_end ? alt_tgt
                               : png_alt_prev + (alt_tgt - png_alt_prev) * progress;
    }

    // Waypoint capture (horizontal distance only) -> advance / latch end.
    // On advance, re-anchor the ramp start to the craft's ACTUAL current altitude
    // (not the reached WP's target). If the previous segment's climb/descent did
    // not finish before the WP was captured horizontally, the next segment ramps
    // smoothly from where we actually are instead of jumping the alt setpoint to
    // the (un-reached) previous WP target. The next WP's altitude is still the
    // ramp end, so the craft just continues toward the new target from here.
    if (!png_is_end && g.rng < PNG_WP_CAPTURE_R) {
        int reached = png_wp_idx;       // the WP we just captured
        png_alt_prev = alt_now_m;       // anchor on actual altitude, not WP target
        png_wp_prev_idx = png_wp_idx;   // record the WP we just passed (gates next-seg avoidance)
        png_wp_idx++;
        if (png_wp_idx >= png_wp_size) {
            png_wp_idx = png_wp_size - 1;
            png_is_end = true;
        }

        // Orbit entry: if the WP we just reached carries WP_FLAG_ORBIT and a next
        // WP exists, orbit the midpoint of reached/next with radius = dist/2.
        // Guidance then steers around that centre until the next WP capture radius.
        if ((png_wp_flags[reached] & WP_FLAG_ORBIT) && !png_is_end) {
            float aN = png_wp_north[reached],     aE = png_wp_east[reached];
            float bN = png_wp_north[png_wp_idx],  bE = png_wp_east[png_wp_idx];
            png_orbit_cN = 0.5f * (aN + bN);
            png_orbit_cE = 0.5f * (aE + bE);
            float r = 0.5f * hypotf(bN - aN, bE - aE);
            png_orbit_r = (r < ORBIT_R_MIN) ? ORBIT_R_MIN : r;
            // Turn direction FIXED to left (CCW): eta_cmd = +PI/2 always keeps the
            // orbit centre on the craft's left, so the orbit law's leading term
            // -(V/r)*sin(eta_cmd) = -(V/r) yields a NEGATIVE yaw-rate (left turn,
            // CCW circle) regardless of approach heading. (Previously auto-selected
            // eta_cmd = sign(eta0)*PI/2 from the side the centre was on at entry,
            // which let some orbits turn right; that is intentionally disabled here.)
            png_orbit_eta_cmd = (PI * 0.5f);   // always left turn (yaw_rate < 0, CCW)
            png_orbit_yawprev = yaw_rad;   // turn-accumulator reference
            png_orbit_turn    = 0.0f;
            png_orbit_active  = true;
            g.orbit_active    = true;
        }
    }
    return g;
}

// ============================================================
// Position control (MODE_POS_CONTROL) - step setpoint + arrival state
// ============================================================
//
// Shares the PNG waypoint NED cache (png_wp_north/east), origin capture and
// png_wp_size. POS_CONTROL targets the active waypoint DIRECTLY (step setpoint,
// no interpolation); the position PID's dedicated tilt limit (POS_MAX_TILT_DEG)
// keeps the lean bounded even when the position error is large. A waypoint is
// reached when the craft's actual horizontal distance to it < POS_ARRIVE_R,
// then the next waypoint is targeted (final WP -> in-place hold).

static int   pos_wp_idx = 0;        // active target waypoint index
static bool  pos_is_end = false;    // reached final waypoint (hold there)

// Result of one position step: the target waypoint (step setpoint) and the
// craft's current horizontal distance to it.
struct PosTarget {
    float sp_n, sp_e;   // active waypoint, local NED (origin-ref) [m]
    float rng;          // craft's horizontal distance to the waypoint [m]
};

// Target the active waypoint directly and advance once the craft is within
// POS_ARRIVE_R of it. pos_n/pos_e = current local-NED position [m]. Latches
// pos_is_end at the final waypoint (target pinned there -> in-place hold).
static PosTarget pos_update_target(float pos_n, float pos_e)
{
    PosTarget o;
    o.sp_n = png_wp_north[pos_wp_idx];
    o.sp_e = png_wp_east [pos_wp_idx];

    float dN = o.sp_n - pos_n;
    float dE = o.sp_e - pos_e;
    o.rng = sqrtf(dN * dN + dE * dE);

    // Arrival = actual craft distance within the radius -> advance / latch end.
    if (!pos_is_end && o.rng < POS_ARRIVE_R) {
        pos_wp_idx++;
        if (pos_wp_idx >= png_wp_size) {
            pos_wp_idx = png_wp_size - 1;
            pos_is_end = true;
        }
    }
    return o;
}

// ============================================================
// Auto takeoff / landing (MODE_AUTO_TKO_LAND) - state machine
// ------------------------------------------------------------
// Vertical uses the AltitudePID inner velocity loop (update_climb_rate) for a
// constant climb/descent rate; horizontal holds the takeoff-point (NED origin
// 0,0 captured at entry) via posN/posE; yaw holds the takeoff heading. The
// takeoff/land triggers are latched request flags set by Telem (GCS commands)
// and consumed here. Touchdown (LiDAR + vertical speed, time-persistent) stops
// the motors.
// ============================================================

extern volatile bool atl_takeoff_request;   // sketch.ino (set by Telem)
extern volatile bool atl_land_request;       // sketch.ino (set by Telem)
extern volatile bool atl_mission_request;    // sketch.ino (set by Telem, MISSION only)

// Vertical takeoff/landing sequence phases. Shared by ATL (auto takeoff/land)
// and MISSION (auto takeoff + PNG guidance + auto land). ATL never enters
// ATL_GUIDANCE; MISSION inserts it between HOLD and LAND.
enum AtlState : uint8_t {
    ATL_GROUND,      // motors idle, waiting for CMD_AUTO_TAKEOFF
    ATL_SPOOLUP,     // hold ATL_SPOOLUP_FRAC*mg for ATL_SPOOLUP_MS, then climb
    ATL_TAKEOFF,     // climb at +TKO_CLIMB_RATE until TKO_TARGET_ALT
    ATL_HOLD,        // altitude-hold at the captured target, waiting for land/mission cmd
    ATL_GUIDANCE,    // (MISSION only) alt-hold at target; caller runs PNG guidance
    ATL_LAND,        // descend at -LAND_SPEED_HIGH until LAND_SLOW_ALT
    ATL_LAND_SLOW,   // descend at -LAND_SPEED_SLOW, run touchdown detection
    ATL_SPOOLDOWN,   // after touchdown hold ATL_SPOOLDOWN_FRAC*mg for ATL_SPOOLDOWN_MS
    ATL_DISARMED     // motors stopped (re-takeoff via command)
};

// Vertical sequencer state. One instance per mode (ATL and MISSION each own one)
// so the two modes never share runtime state.
struct SeqState {
    AtlState state;
    float    hold_h;            // alt-hold setpoint captured at TAKEOFF->HOLD [m, up+]
    uint32_t td_start_ms;       // touchdown-condition start ms (0 = not counting)
    uint32_t phase_start_ms;    // spool-up/down phase start ms
    float    climb_cmd_log;     // last commanded climb rate [m/s, up+] (telemetry)
    uint16_t phase_ms_log;      // ms remaining in spool phase (telemetry)
};

// What the caller must do this step (sequencer owns vertical; caller owns horizontal).
struct SeqOut {
    float U1;            // vertical thrust command [N] (valid when motors_run)
    bool  motors_run;    // false -> stop motors this step
    bool  horiz_hold;    // true -> caller holds a fixed point (origin or last WP)
    bool  guidance_run;  // true -> caller runs PNG guidance (ATL_GUIDANCE only)
};

static SeqState atl_seq;   // ATL mode sequencer state

static void seq_reset(SeqState &st)
{
    st.state          = ATL_GROUND;
    st.hold_h         = 0.0f;
    st.td_start_ms    = 0;
    st.phase_start_ms = 0;
    st.climb_cmd_log  = 0.0f;
    st.phase_ms_log   = 0;
}

static void atl_reset_state()
{
    seq_reset(atl_seq);
    atl_takeoff_request = false;
    atl_land_request    = false;
    atl_mission_request = false;
}

// Telem gate: takeoff allowed only on the ground (and the loop separately
// requires fix3D + IMU + low LiDAR + AUTO at the actual transition).
bool atl_can_takeoff()
{
    return (atl_seq.state == ATL_GROUND);
}

// Telem gate: landing allowed only while holding altitude.
bool atl_in_hold()
{
    return (atl_seq.state == ATL_HOLD);
}

// ============================================================
// Control Source (EMERGENCY > MANUAL > AUTO)
// ============================================================

ControlSource get_control_source()
{
    if (rc_pulse_us[RC_CH_EMERGENCY] >= RC_EMERGENCY_THRESHOLD)
        return CTRL_EMERGENCY;
    if (rc_pulse_us[RC_CH_MODE] >= RC_MODE_THRESHOLD)
        return CTRL_AUTO;
    return CTRL_MANUAL;
}

void set_motor_output(uint16_t us0, uint16_t us1, uint16_t us2, uint16_t us3)
{
    if (!motors_active) return;

    // EMERGENCY override: force all motors to minimum regardless of input
    if (get_control_source() == CTRL_EMERGENCY) {
        motors.disarm();
        return;
    }

    motors.writeMicroseconds(0, us0);
    motors.writeMicroseconds(1, us1);
    motors.writeMicroseconds(2, us2);
    motors.writeMicroseconds(3, us3);
}

// ============================================================
// Mode Names
// ============================================================

const char* mode_name(FlightMode m)
{
    switch (m) {
    case MODE_IDLE:       return "IDLE";
    case MODE_RC_CONTROL: return "RC_CONTROL";
    case MODE_RTK_MAIN:   return "RTK_MAIN";
    case MODE_PWM_TEST:   return "PWM_TEST";
    case MODE_MTI_TEST:   return "MTI_TEST";
    case MODE_ESC_CAL:    return "ESC_CAL";
    case MODE_VEL_CONTROL: return "VEL_CONTROL";
    case MODE_PNG_GUIDANCE: return "PNG_GUIDANCE";
    case MODE_POS_CONTROL: return "POS_CONTROL";
    case MODE_AUTO_TKO_LAND: return "AUTO_TKO_LAND";
    case MODE_MISSION:    return "MISSION";
    default:              return "UNKNOWN";
    }
}

// ============================================================
// IMU Setup (called once from setup())
// ============================================================

void imu_setup()
{
    IMU_SERIAL.begin(IMU_BAUD);
    if (!ins.begin(IMU_SERIAL, IMU_BAUD)) {
        #if USE_DEBUG_SERIAL
        Serial.println("[ERR] IMU init failed in setup");
        #endif
        IMU_SERIAL.end();
        return;
    }
    ins_active = true;

    // Attitude calibration: read IMU for 1s and average Roll/Pitch/Yaw as offset
    {
        // Roll/pitch cal offsets only (level trim). Yaw is used RAW (absolute
        // NED heading) everywhere, so no yaw offset is captured.
        float r_sum = 0, p_sum = 0;
        int samples = 0;
        uint32_t cal_start = millis();
        while (millis() - cal_start < 1000) {
            ins.update();
            if (ins.available()) {
                MtiData d = ins.getData();
                r_sum += (float)d.data[ROLL];
                p_sum += (float)d.data[PITCH];
                samples++;
            }
            delay(5);
        }
        if (samples > 0) {
            roll_offset  = r_sum / samples;
            pitch_offset = p_sum / samples;
        } else {
            roll_offset  = 0.0f;
            pitch_offset = 0.0f;
        }
        #if USE_DEBUG_SERIAL
        Serial.print("[CAL] R="); Serial.print(roll_offset, 2);
        Serial.print(" P="); Serial.println(pitch_offset, 2);
        #endif
    }

    // Altitude estimator: P0 baro calibration (3s blocking)
    filter_state_init(&alt_filter);
    filter_gains_init(&alt_gains, ALT_CF_TAU);
    baro_cal_init(&baro_cal, ALT_BARO_CAL_SEC);
    alt_baro_latest = 0.0f;
    alt_ready = false;
    {
        uint32_t p0_start = millis();
        while (millis() - p0_start < 5000) {
            ins.update();
            if (ins.available()) {
                MtiData dd = ins.getData();
                if (dd.data_available & HAS_PRESSURE) {
                    float temp_c = (dd.data_available & HAS_TEMPERATURE)
                                   ? (float)dd.data[TEMPERATURE] : 20.0f;
                    baro_cal_update(&baro_cal, (float)dd.data[PRESSURE], temp_c, 0.02f);
                    if (baro_cal.ready) {
                        alt_baro_latest = baro_to_altitude(&baro_cal, (float)dd.data[PRESSURE]);
                        alt_filter.h = alt_baro_latest;
                        alt_ready = true;
                        break;
                    }
                }
            }
            delay(5);
        }
    }

    // LiDAR ground-bias zeroing: with the craft on the ground the downward
    // LiDAR reads the sensor-to-ground offset (body mount height). Average a
    // few samples and store it as the zero point so altitude starts at ~0 m.
    #if ALT_AIDING_SOURCE == ALT_AID_LIDAR
    {
        bool ok = lidar.calibrateGround();
        // After zeroing, the LiDAR reports ~0 m on the ground, so seed the
        // complementary-filter altitude at 0 to match (baro seeded it above).
        if (ok) alt_filter.h = 0.0f;
        #if USE_DEBUG_SERIAL
        Serial.print("[LIDAR] ground offset = ");
        Serial.print(lidar.groundOffset(), 3);
        Serial.println(ok ? " m" : " m (FAILED, using 0)");
        #endif
    }
    #endif
}

// ============================================================
// Exit / Enter Mode
// ============================================================

void exit_current_mode()
{
    switch (current_mode) {
    case MODE_RC_CONTROL:
    case MODE_VEL_CONTROL:   // same teardown as RC_CONTROL (shared 200Hz loop)
    case MODE_PNG_GUIDANCE:  // same teardown; also clears PNG guidance state
    case MODE_POS_CONTROL:   // same teardown; also clears position-control state
    case MODE_AUTO_TKO_LAND: // same teardown; also clears auto-tko/land state
    case MODE_MISSION:       // same teardown; auto-tko + PNG guidance + land
        if (motors_active) { motors.disarm(); motors_active = false; }
        // IMU stays active (initialized in setup, shared across modes)
		autopilot.reset();
        // Clear altitude-hold / estimator state so a re-arm starts clean and
        // does not resume from a stale setpoint or frozen FMF state.
        alt_hold_active = false;
        alt_hold_h      = 0.0f;
        filter_state_init(&alt_filter);
        #if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
        fmf_ready = false;
        h_fmf = 0.0f; v_fmf = 0.0f; a_fmf = 0.0f;
        #endif
        // PNG guidance state cleared so a re-entry recaptures the origin.
        png_origin_set = false;
        png_wp_idx     = 0;
        png_wp_prev_idx = -1;
        png_is_end     = false;
        #if USE_OBS_AVOID
        cc.reset(); g_ccd = CCDebug{};   // clear collision-cone latch on teardown
        #endif
        // Position-control state cleared (re-entry recaptures origin).
        pos_wp_idx     = 0;
        pos_is_end     = false;
        // Auto takeoff/landing state cleared (re-entry starts on the ground).
        atl_reset_state();
        // Clear the emergency-landing latch on mode teardown so it never carries
        // into the next flight.
        emland_active  = false;
        break;
    case MODE_RTK_MAIN:
        break;
    case MODE_PWM_TEST:
        if (motors_active) { motors.disarm(); motors_active = false; }
        break;
    case MODE_MTI_TEST:
        break;
    case MODE_ESC_CAL:
        if (motors_active) { motors.disarm(); motors_active = false; }
        break;
    default:
        break;
    }
    current_mode = MODE_IDLE;
}

void enter_mode(FlightMode mode)
{
    if (mode == current_mode) return;
    exit_current_mode();

    #if USE_DEBUG_SERIAL
    Serial.print("[MODE] Entering "); Serial.println(mode_name(mode));
    #endif

    bool ok = true;

    switch (mode) {
    case MODE_RC_CONTROL:
    case MODE_VEL_CONTROL:   // same init as RC_CONTROL (shared 200Hz loop)
        if (!ins_active) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] IMU not initialized");
            #endif
            ok = false; break;
        }
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        init_timer();
        last_imu_data_ms = millis();  // reset IMU watchdog
        break;

    case MODE_PNG_GUIDANCE:
        // Same hardware init as RC_CONTROL/VEL_CONTROL, plus capture the local
        // NED origin from the current GNSS LLA. Requires a valid GNSS fix.
        if (!ins_active) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] IMU not initialized");
            #endif
            ok = false; break;
        }
        // Require an uploaded mission (GCS CMD_SET_WAYPOINTS) before arming PNG.
        // No waypoints -> refuse entry (stay IDLE) so we never fly an empty mission.
        if (png_wp_size <= 0) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PNG: no waypoints uploaded");
            #endif
            ok = false; break;
        }
        {
            // Capture the local-NED origin from a fresh GNSS PVT fix. Position is
            // taken from the GNSS PVT block (0x7010, 4Hz) which carries lat/lon/
            // height directly; data[LATITUDE] (XDI_LATLON) is NOT in the output
            // config, so we must use gnss_pvt here. PVT is 4Hz -> allow ~2s.
            //   lat/lon: int32 1e-7 deg, height: int32 mm (ellipsoid), fix>=3 = 3D.
            png_origin_set = false;
            uint32_t cap_start = millis();
            while (millis() - cap_start < 2000) {
                ins.update();
                if (ins.available()) {
                    MtiData dd = ins.getData();
                    if ((dd.data_available & HAS_GNSS_PVT) &&
                        (dd.gnss_pvt.fix_type >= 3)) {
                        png_capture_origin(dd.gnss_pvt.lat / 1e7,
                                           dd.gnss_pvt.lon / 1e7,
                                           dd.gnss_pvt.height / 1000.0);
                        break;
                    }
                }
                delay(5);
            }
            if (!png_origin_set) {
                #if USE_DEBUG_SERIAL
                Serial.println("[ERR] PNG: no GNSS 3D fix, cannot set origin");
                #endif
                ok = false; break;
            }
        }
        // Convert the uploaded lat/lon mission to local NED (origin-referenced),
        // once, now that the origin is fixed. The 200Hz guidance loop reads the
        // png_wp_north/east cache only. Height is not used by PNG (horizontal
        // guidance), so pass 0 for the waypoint height.
        for (int i = 0; i < png_wp_size; i++) {
            png_lla_to_ned(png_wp_lat[i] / 1e7, png_wp_lon[i] / 1e7, 0.0,
                           png_wp_north[i], png_wp_east[i]);
        }
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        init_timer();
        png_wp_idx       = 0;
        png_wp_prev_idx  = -1;
        png_is_end       = false;
        png_orbit_active = false;     // clear any latched orbit state
        png_orbit_turn   = 0.0f;
        emland_active    = false;     // start each PNG flight with the latch clear
        last_imu_data_ms = millis();  // reset IMU watchdog
        #if USE_DEBUG_SERIAL
        Serial.print("[PNG] Origin set, "); Serial.print(png_wp_size);
        Serial.println(" waypoints loaded");
        #endif
        break;

    case MODE_POS_CONTROL:
        // Same hardware init + GNSS origin capture as PNG, plus initialize the
        // moving-setpoint segment state. Requires a valid GNSS 3D fix and an
        // uploaded mission (shared png_set_waypoints / png_wp_* cache).
        if (!ins_active) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] IMU not initialized");
            #endif
            ok = false; break;
        }
        if (png_wp_size <= 0) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] POS: no waypoints uploaded");
            #endif
            ok = false; break;
        }
        {
            // Capture the local-NED origin from a fresh GNSS PVT fix (4Hz -> 2s).
            png_origin_set = false;
            uint32_t cap_start = millis();
            while (millis() - cap_start < 2000) {
                ins.update();
                if (ins.available()) {
                    MtiData dd = ins.getData();
                    if ((dd.data_available & HAS_GNSS_PVT) &&
                        (dd.gnss_pvt.fix_type >= 3)) {
                        png_capture_origin(dd.gnss_pvt.lat / 1e7,
                                           dd.gnss_pvt.lon / 1e7,
                                           dd.gnss_pvt.height / 1000.0);
                        break;
                    }
                }
                delay(5);
            }
            if (!png_origin_set) {
                #if USE_DEBUG_SERIAL
                Serial.println("[ERR] POS: no GNSS 3D fix, cannot set origin");
                #endif
                ok = false; break;
            }
        }
        // Convert the uploaded lat/lon mission to local NED (origin-referenced).
        for (int i = 0; i < png_wp_size; i++) {
            png_lla_to_ned(png_wp_lat[i] / 1e7, png_wp_lon[i] / 1e7, 0.0,
                           png_wp_north[i], png_wp_east[i]);
        }
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        init_timer();
        // Target the first waypoint directly (step setpoint, no interpolation).
        pos_wp_idx      = 0;
        pos_is_end      = false;
        emland_active   = false;
        last_imu_data_ms = millis();  // reset IMU watchdog
        #if USE_DEBUG_SERIAL
        Serial.print("[POS] Origin set, "); Serial.print(png_wp_size);
        Serial.println(" waypoints loaded");
        #endif
        break;

    case MODE_AUTO_TKO_LAND:
        // Auto takeoff/landing: capture the takeoff-point GNSS origin (no mission
        // needed; horizontal hold targets NED origin 0,0). Same hardware init and
        // GNSS-fix wait as POS. The state machine starts on the ground and waits
        // for CMD_AUTO_TAKEOFF.
        if (!ins_active) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] IMU not initialized");
            #endif
            ok = false; break;
        }
        {
            // Capture the local-NED origin from a fresh GNSS PVT fix (4Hz -> 2s).
            png_origin_set = false;
            uint32_t cap_start = millis();
            while (millis() - cap_start < 2000) {
                ins.update();
                if (ins.available()) {
                    MtiData dd = ins.getData();
                    if ((dd.data_available & HAS_GNSS_PVT) &&
                        (dd.gnss_pvt.fix_type >= 3)) {
                        png_capture_origin(dd.gnss_pvt.lat / 1e7,
                                           dd.gnss_pvt.lon / 1e7,
                                           dd.gnss_pvt.height / 1000.0);
                        break;
                    }
                }
                delay(5);
            }
            if (!png_origin_set) {
                #if USE_DEBUG_SERIAL
                Serial.println("[ERR] ATL: no GNSS 3D fix, cannot set origin");
                #endif
                ok = false; break;
            }
        }
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        init_timer();
        atl_reset_state();            // start on the ground, requests cleared
        emland_active    = false;
        last_imu_data_ms = millis();  // reset IMU watchdog
        #if USE_DEBUG_SERIAL
        Serial.println("[ATL] Origin set, ready for auto takeoff");
        #endif
        break;

    case MODE_MISSION:
        // Auto takeoff + PNG guidance + auto land. Requires an uploaded mission
        // (PNG waypoints) AND a GNSS 3D fix (origin capture), like PNG. The
        // sequencer starts on the ground and waits for CMD_AUTO_TAKEOFF.
        if (!ins_active) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] IMU not initialized");
            #endif
            ok = false; break;
        }
        if (png_wp_size <= 0) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] MISSION: no waypoints uploaded");
            #endif
            ok = false; break;
        }
        {
            // Capture the local-NED origin from a fresh GNSS PVT fix (4Hz -> 2s).
            png_origin_set = false;
            uint32_t cap_start = millis();
            while (millis() - cap_start < 2000) {
                ins.update();
                if (ins.available()) {
                    MtiData dd = ins.getData();
                    if ((dd.data_available & HAS_GNSS_PVT) &&
                        (dd.gnss_pvt.fix_type >= 3)) {
                        png_capture_origin(dd.gnss_pvt.lat / 1e7,
                                           dd.gnss_pvt.lon / 1e7,
                                           dd.gnss_pvt.height / 1000.0);
                        break;
                    }
                }
                delay(5);
            }
            if (!png_origin_set) {
                #if USE_DEBUG_SERIAL
                Serial.println("[ERR] MISSION: no GNSS 3D fix, cannot set origin");
                #endif
                ok = false; break;
            }
        }
        // Convert the uploaded lat/lon mission to local NED (origin-referenced).
        for (int i = 0; i < png_wp_size; i++) {
            png_lla_to_ned(png_wp_lat[i] / 1e7, png_wp_lon[i] / 1e7, 0.0,
                           png_wp_north[i], png_wp_east[i]);
        }
        // Precompute the horizontal segment length into each WP for the altitude
        // ramp. WP0's segment starts at the takeoff origin (0,0); later WPs start
        // at the previous WP. Floored to PNG_RNG_MIN so the ramp never divides by
        // zero on coincident waypoints (ramp then snaps straight to the WP alt).
        for (int i = 0; i < png_wp_size; i++) {
            float pn = (i == 0) ? 0.0f : png_wp_north[i - 1];
            float pe = (i == 0) ? 0.0f : png_wp_east [i - 1];
            float seg = hypotf(png_wp_north[i] - pn, png_wp_east[i] - pe);
            png_wp_seg_len[i] = (seg < PNG_RNG_MIN) ? PNG_RNG_MIN : seg;
        }
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        init_timer();
        png_wp_idx       = 0;
        png_wp_prev_idx  = -1;
        png_is_end       = false;
        png_alt_prev     = 0.0f;       // ramp start filled at GUIDANCE entry
        atl_reset_state();            // sequencer on the ground, requests cleared
        emland_active    = false;
        last_imu_data_ms = millis();  // reset IMU watchdog
        #if USE_DEBUG_SERIAL
        Serial.print("[MISSION] Origin set, "); Serial.print(png_wp_size);
        Serial.println(" waypoints loaded, ready for auto takeoff");
        #endif
        break;

    case MODE_RTK_MAIN:
        init_timer();
        break;

    case MODE_PWM_TEST:
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        for (int i = 0; i < 4; i++) pwm_motor_us[i] = motors.getMinPulse();
        pwm_selected_motor = 0;
        #if USE_DEBUG_SERIAL
        Serial.println("[PWM_TEST] Commands: m0~m3, +, -, us<N>, d, s");
        #endif
        break;

    case MODE_MTI_TEST:
        init_timer();
        break;

    case MODE_ESC_CAL:
        if (!motors.begin(PWM_STANDARD)) {
            #if USE_DEBUG_SERIAL
            Serial.println("[ERR] PWM init failed");
            #endif
            ok = false; break;
        }
        motors_active = true;
        motors.disarm();
        rc_init();
        rc_auto_inhibit = true;  // prevent auto RC_CONTROL re-entry during ESC cal
        #if USE_DEBUG_SERIAL
        Serial.println("[ESC_CAL] Throttle >=1500 -> 2000us, else -> 1000us");
        #endif
        break;

    default:
        break;
    }

    if (ok) {
        current_mode = mode;
        telem_send_ack(P8Serial, CMD_SET_MODE, ACK_OK);
    } else {
        current_mode = MODE_IDLE;
        // Send extended ACK with diagnostic: [flush_count_H, flush_count_L, fail_step]
        uint8_t diag[3] = {
            (uint8_t)(ins._flush_count >> 8),
            (uint8_t)(ins._flush_count & 0xFF),
            (uint8_t)ins._fail_step
        };
        telem_send_ack_ext(P8Serial, CMD_SET_MODE, ACK_ERR_HW_FAIL, diag, 3);
        #if USE_DEBUG_SERIAL
        Serial.println("[MODE] Fallback to IDLE");
        #endif
    }
}

// ============================================================
// Mode Loops
// ============================================================

void loop_idle()
{
    // Nothing to do - wait for commands
}

static uint16_t throttle_to_us(float t)
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return 1000 + (uint16_t)(t * 1000.0f);
}

// Thrust(N) -> PWM(us) using measured motor model
// T = Ct * omega^2,  omega = 10.4713*duty + 107.2435,  PWM = 1000 + duty*10
static uint16_t thrust_to_pwm(float thrust_N)
{
    if (thrust_N <= 0.0f) return 1000;
    float omega = sqrtf(thrust_N / CT);
    float duty = (omega - OMEGA_OFFSET) / OMEGA_SLOPE;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    uint16_t pwm = 1000 + (uint16_t)(duty * 10.0f);
    if (pwm > 2000) pwm = 2000;
    return pwm;
}

// ============================================================
// Shared control-loop helpers
// ------------------------------------------------------------
// The three 200Hz loops (RC_CONTROL, VEL_CONTROL, PNG_GUIDANCE) run an almost
// identical sequence of stages; only the roll/pitch/yaw setpoint source differs.
// The shared stages are factored into the helpers below. ControlContext carries
// the per-iteration values between stages (replacing the loose locals the
// monolithic loops used). Operation order inside each helper is unchanged from
// the original inline code, so the flight behavior is preserved.
//
// NOTE on `static` locals moved into helpers: dividers/timers/accumulators
// (bridge_div, lidar_div, batt_ms, LED timers, rt_count/rt_dt_max,
// last_debug_ms) were per-loop in the original and are now shared across the
// three modes. They only affect sub-tick phase / one telemetry window right at
// a mode switch, never the steady-state control output.
// ============================================================

struct ControlContext {
    MtiData d;                  // latest IMU/INS sample
    float   dt;                 // 200Hz tick period [s]

    // Attitude (calibrated)
    float current_roll_deg, current_pitch_deg, current_yaw_deg;
    float roll_rad, pitch_rad, yaw_rad;
    float roll_rate, pitch_rate, yaw_rate;

    // Altitude aiding / estimate
    bool  aid_valid;
    float aid_alt_m;
    float lidar_alt_m;          // tilt-comp LiDAR alt (0 if invalid / not LiDAR)
    float h_used, v_used;       // controller feedback (up+)

    // GNSS/RTK velocity rotated to body
    float cpsi_b, spsi_b;
    float gnss_vel_u, gnss_vel_v;   // body forward (u) / right (v) velocity [m/s]

    // Control source + alt-hold edge latch
    ControlSource ctrl;
    bool  alt_hold_capture;
    // PNG forces altitude-hold ON regardless of CH_MODE (AUTO only toggles
    // guidance vs manual attitude; throttle is ALWAYS an altitude command). Set
    // by the caller before ctrl_read_sensors(). false in RC/VEL (CH_MODE-gated).
    bool  force_alt_hold;

    // Outputs of the throttle / attitude / mixing stages (logged in DebugFrame)
    float rc_throttle;
    float U1, U2, U3, U4;
    float F_FR, F_RR, F_RL, F_FL;
    float roll_ff_rad, pitch_ff_rad;
};

// ---- Housekeeping (runs every iteration, before the 200Hz gate) ----
// Bridge RPC pump (~50Hz), LiDAR read (~50Hz), P8 uplink parsing. Identical in
// all three loops.
static inline void ctrl_housekeeping()
{
#if USE_BRIDGE
    {
        static uint8_t bridge_div = 0;
        if (++bridge_div >= 4) { bridge_div = 0; Bridge.update(); }
    }
#endif

    #if ALT_AIDING_SOURCE == ALT_AID_LIDAR
    {
        static uint8_t lidar_div = 0;
        if (++lidar_div >= LIDAR_READ_DIV) {
            lidar_div = 0;
            float r;
            lidar.read(r);
        }
    }
    #endif

    // H7 obstacle slave polling (~50Hz). The tight-loop flight modes (PNG/VEL/
    // POS/ATL) never return to sketch.ino's loop() while running, so the
    // obstacle.poll() in stage 8 there is starved during flight. Poll it here so
    // obstacle.lastOkMs()/count() stay fresh for the collision-cone avoidance.
    #if USE_OBSTACLE_RX
    {
        static uint8_t obs_div = 0;
        if (++obs_div >= OBSTACLE_READ_DIV) {
            obs_div = 0;
            obstacle.poll();
        }
    }
    #endif

    // P8 uplink parsing
    while (P8Serial.available()) {
        if (p8_parser.feedByte(P8Serial.read()))
            telem_handle_cmd(p8_parser.getCmd());
    }
}

// RC signal-loss detection (500ms debounce).
//
// failsafe_land controls what happens once RC has been lost long enough:
//   false (RC_CONTROL): RC is the only control input, so exit to IDLE
//          (exit_current_mode stops the motors) and return true -> caller breaks.
//   true  (VEL/PNG/POS/ATL autonomous modes): RC is the pilot's manual-recovery
//          path, not the primary input. Dropping to IDLE here would cut the
//          motors in mid-air. Instead latch emland_active so the mode's own
//          control loop flies a fixed-thrust gentle descent (level / station-keep
//          attitude) while staying armed, and return false -> caller keeps running.
// The two debounce locals live in the caller so each loop entry starts fresh.
static inline bool ctrl_rc_loss(uint32_t &rc_lost_since_ms, bool &rc_was_valid,
                                bool failsafe_land)
{
    bool rc_valid = (rc_pulse_us[RC_CH_THROTTLE] >= 900 &&
                     rc_pulse_us[RC_CH_THROTTLE] <= 2200);
    if (rc_valid) {
        rc_was_valid = true;
        rc_lost_since_ms = millis();
    } else if (rc_was_valid && (millis() - rc_lost_since_ms) >= 500) {
        if (failsafe_land) {
            // Autonomous mode: don't cut motors in the air. Latch the gentle
            // emergency-landing descent (idempotent if already latched) and keep
            // flying. rc_was_valid stays set so the latch is armed only once;
            // RC EMERGENCY / mode change clears emland_active elsewhere.
            emland_active = true;
            return false;
        }
        rc_was_valid = false;
        exit_current_mode();  // sets current_mode = IDLE -> while exits
        return true;
    }
    return false;
}

// Realtime diagnostics: track ticks-per-50Hz-window and worst-case tick period.
// Accumulators are passed by ref (owned by the caller, attached to DebugFrame).
static inline void ctrl_diag_tick(uint32_t &rt_last_us, uint16_t &rt_count,
                                  float &rt_dt_max)
{
    uint32_t now_us = micros();
    if (rt_last_us != 0) {
        float tick_ms = (float)(now_us - rt_last_us) / 1000.0f;
        if (tick_ms > rt_dt_max) rt_dt_max = tick_ms;
    }
    rt_last_us = now_us;
    rt_count++;
}

// Periodic tasks inside the 200Hz tick: battery sampling (2Hz, 8-sample MA) and
// status LEDs (alive 1Hz, INS activity). Identical in all three loops.
static inline void ctrl_periodic_tasks()
{
    // Battery voltage sampling (2Hz)
    {
        static uint32_t batt_ms = 0;
        if (millis() - batt_ms >= (1000 / BATT_SAMPLE_HZ)) {
            batt_ms = millis();
            static uint16_t b1[8], b2[8];
            static uint8_t bi = 0;
            static bool bf = false;
            b1[bi] = (uint16_t)(analogRead(BATT_MAIN_PIN) * (ADC_VREF_MV / (float)ADC_MAX) * BATT_DIVIDER_RATIO);
            b2[bi] = (uint16_t)(analogRead(BATT_SUB_PIN)  * (ADC_VREF_MV / (float)ADC_MAX) * BATT_DIVIDER_RATIO);
            bi = (bi + 1) % 8;
            if (bi == 0) bf = true;
            uint8_t cnt = bf ? 8 : bi;
            uint32_t s1 = 0, s2 = 0;
            for (uint8_t i = 0; i < cnt; i++) { s1 += b1[i]; s2 += b2[i]; }
            batt_main_mv = (uint16_t)(s1 / cnt);
            batt_sub_mv  = (uint16_t)(s2 / cnt);
        }
    }

    // Alive LED (1Hz blink)
    {
        static uint32_t led_ms = 0;
        if (millis() - led_ms >= 500) {
            led_ms = millis();
            digitalWrite(LEDR, !digitalRead(LEDR));
        }
    }

    // INS sensor LED
    if (ins_active && ins.available()) {
        static uint32_t ins_led_ms = 0;
        if (millis() - ins_led_ms > 100) {
            digitalWrite(LEDB, !digitalRead(LEDB));
            ins_led_ms = millis();
        }
    }
}

// Read this iteration's IMU/INS sample into the context: latched fallback
// altitudes (baro/fusion/vert-vel), calibrated attitude (rad), body rates, the
// control source, and the alt-hold OFF->ON edge. Mutates the file-scope
// alt_baro_latest/alt_fusion_latest/vel_down_latest/alt_hold_active just as the
// original inline code did. Operation order unchanged.
static void ctrl_read_sensors(ControlContext &cx)
{
    cx.ctrl = get_control_source();

    // Alt-hold engage. Normally CH_MODE-gated (AUTO = alt-hold ON). PNG sets
    // force_alt_hold so altitude-hold is ALWAYS active there (CH_MODE only
    // toggles guidance vs manual attitude; throttle is always an alt command).
    // On the OFF->ON edge, latch a deferred setpoint capture (applied after the
    // aiding/FMF update so feedback and setpoint match).
    cx.alt_hold_capture = false;
    bool want_alt_hold = (cx.ctrl == CTRL_AUTO) || cx.force_alt_hold;
    if (want_alt_hold) {
        if (!alt_hold_active) {
            alt_hold_active     = true;
            cx.alt_hold_capture = true;
        }
    } else {
        alt_hold_active = false;
    }

    cx.d = ins.getData();
    MtiData &d = cx.d;

    // Baro altitude (always updated; used as fallback h_aid)
    if (alt_ready && (d.data_available & HAS_PRESSURE))
        alt_baro_latest = baro_to_altitude(&baro_cal, (float)d.data[PRESSURE]);

    // MTi XKF3 fusion altitude (ellipsoid) @50Hz; ZOH at 200Hz
    if (d.data_available & HAS_ALT_ELLIPSOID)
        alt_fusion_latest = (float)d.data[ALT_ELLIPSOID];

    // Vertical velocity w (NED down+) from MTi GNSS/INS @50Hz; ZOH at 200Hz
    if (d.data_available & HAS_VELOCITY_XYZ)
        vel_down_latest = (float)d.data[VEL_Z];

    // Sensor angles (deg, calibrated) -> rad. Yaw uses RAW MTi heading (NED,
    // no offset) for a true absolute heading; roll/pitch keep cal offsets.
    cx.current_roll_deg  = (float)d.data[ROLL]  - roll_offset;
    cx.current_pitch_deg = (float)d.data[PITCH] - pitch_offset;
    cx.current_yaw_deg   = (float)d.data[YAW];

    cx.roll_rad  = cx.current_roll_deg  * DEG_TO_RAD;
    cx.pitch_rad = cx.current_pitch_deg * DEG_TO_RAD;
    cx.yaw_rad   = cx.current_yaw_deg   * DEG_TO_RAD;
    cx.yaw_rad   = wrap_pi(cx.yaw_rad);

    cx.roll_rate  = (float)d.data[GYR_X];
    cx.pitch_rate = (float)d.data[GYR_Y];
    cx.yaw_rate   = (float)d.data[GYR_Z];
}

// Altitude estimation stage: pick the aiding source (compile-time), step the
// complementary filter (and FMF on LiDAR), and select the controller feedback
// h_used/v_used. Populates cx.aid_valid/aid_alt_m/lidar_alt_m/h_used/v_used.
// Operation order and the #if branches are unchanged from the inline original.
static void ctrl_update_altitude_est(ControlContext &cx)
{
    MtiData &d = cx.d;

    // ---- Altitude aiding source (compile-time, hw_config.h) ----
    cx.aid_valid   = false;
    cx.aid_alt_m   = 0.0f;
    cx.lidar_alt_m = 0.0f;  // kept for debug frame (tilt-compensated)

#if ALT_AIDING_SOURCE == ALT_AID_BARO
    // Barometer (MTi pressure -> ISA altitude). Valid once P0 calibrated.
    if (alt_ready) {
        cx.aid_alt_m = alt_baro_latest;   // updated in ctrl_read_sensors
        cx.aid_valid = true;
    }
#elif ALT_AIDING_SOURCE == ALT_AID_LIDAR
    // TF-Nova LiDAR (downward). Driver gates invalid samples and refreshes its
    // timestamp only on valid ones; lastRangeM() is ground-bias zeroed.
    {
        // Spike-rejection state on the raw range (held across ticks). Seeded on
        // the first valid sample and re-seeded after a dropout so the estimate
        // can never latch onto a stale value (see hw_config.h LIDAR_SPIKE_*).
        static float    lidar_prev_r    = 0.0f;
        static bool     lidar_seeded    = false;
        static uint8_t  lidar_spike_cnt = 0;

        float r     = lidar.lastRangeM();
        uint32_t ts = lidar.lastMs();
        if (ts != 0 && (millis() - ts) <= LIDAR_ALT_TIMEOUT_MS) {
            // Reject sudden jumps on the RAW range (before tilt-comp/CF/FMF).
            if (!lidar_seeded) {
                lidar_prev_r    = r;       // first valid sample / dropout return
                lidar_seeded    = true;
                lidar_spike_cnt = 0;
            } else if (fabsf(r - lidar_prev_r) > LIDAR_SPIKE_M) {
                if (++lidar_spike_cnt <= LIDAR_SPIKE_MAX) {
                    r = lidar_prev_r;      // hold previous (limited # of ticks)
                } else {
                    lidar_spike_cnt = 0;   // sustained -> accept genuine change
                    lidar_prev_r    = r;
                }
            } else {
                lidar_spike_cnt = 0;       // back in range
                lidar_prev_r    = r;
            }

            // Tilt-comp: alt = range * cos(roll) * cos(pitch).
            cx.lidar_alt_m = r * cosf(cx.roll_rad) * cosf(cx.pitch_rad);
            cx.aid_alt_m   = cx.lidar_alt_m;
            cx.aid_valid   = true;
        } else {
            // Stale/no sample: arm a re-seed so the next valid range is taken
            // as-is instead of being rejected against an outdated prev value.
            lidar_seeded = false;
        }
    }
#else
    #error "ALT_AIDING_SOURCE must be ALT_AID_BARO or ALT_AID_LIDAR"
#endif

    // CF steps only while the aiding source is valid (else alt-hold released).
    if (alt_ready && cx.aid_valid && (d.data_available & HAS_FREE_ACCELERATION))
        step_free_acc(&alt_filter, &alt_gains, (float)d.data[FREE_ACC_Z], cx.aid_alt_m, cx.dt);

#if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
    // FMF on the tilt-comp'd LiDAR altitude. Seeded on first valid sample.
    if (cx.aid_valid) {
        if (!fmf_ready) {
            alt_fmf.init((float)SAMPLING_TIME, FMF_MODE, FMF_BETA, cx.lidar_alt_m);
            fmf_ready = true;
        } else {
            alt_fmf.measUpdate(cx.lidar_alt_m, true);  // up+ in, up+ out
        }
        h_fmf = alt_fmf.height();
        v_fmf = alt_fmf.velocity();
        a_fmf = alt_fmf.accel();
    } else {
        // LiDAR dropped out: arm a re-seed and zero the outputs.
        fmf_ready = false;
        h_fmf = 0.0f;
        v_fmf = 0.0f;
        a_fmf = 0.0f;
    }
#endif

    // Altitude feedback used by the controller (up+), same source as alt-hold.
    cx.h_used = 0.0f;
    cx.v_used = 0.0f;
    if (alt_ready) {
    #if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
        cx.h_used = h_fmf;
        cx.v_used = v_fmf;
    #elif (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && ALT_LIDAR_DIRECT
        cx.h_used = cx.lidar_alt_m;
        cx.v_used = alt_filter.v;
    #else
        cx.h_used = alt_filter.h;
        cx.v_used = alt_filter.v;
    #endif
    }

    // Deferred alt-hold setpoint capture (edge latched in ctrl_read_sensors).
    if (cx.alt_hold_capture) {
        alt_hold_h = alt_ready ? alt_hold_active_h(cx.lidar_alt_m) : 0.0f;
        autopilot.reset_altitude();
    }
}

// GNSS/RTK velocity (NED N/E) rotated to body by yaw. Fills cpsi_b/spsi_b and
// body forward/right velocity. Computed in all loops so (u,v) is always logged.
static inline void ctrl_rotate_gnss_vel(ControlContext &cx)
{
    cx.cpsi_b = cosf(cx.yaw_rad);
    cx.spsi_b = sinf(cx.yaw_rad);
    float velN_b = (float)cx.d.data[VEL_X], velE_b = (float)cx.d.data[VEL_Y];
    cx.gnss_vel_u =  velN_b * cx.cpsi_b + velE_b * cx.spsi_b;   // body u (forward)
    cx.gnss_vel_v = -velN_b * cx.spsi_b + velE_b * cx.cpsi_b;   // body v (right)
}

// RC throttle -> total thrust U1, with alt-hold override (CTRL_AUTO).
// Fills cx.rc_throttle and cx.U1. Operation order unchanged from the inline
// original (shared verbatim by all three loops).
static void ctrl_compute_thrust(ControlContext &cx)
{
    // RC throttle -> total thrust U1 (N)
    float rc_throttle = (rc_pulse_us[RC_CH_THROTTLE] - 1000) / 1000.0f;
    if (rc_throttle < 0.0f) rc_throttle = 0.0f;
    if (rc_throttle > 1.0f) rc_throttle = 1.0f;
    cx.rc_throttle = rc_throttle;   // still logged, even under emergency landing

    // Emergency landing (latched): force a fixed gentle-descent thrust and skip
    // RC throttle / alt-hold entirely. rc_throttle was computed above only for
    // logging; the cutoff in ctrl_apply_attitude is bypassed when emland_active.
    if (emland_active) {
        cx.U1 = EMLAND_THRUST_FRAC * (MASS_KG * 9.81f);
        return;
    }

    float U1 = rc_throttle * (MASS_KG * 9.81f * 2.0f);

    // AUTO throttle: below SPLIT = gentle thrust ramp (0 -> SPLIT_THRUST_FRAC*mg)
    // so takeoff and landing behave the same with no direct-thrust jump at the
    // boundary; above SPLIT = linear throttle->target alt via the alt controller.
    // Invalid aiding releases alt-hold (U1 stays RC throttle). up+ -> NED in AP.
    const float mg = MASS_KG * 9.81f;

    if (alt_hold_active && alt_ready && cx.aid_valid) {
        if (rc_throttle <= ALT_CMD_THR_SPLIT) {
            // Ramp region (takeoff/landing): U1 maps linearly from 0 (stick down)
            // to SPLIT_THRUST_FRAC*mg at SPLIT. 0.8*mg < hover -> the craft eases
            // down to the ground instead of dropping when the stick comes below
            // SPLIT. Keep the alt integrator primed for a bump-free hand-off into
            // the alt-command region; alt_hold_h tracks the current altitude.
            autopilot.reset_altitude();
            alt_hold_h = alt_hold_active_h(cx.lidar_alt_m);
            U1 = (rc_throttle / ALT_CMD_THR_SPLIT) *
                 (ALT_CMD_SPLIT_THRUST_FRAC * mg);
        } else {
            // Alt-command region: linear throttle -> target altitude.
            alt_hold_h = (rc_throttle - ALT_CMD_THR_SPLIT) /
                         (1.0f - ALT_CMD_THR_SPLIT) * ALT_CMD_MAX_M;
            U1 = autopilot.update_altitude(cx.h_used, cx.v_used,
                                           (float)cx.d.data[FREE_ACC_Z],
                                           alt_hold_h, mg, cx.dt);
        }
    }
    cx.U1 = U1;
}

// Takeoff feed-forward + attitude control (torques U2/U3/U4) + mixing -> motor
// output. Below the low-throttle cutoff, motors are stopped and the autopilot
// reset (also re-arming yaw/takeoff-FF via the caller's flags). Identical in all
// three loops. takeoff_ff_done/yaw_initialized owned by the caller (per-loop).
static void ctrl_apply_attitude(ControlContext &cx,
                                float target_roll_rad, float target_pitch_rad,
                                float target_yaw_rate,
                                bool &yaw_initialized, bool &takeoff_ff_done)
{
    cx.U2 = 0.0f; cx.U3 = 0.0f; cx.U4 = 0.0f;
    cx.F_FR = 0.0f; cx.F_RR = 0.0f; cx.F_RL = 0.0f; cx.F_FL = 0.0f;

    // Takeoff FF applied this iteration [rad] (logged). 0 when motors stopped.
    cx.roll_ff_rad  = 0.0f;
    cx.pitch_ff_rad = 0.0f;

    // Low thrust safety. Bypassed during emergency landing: emland forces U1
    // directly (independent of RC throttle), so a low stick must NOT cut motors
    // and drop the craft. The descent thrust keeps it controllable.
    if (cx.rc_throttle < 0.05f && !emland_active) {
        set_motor_output(1000, 1000, 1000, 1000);
        autopilot.reset();
        yaw_initialized = false;
        takeoff_ff_done = false;   // re-arm takeoff FF for next flight
        return;
    }

    // Takeoff tilt FF: fade=1 below H_LO, 0 above H_HI (linear between). Latched
    // off once past H_HI, re-armed whenever the craft drops back below H_LO
    // (hysteresis). alt not ready -> treat as on-ground (fade=1).
    float ff_fade = 0.0f;
    if (alt_ready && alt_filter.h <= TAKEOFF_FADE_H_LO) {
        takeoff_ff_done = false;  // re-arm for the next climb
    }
    if (!takeoff_ff_done) {
        ff_fade = 1.0f;
        if (alt_ready) {
            float h = alt_filter.h;  // up+, m
            if (h >= TAKEOFF_FADE_H_HI) {
                ff_fade = 0.0f;
                takeoff_ff_done = true;   // latch off until h drops below H_LO
            } else if (h > TAKEOFF_FADE_H_LO) {
                ff_fade = (TAKEOFF_FADE_H_HI - h) /
                          (TAKEOFF_FADE_H_HI - TAKEOFF_FADE_H_LO);
            }
            // h <= H_LO -> ff_fade stays 1.0
        }
    }
    cx.roll_ff_rad  = TAKEOFF_TRIM_ROLL_DEG  * DEG_TO_RAD * ff_fade;
    cx.pitch_ff_rad = TAKEOFF_TRIM_PITCH_DEG * DEG_TO_RAD * ff_fade;

    // Attitude control (Autopilot): setpoints -> torques U2/U3/U4 (Nm)
    autopilot.set_angle(target_roll_rad + cx.roll_ff_rad,
                        target_pitch_rad + cx.pitch_ff_rad);
    autopilot.set_yaw_rate(target_yaw_rate);
    autopilot.update(cx.roll_rad, cx.pitch_rad,
                     cx.roll_rate, cx.pitch_rate, cx.yaw_rate,
                     cx.dt, cx.U2, cx.U3, cx.U4);

    // Mixing matrix -> per-motor thrust (N)
    mix_to_thrust(cx.U1, cx.U2, cx.U3, cx.U4, cx.F_FR, cx.F_RR, cx.F_RL, cx.F_FL);

    set_motor_output(thrust_to_pwm(cx.F_FL), thrust_to_pwm(cx.F_RL),
                     thrust_to_pwm(cx.F_RR), thrust_to_pwm(cx.F_FR));
}

#if USE_OBS_AVOID
// Collision-cone yaw-rate avoidance step, shared by PNG (always on) and MISSION
// (per-segment gated). Picks the nearest fresh H7 obstacle and runs cc.update();
// returns the avoidance yaw-rate while the latch is on, otherwise r_wp unchanged.
// When `enable` is false the latch is cleared and r_wp passes straight through,
// so callers gate avoidance simply by toggling `enable`. g_ccd is always updated
// for the DebugFrame.
static float ctrl_apply_avoidance(const ControlContext &cx, bool enable, float r_wp)
{
    if (!enable) {
        cc.reset();
        g_ccd = CCDebug{};
        return r_wp;
    }
    const MtiData &d = cx.d;
    // Nearest H7 obstacle (min dist_mm), gated on packet freshness.
    bool obs_fresh = (millis() - obstacle.lastOkMs()) < OBSTACLE_TIMEOUT_MS;
    int  obs_best  = -1;
    uint16_t obs_best_mm = 0xFFFF;
    for (uint8_t oi = 0; oi < obstacle.count(); ++oi) {
        uint16_t dmm = obstacle.obstacle(oi).dist_mm;
        if (dmm < obs_best_mm) { obs_best_mm = dmm; obs_best = oi; }
    }
    bool  obs_valid = obs_fresh && (obs_best >= 0);
    float obs_R_o = obs_valid ? (obs_best_mm / 1000.0f) : 0.0f;
    // e_o: H7 body heading-error fed in as-is (per spec; flip sign here if a ground
    // test shows turning toward the obstacle).
    float obs_e_o = obs_valid
                  ? obstacle.obstacle(obs_best).center_deg * DEG_TO_RAD
                  : 0.0f;
    float obs_v_a = hypotf((float)d.data[VEL_X], (float)d.data[VEL_Y]);
    return cc.update(obs_valid, obs_R_o, obs_e_o, obs_v_a, r_wp, g_ccd);
}
#endif

// Fill the DebugFrame fields shared by all three control loops: attitude/rates,
// torque/thrust outputs, altitude estimator + controller telemetry, GNSS/RTK
// velocity, and attitude inner-loop diagnostics. Mode-specific fields (velocity
// controller u_cmd/... and PNG png_*/wp_idx) are filled by the caller AFTER this
// returns. Operation order matches the original inline assignments.
static void ctrl_fill_debug_common(DebugFrame &df, const ControlContext &cx,
                                   float target_roll_rad, float target_pitch_rad,
                                   float target_yaw_rate)
{
    const MtiData &d = cx.d;

    // Zero the whole frame first so fields not written by the active loop
    // (e.g. wp_alt_cmd / vlatch_* outside ATL guidance) read as 0 instead of
    // stale stack garbage. Cheap (~once/20ms) and keeps the GCS log clean.
    memset(&df, 0, sizeof(df));

    // Group 1: attitude controller
    df.roll  = cx.roll_rad;
    df.pitch = cx.pitch_rad;
    df.yaw   = cx.yaw_rad;
    df.p = cx.roll_rate;
    df.q = cx.pitch_rate;
    df.r = cx.yaw_rate;
    df.roll_cmd  = target_roll_rad;
    df.pitch_cmd = target_pitch_rad;
    df.r_cmd     = target_yaw_rate;
    df.roll_rate_cmd  = autopilot.roll_rate_cmd();
    df.pitch_rate_cmd = autopilot.pitch_rate_cmd();
    df.p_f            = autopilot.roll_rate_f();
    df.q_f            = autopilot.pitch_rate_f();
    df.e_roll      = autopilot.roll_angle_error();
    df.e_pitch     = autopilot.pitch_angle_error();
    df.int_e_roll  = autopilot.roll_angle_integral();
    df.int_e_pitch = autopilot.pitch_angle_integral();
    df.roll_ff     = cx.roll_ff_rad;
    df.pitch_ff    = cx.pitch_ff_rad;
    df.e_r   = autopilot.yaw_last_error();
    df.int_e_r = autopilot.yaw_integral();
    df.I_r   = autopilot.yaw_last_I();
    df.U1 = cx.U1; df.U2 = cx.U2; df.U3 = cx.U3; df.U4 = cx.U4;
    // F1..F4 (per-motor thrust) removed from DebugFrame; mixing still uses
    // cx.F_FR/F_RR/F_RL/F_FL internally.

    // Group 2: altitude controller. Barometer/CF debug fields (alt_est,
    // alt_baro, free_acc_z, pressure_pa, w_down) were removed from DebugFrame;
    // the estimator still runs, only its logging is dropped.
    df.throttle_N = cx.U1;
    df.lidar_alt   = cx.lidar_alt_m;            // body-tilt-comp LiDAR alt (0 if invalid)
    df.lidar_valid = cx.aid_valid ? 1 : 0;      // 1 = LiDAR aiding source valid
    df.alt_hold    = alt_hold_active ? 1 : 0;
    df.alt_cmd     = alt_hold_h;
    df.alt_error   = autopilot.alt_last_error();
    df.hdot_cmd    = autopilot.alt_last_hdot();
    df.DeltaT_cmd  = autopilot.alt_last_DeltaT();
    // FMF (LiDAR-only height/velocity); 0 when the FMF path is compiled out.
    #if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_LIDAR_FMF
    df.h_fmf = h_fmf;
    df.v_fmf = v_fmf;
    df.a_fmf = a_fmf;
    #else
    df.h_fmf = 0.0f;
    df.v_fmf = 0.0f;
    df.a_fmf = 0.0f;
    #endif
    df.h_used = cx.h_used;
    df.v_used = cx.v_used;

    // Group 5: misc (timing, battery, GNSS/RTK). Velocity (group 3) and PNG
    // (group 4) fields are mode-specific -> filled by the caller after this.
    df.uptime_ms = millis();
    df.batt_mv   = batt_main_mv;
    df.rtk_status = (uint8_t)d.data[RTK_STATUS];   // 0=none,1=float,2=fixed
    df.gnss_fix   = (uint8_t)d.data[GNSS_FIX];     // 1 = fix valid
    df.vel_n      = (float)d.data[VEL_X];          // NED North [m/s]
    df.vel_e      = (float)d.data[VEL_Y];          // NED East  [m/s]
    df.vel_u      = cx.gnss_vel_u;                 // body forward (u) [m/s]
    df.vel_v      = cx.gnss_vel_v;                 // body right   (v) [m/s]

    // Group 4b: collision-cone avoidance defaults (0 in every loop; PNG
    // overwrites with the live g_ccd values after this).
    df.obs_r_ca      = 0.0f;
    df.obs_Ro        = 0.0f;
    df.obs_eo        = 0.0f;
    df.obs_gamma     = 0.0f;
    df.obs_theta_dot = 0.0f;
    df.obs_avoid     = 0;
}

// Stamp realtime diagnostics + send the DebugFrame at TELEM_FREQ_HZ (50Hz).
// rt_count/rt_dt_max accumulators are reset for the next window. Shared verbatim.
static inline void ctrl_send_debug(DebugFrame &df, uint16_t &rt_count, float &rt_dt_max)
{
    static uint32_t last_debug_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_debug_ms >= (1000 / TELEM_FREQ_HZ)) {
        last_debug_ms = now_ms;
        df.loop_count_50hz = rt_count;
        df.loop_dt_max_ms  = rt_dt_max;
        rt_count  = 0;
        rt_dt_max = 0.0f;
        telem_send_debug(P8Serial, df);
    }
}

// Shared 200Hz control loop for MODE_RC_CONTROL and MODE_VEL_CONTROL.
// Only the roll/pitch setpoint source differs:
//   use_vel_ctrl == false : RC stick -> target angle directly.
//   use_vel_ctrl == true  : RC stick -> velocity cmd -> VelocityController.
// Altitude / throttle / yaw / alt-hold / DebugFrame logic is shared.
// (PNG waypoint guidance has its own dedicated loop: loop_png_guidance().)
static void run_control_loop(bool use_vel_ctrl)
{
    const FlightMode this_mode = use_vel_ctrl ? MODE_VEL_CONTROL : MODE_RC_CONTROL;

    static float target_yaw_rad = 0.0f;
    static bool yaw_initialized = false;

    // Takeoff FF latch: applied only on the first climb. Latches off once past
    // TAKEOFF_FADE_H_HI; cleared on motor stop so the next takeoff re-enables it.
    static bool takeoff_ff_done = false;

    // RC signal loss tracking (local to tight loop)
    uint32_t rc_lost_since_ms = millis();
    bool     rc_was_valid     = true;

    // Realtime diagnostics accumulators (attached to DebugFrame each 50Hz window).
    static uint32_t rt_last_us = 0;
    static uint16_t rt_count   = 0;
    static float    rt_dt_max  = 0.0f;

    // --- 200Hz tight loop (bypass Arduino loop() overhead) ---
    while (current_mode == this_mode) {

        // ---- Housekeeping (every iteration): Bridge/LiDAR/P8 uplink ----
        ctrl_housekeeping();

        // RC signal loss (500ms debounce). RC_CONTROL -> IDLE (RC is the only
        // input); VEL_CONTROL is autonomous -> failsafe emland descent instead
        // of cutting motors in the air.
        if (ctrl_rc_loss(rc_lost_since_ms, rc_was_valid, use_vel_ctrl)) break;

        ins.update();

        if (!time_ready()) continue;  // busy-wait for 200Hz
        update_time();

        float dt = (float)SAMPLING_TIME;

        // ---- Realtime diagnostics: actual 200Hz tick period ----
        ctrl_diag_tick(rt_last_us, rt_count, rt_dt_max);

        // ---- Periodic tasks (inside 200Hz tick): battery + LEDs ----
        ctrl_periodic_tasks();

        // ---- 200Hz control logic ----

        if (ins.available())
            last_imu_data_ms = millis();

        // IMU watchdog: no data for 500ms -> force motor stop
        if ((millis() - last_imu_data_ms) >= IMU_TIMEOUT_MS) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // ---- Read IMU sample + control source + alt-hold edge into context ----
        // RC/VEL: alt-hold is CH_MODE-gated (AUTO only). PNG overrides this.
        ControlContext cx;
        cx.dt = dt;
        cx.force_alt_hold = false;
        ctrl_read_sensors(cx);
        ControlSource ctrl = cx.ctrl;

        if (ctrl == CTRL_EMERGENCY) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // Convenience aliases (only those the setpoint section below still uses;
        // attitude/altitude values are consumed directly from cx by the helpers).
        MtiData &d = cx.d;

        // ---- Altitude estimation (aiding + CF + FMF + h_used/v_used) ----
        ctrl_update_altitude_est(cx);
        float lidar_alt_m = cx.lidar_alt_m;
        bool  aid_valid   = cx.aid_valid;
        float yaw_rad     = cx.yaw_rad;

        // Initialize yaw hold
        if (!yaw_initialized) {
            target_yaw_rad = yaw_rad;
            yaw_initialized = true;
        }

        // Tilt protection (in deg)
        if (cx.current_roll_deg > TILT_LIMIT_DEG || cx.current_roll_deg < -TILT_LIMIT_DEG ||
            cx.current_pitch_deg > TILT_LIMIT_DEG || cx.current_pitch_deg < -TILT_LIMIT_DEG) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            takeoff_ff_done = false;   // re-arm takeoff feed-forward for next flight
            continue;
        }

        // GNSS/RTK velocity (NED N/E) rotated to body by yaw. Computed in both
        // modes so (u,v) is always logged. VEL_X=North, VEL_Y=East -> u=fwd+, v=right+.
        ctrl_rotate_gnss_vel(cx);
        float cpsi_b = cx.cpsi_b, spsi_b = cx.spsi_b;
        float gnss_vel_u = cx.gnss_vel_u;   // body forward (u)
        float gnss_vel_v = cx.gnss_vel_v;   // body right   (v)

        // RC -> target attitude. Two paths share the rest of the loop:
        float target_roll_rad;
        float target_pitch_rad;
        // Velocity-controller logging (u=forward, v=right; 0 in RC_CONTROL).
        float vc_u_cmd = 0.0f, vc_u_fb = 0.0f, vc_u_acc = 0.0f;
        float vc_v_cmd = 0.0f, vc_v_fb = 0.0f, vc_v_acc = 0.0f;

        if (!use_vel_ctrl) {
            // RC_CONTROL: RC stick -> target angles (deg -> rad)
            float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            float target_roll_deg  = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            target_roll_rad  = target_roll_deg  * DEG_TO_RAD;
            target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
        } else {
            // VEL_CONTROL: RC stick -> velocity command [m/s]. Stick fwd -> u>0.
            float u_cmd =  ((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (VEL_MAX_CMD_MS / 500.0f);
            float v_cmd = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (VEL_MAX_CMD_MS / 500.0f);

            // Velocity feedback from GNSS (rotated above); free accel (NED)
            // rotated here by yaw. u=forward, v=right.
            float cpsi = cpsi_b, spsi = spsi_b;
            float accN = (float)d.data[FREE_ACC_X], accE = (float)d.data[FREE_ACC_Y];
            float u_fb  = gnss_vel_u;
            float v_fb  = gnss_vel_v;
            float u_acc =  accN * cpsi + accE * spsi;
            float v_acc = -accN * spsi + accE * cpsi;

            // Controller ALWAYS runs/logs (observable from the ground); only its
            // OUTPUT is gated by altitude (VEL_ENABLE_ALT_M). The integral is
            // reset in the gated (below enable-alt) branch below so it cannot wind
            // up while the output is not applied.
            float vc_pitch_cmd, vc_roll_cmd;
            autopilot.update_velocity(u_cmd, u_fb, u_acc,
                                      v_cmd, v_fb, v_acc,
                                      cx.dt, vc_pitch_cmd, vc_roll_cmd);
            // Always log the controller's signals (visible below 1 m too).
            vc_u_cmd = u_cmd;
            vc_u_fb  = u_fb;
            vc_u_acc = u_acc;
            vc_v_cmd = v_cmd;
            vc_v_fb  = v_fb;
            vc_v_acc = v_acc;

            float vel_alt_m = (alt_ready && aid_valid)
                              ? alt_hold_active_h(lidar_alt_m) : 0.0f;
            bool vel_enabled = (alt_ready && aid_valid &&
                                vel_alt_m >= VEL_ENABLE_ALT_M);

            if (vel_enabled) {
                // Above enable alt: fwd->pitch, right->roll.
                // pitch: fwd demand -> nose-down, so negate (kp*err-kd*a, fwd+).
                target_pitch_rad = -vc_pitch_cmd;
                // roll: right demand -> right bank, no negation. VERIFY ON GROUND.
                target_roll_rad = vc_roll_cmd;
            } else {
                // Below enable alt (or invalid): output NOT applied, fall back to
                // RC direct angles so takeoff/landing stay hand-flyable. Reset the
                // velocity controllers so the integral does not wind up while gated
                // (it would otherwise dump a large bias at the enable-alt edge).
                autopilot.reset_velocity();
                float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
                target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
                float target_roll_deg = -((int)rc_pulse_us[RC_CH_ROLL] + RC_TRIM_ROLL - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
                target_roll_rad = target_roll_deg * DEG_TO_RAD;
            }
        }

        // RC -> target yaw rate (deg/s -> rad/s)
        float target_yaw_rate = -((int)rc_pulse_us[RC_CH_YAW] + RC_TRIM_YAW - 1500) * (RC_MAX_YAW_RATE_DPS / 400.0f) * DEG_TO_RAD;

        // RC throttle -> U1 (+ alt-hold override), then takeoff-FF + attitude
        // control + mixing -> motor output.
        ctrl_compute_thrust(cx);
        ctrl_apply_attitude(cx, target_roll_rad, target_pitch_rad, target_yaw_rate,
                            yaw_initialized, takeoff_ff_done);

        // ---- DebugFrame (50Hz) ----
        if (!debug_output_enabled) continue;

        DebugFrame df;
        ctrl_fill_debug_common(df, cx, target_roll_rad, target_pitch_rad, target_yaw_rate);

        // Group 3: velocity controller (X->u, Y->v); 0 in RC_CONTROL.
        // u_acc/v_acc log the FILTERED accel feedback (a_f, 12Hz LPF) that the
        // -kd*a_f damping term actually saw, not the raw free accel.
        df.u_cmd = vc_u_cmd;
        df.u_fb  = vc_u_fb;
        df.u_acc = use_vel_ctrl ? autopilot.vel_u_acc_f() : 0.0f;
        df.u_err = use_vel_ctrl ? autopilot.vel_u_error() : 0.0f;
        df.v_cmd = vc_v_cmd;
        df.v_fb  = vc_v_fb;
        df.v_acc = use_vel_ctrl ? autopilot.vel_v_acc_f() : 0.0f;
        df.v_err = use_vel_ctrl ? autopilot.vel_v_error() : 0.0f;

        // Group 4: PNG fields unused in this loop (RC/VEL); zero them.
        df.png_px       = 0.0f;
        df.png_py       = 0.0f;
        df.png_rng      = 0.0f;
        df.png_eta      = 0.0f;
        df.png_los      = 0.0f;
        df.png_yaw_rate = 0.0f;
        df.wp_idx       = 0;
        df.png_lat      = 0.0;
        df.png_lon      = 0.0;
        df.png_alt      = 0.0f;

        // Send at TELEM_FREQ_HZ (50Hz)
        ctrl_send_debug(df, rt_count, rt_dt_max);
    } // end while (current_mode == this_mode)
}

void loop_rc_control()
{
    run_control_loop(false);
}

void loop_vel_control()
{
    run_control_loop(true);
}

// ============================================================
// PNG Waypoint Guidance (MODE_PNG_GUIDANCE) - dedicated 200Hz loop
// ------------------------------------------------------------
// Standalone loop (NOT shared with run_control_loop) so the guidance logic is
// self-contained and the RC/VEL path is never touched. Behavior:
//   - Horizontal: proportional navigation (PNG) over hardcoded local-NED
//     waypoints. Position from GNSS PVT (0x7010, 4Hz, ZOH at 200Hz).
//       forward Vx (fixed PNG_VX_MS) -> fwd velocity controller -> pitch
//       yaw_rate = PNG_KP*Vx*sin(eta)/rng -> set_yaw_rate
//       roll = 0 (yaw steers the heading)
//   - Vertical / throttle / alt-hold: identical to RC/VEL (RC throttle +
//     CH_MODE alt-hold). Altitude is the pilot's responsibility.
//   - GNSS-fix loss (fix_type < 3) -> fall back to RC direct-angle control so
//     the pilot can take over. Re-engages PNG when the fix returns.
//   - Same safety set as RC/VEL: EMERGENCY, IMU watchdog, tilt protection,
//     low-throttle cutoff, RC-loss 500ms -> IDLE.
void loop_png_guidance()
{
    const FlightMode this_mode = MODE_PNG_GUIDANCE;

    static float target_yaw_rad = 0.0f;
    static bool  yaw_initialized = false;
    static bool  takeoff_ff_done = false;

    uint32_t rc_lost_since_ms = millis();
    bool     rc_was_valid     = true;

    // Realtime diagnostics accumulators (attached to DebugFrame each 50Hz window).
    static uint32_t rt_last_us = 0;
    static uint16_t rt_count   = 0;
    static float    rt_dt_max  = 0.0f;

    while (current_mode == this_mode) {

        // ---- Housekeeping (every iteration): Bridge/LiDAR/P8 uplink ----
        ctrl_housekeeping();

        // RC signal loss (500ms debounce). PNG is autonomous -> failsafe emland
        // descent (stay armed) instead of cutting motors in the air.
        if (ctrl_rc_loss(rc_lost_since_ms, rc_was_valid, true)) break;

        ins.update();

        if (!time_ready()) continue;  // busy-wait for 200Hz
        update_time();

        float dt = (float)SAMPLING_TIME;

        // ---- Realtime diagnostics: actual 200Hz tick period ----
        ctrl_diag_tick(rt_last_us, rt_count, rt_dt_max);

        // ---- Periodic tasks (inside 200Hz tick): battery + LEDs ----
        ctrl_periodic_tasks();

        // ---- 200Hz control logic ----
        if (ins.available())
            last_imu_data_ms = millis();

        // IMU watchdog
        if ((millis() - last_imu_data_ms) >= IMU_TIMEOUT_MS) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // ---- Read IMU sample + control source + alt-hold edge into context ----
        // PNG: altitude-hold ALWAYS on (throttle is always an alt command);
        // CH_MODE only toggles guidance (AUTO) vs manual attitude (MANUAL).
        ControlContext cx;
        cx.dt = dt;
        cx.force_alt_hold = true;
        ctrl_read_sensors(cx);
        ControlSource ctrl = cx.ctrl;

        if (ctrl == CTRL_EMERGENCY) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            emland_active   = false;   // RC EMERGENCY clears the emland latch
            continue;
        }

        MtiData &d = cx.d;

        // ---- Altitude estimation (aiding + CF + FMF + h_used/v_used) ----
        ctrl_update_altitude_est(cx);
        float lidar_alt_m = cx.lidar_alt_m;
        bool  aid_valid   = cx.aid_valid;
        float yaw_rad     = cx.yaw_rad;

        if (!yaw_initialized) {
            target_yaw_rad = yaw_rad;
            yaw_initialized = true;
        }

        // Tilt protection
        if (cx.current_roll_deg > TILT_LIMIT_DEG || cx.current_roll_deg < -TILT_LIMIT_DEG ||
            cx.current_pitch_deg > TILT_LIMIT_DEG || cx.current_pitch_deg < -TILT_LIMIT_DEG) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            takeoff_ff_done = false;
            continue;
        }

        // GNSS/RTK velocity (NED N/E) rotated to body by yaw.
        ctrl_rotate_gnss_vel(cx);
        float cpsi_b = cx.cpsi_b, spsi_b = cx.spsi_b;
        float gnss_vel_u = cx.gnss_vel_u;   // body forward (u)
        float gnss_vel_v = cx.gnss_vel_v;   // body right   (v)

        // ---- Target attitude / yaw rate (PNG core) ----
        float target_roll_rad  = 0.0f;
        float target_pitch_rad = 0.0f;
        float target_yaw_rate  = 0.0f;

        // Velocity-controller logging (u=forward, v=right).
        float vc_u_cmd = 0.0f, vc_u_fb = 0.0f, vc_u_acc = 0.0f;
        float vc_v_cmd = 0.0f, vc_v_fb = 0.0f, vc_v_acc = 0.0f;

        // PNG logging (0 on fix loss).
        float png_px_log = 0.0f, png_py_log = 0.0f;
        float png_rng_log = 0.0f, png_eta_log = 0.0f, png_yaw_rate_log = 0.0f;
        float png_los_log = 0.0f;
        // Raw GNSS PVT used by PNG this step (0 unless active guidance ran).
        double png_lat_log = 0.0, png_lon_log = 0.0;
        float  png_alt_log = 0.0f;

        // PNG usable? Need origin captured + GNSS PVT 3D fix (fix_type>=3).
        bool png_fix_ok = png_origin_set &&
                          (d.data_available & HAS_GNSS_PVT) &&
                          (d.gnss_pvt.fix_type >= 3);

        // Altitude gate (same as VEL): forward velocity ctrl only above enable alt.
        float vel_alt_m = (alt_ready && aid_valid)
                          ? alt_hold_active_h(lidar_alt_m) : 0.0f;
        bool vel_enabled = (alt_ready && aid_valid && vel_alt_m >= VEL_ENABLE_ALT_M);

        // PNG mode behavior (altitude is always RC throttle, applied later):
        //   AUTO  (CH_MODE high):
        //       - fix + above enable alt -> PNG guidance (fwd Vx -> pitch, yaw
        //         rate = PN command). Y (right) body velocity is held at 0 by
        //         the right velocity controller (NOT roll=0): roll is actively
        //         driven to cancel any sideways drift.
        //       - RTK fix lost / below enable alt -> HOLD: command zero body
        //         velocity (X and Y) and zero yaw rate so the craft stabilizes
        //         in place (NOT a motor cut). The pilot can recover by flipping
        //         to MANUAL. Final waypoint reached -> guidance already commands
        //         Vx=0, yaw_rate=0, i.e. the same in-place hold.
        //   MANUAL (CH_MODE low):
        //       - RC direct angle (roll/pitch) + RC yaw rate. Takes over from a
        //         HOLD or active guidance immediately.
        // In every AUTO sub-case the right velocity command is 0, so the craft
        // holds zero sideways velocity (station-keeps laterally) rather than just
        // levelling roll.
        //
        // Emergency landing overrides AUTO/MANUAL (but not RC EMERGENCY, handled
        // above): force the in-place hold attitude path so the craft stays level
        // while ctrl_compute_thrust drives the fixed-thrust descent. Latched, so
        // a MANUAL flip does NOT recover stick control until cleared.
        const bool png_auto = (ctrl == CTRL_AUTO) || emland_active;

        if (png_auto) {
            // Body free-accel rotated to forward (x) and right (y) axes
            // (velocity-controller damping feedback).
            float accN = (float)d.data[FREE_ACC_X], accE = (float)d.data[FREE_ACC_Y];
            float u_acc =  accN * cpsi_b + accE * spsi_b;   // body forward
            float v_acc = -accN * spsi_b + accE * cpsi_b;   // body right

            float u_cmd;   // forward velocity command [m/s]
            if (emland_active) {
                // Emergency landing: zero all horizontal velocity + yaw rate. The
                // velocity controllers hold the craft level; descent comes from
                // the fixed U1 in ctrl_compute_thrust.
                u_cmd           = 0.0f;
                target_yaw_rate = 0.0f;
#if USE_OBS_AVOID
                cc.reset(); g_ccd = CCDebug{};   // no guidance -> clear avoidance
#endif
            } else if (png_fix_ok && vel_enabled) {
                // ---- PNG guidance: position + heading -> yaw rate + Vx ----
                // Guidance math + waypoint capture live in png_compute_guidance().
                png_lat_log = d.gnss_pvt.lat / 1e7;
                png_lon_log = d.gnss_pvt.lon / 1e7;
                png_alt_log = (float)(d.gnss_pvt.height / 1000.0);
                // MODE_PNG flies at a fixed altitude (no WP-altitude ramp), so the
                // alt_now_m anchor is unused here -> pass 0. Only g.yaw_rate/u_cmd
                // are consumed; g.alt_cmd is ignored in this mode.
                PngGuidance g = png_compute_guidance(png_lat_log,
                                                     png_lon_log,
                                                     d.gnss_pvt.height / 1000.0,
                                                     yaw_rad, 0.0f, gnss_vel_u);
                png_px_log = g.px; png_py_log = g.py;
                png_rng_log = g.rng; png_eta_log = g.eta;
                png_yaw_rate_log = g.yaw_rate; png_los_log = g.los;
                u_cmd            = g.u_cmd;     // 0 once final WP reached -> hold

                // ---- Collision-cone yaw-rate avoidance ----
                // Substitute r_ca for the PNG waypoint yaw-rate (r_wp) while the
                // avoid-mode latch is ON. Forward speed (u_cmd) stays PNG_VX.
                // PNG standalone mode keeps avoidance ALWAYS on (test/manual),
                // EXCEPT while orbiting (WP_FLAG_ORBIT): the orbit yaw-rate must
                // not be overridden, so avoidance is disabled for the turn.
                float r_wp = g.yaw_rate;        // 0 once final WP reached -> hold
#if USE_OBS_AVOID
                target_yaw_rate = ctrl_apply_avoidance(cx, /*enable=*/!g.orbit_active, r_wp);
#else
                target_yaw_rate = r_wp;
#endif
            } else {
                // RTK fix lost / below enable alt: hold in place. Command zero
                // forward velocity and zero yaw rate (Y also 0). Motors stay on;
                // the velocity controllers level/station-keep the craft.
                u_cmd           = 0.0f;
                target_yaw_rate = 0.0f;
#if USE_OBS_AVOID
                cc.reset(); g_ccd = CCDebug{};   // no guidance -> clear avoidance
#endif
            }

            // Forward/right velocity controller. Y (right) command fixed at 0 so
            // the right controller actively drives roll to null sideways velocity
            // (station-keep), not just hold roll level. fwd demand -> pitch.
            float vc_pitch_cmd, vc_roll_cmd;
            autopilot.update_velocity(u_cmd, gnss_vel_u, u_acc,
                                      0.0f, gnss_vel_v, v_acc,
                                      cx.dt, vc_pitch_cmd, vc_roll_cmd);
            vc_u_cmd = u_cmd;
            vc_u_fb  = gnss_vel_u;
            vc_u_acc = u_acc;
            vc_v_cmd = 0.0f;
            vc_v_fb  = gnss_vel_v;
            vc_v_acc = v_acc;

            // fwd demand -> nose-down -> negate. yaw steers heading; roll is
            // driven by the right velocity controller to hold Y velocity = 0.
            target_pitch_rad = -vc_pitch_cmd;
            target_roll_rad  = vc_roll_cmd;   // same sign mapping as VEL_CONTROL
        } else {
            // MANUAL: hand-fly with RC sticks (direct angle + RC yaw rate).
            // Reset the velocity controller so a later AUTO re-engage starts clean.
            autopilot.reset_velocity();
#if USE_OBS_AVOID
            cc.reset(); g_ccd = CCDebug{};   // manual takeover -> clear avoidance
#endif
            float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            float target_roll_deg  = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
            target_roll_rad  = target_roll_deg  * DEG_TO_RAD;
            target_yaw_rate  = -((int)rc_pulse_us[RC_CH_YAW] + RC_TRIM_YAW - 1500) * (RC_MAX_YAW_RATE_DPS / 400.0f) * DEG_TO_RAD;
        }

        // RC throttle -> U1 (+ alt-hold override), then takeoff-FF + attitude
        // control + mixing -> motor output. Identical to RC/VEL.
        ctrl_compute_thrust(cx);
        ctrl_apply_attitude(cx, target_roll_rad, target_pitch_rad, target_yaw_rate,
                            yaw_initialized, takeoff_ff_done);

        // ---- DebugFrame (50Hz) ----
        if (!debug_output_enabled) continue;

        DebugFrame df;
        ctrl_fill_debug_common(df, cx, target_roll_rad, target_pitch_rad, target_yaw_rate);

        // Group 3: velocity controller (X->u fwd, Y->v right). AUTO: PNG guidance
        // or in-place hold (v holds Y velocity = 0). 0 in MANUAL.
        // u_acc/v_acc log the FILTERED accel feedback (a_f, 12Hz LPF) the
        // -kd*a_f damping term actually saw, not the raw free accel.
        df.u_cmd = vc_u_cmd;
        df.u_fb  = vc_u_fb;
        df.u_acc = png_auto ? autopilot.vel_u_acc_f() : 0.0f;
        df.u_err = png_auto ? autopilot.vel_u_error() : 0.0f;
        df.v_cmd = vc_v_cmd;
        df.v_fb  = vc_v_fb;
        df.v_acc = png_auto ? autopilot.vel_v_acc_f() : 0.0f;
        df.v_err = png_auto ? autopilot.vel_v_error() : 0.0f;

        // Group 4: PNG guidance telemetry.
        df.png_px       = png_px_log;
        df.png_py       = png_py_log;
        df.png_rng      = png_rng_log;
        df.png_eta      = png_eta_log;
        df.png_los      = png_los_log;
        df.png_yaw_rate = png_yaw_rate_log;
        df.wp_idx       = (uint8_t)png_wp_idx;
        df.png_lat      = png_lat_log;
        df.png_lon      = png_lon_log;
        df.png_alt      = png_alt_log;

        // Group 4b: collision-cone avoidance (g_ccd from the active guidance
        // step; zeroed on emland/HOLD/MANUAL above). Defaults set in
        // ctrl_fill_debug_common, so only overwrite when avoidance is compiled in.
#if USE_OBS_AVOID
        df.obs_r_ca      = g_ccd.r_ca;
        df.obs_Ro        = g_ccd.Ro;
        df.obs_eo        = g_ccd.e_o;
        df.obs_gamma     = g_ccd.gamma;
        df.obs_theta_dot = g_ccd.theta_dot;
        df.obs_avoid     = g_ccd.avoid_mode ? 1 : 0;
#endif

        // Send at TELEM_FREQ_HZ (50Hz)
        ctrl_send_debug(df, rt_count, rt_dt_max);
    } // end while (current_mode == this_mode)
}

// ============================================================
// Position Control (MODE_POS_CONTROL) - dedicated 200Hz loop
// ------------------------------------------------------------
// Mirrors loop_png_guidance() (shared ctrl_* helpers, same safety set and
// altitude/throttle path) but swaps the guidance core for a step-setpoint
// position controller:
//   - Horizontal: targets the active waypoint directly (no interpolation).
//     The NED position PID (posN/posE) maps position error -> tilt, rotated by
//     yaw into body roll/pitch (update_position). The velocity inner loop is
//     not used; POS_MAX_TILT_DEG caps the lean on a large step error.
//   - yaw: held at the takeoff heading (target_yaw_rate = 0).
//   - Vertical / throttle / alt-hold: identical to PNG (RC throttle + alt-hold).
//   - GNSS-fix loss / below enable alt -> level hold (zero tilt); pilot recovers
//     via MANUAL. Same EMERGENCY / IMU watchdog / tilt / RC-loss safety as PNG.
// Telemetry reuses the PNG DebugFrame fields (no GCS rebuild): png_px/py = NED
// position, png_rng = distance to active WP, png_eta = 0 (no segment), and the
// velocity group logs the position PID's NED-axis tilt/error.
void loop_pos_control()
{
    const FlightMode this_mode = MODE_POS_CONTROL;

    static float target_yaw_rad = 0.0f;
    static bool  yaw_initialized = false;
    static bool  takeoff_ff_done = false;

    uint32_t rc_lost_since_ms = millis();
    bool     rc_was_valid     = true;

    static uint32_t rt_last_us = 0;
    static uint16_t rt_count   = 0;
    static float    rt_dt_max  = 0.0f;

    while (current_mode == this_mode) {

        ctrl_housekeeping();

        // Autonomous mode -> failsafe emland descent on RC loss (stay armed),
        // never cut motors in the air.
        if (ctrl_rc_loss(rc_lost_since_ms, rc_was_valid, true)) break;

        ins.update();

        if (!time_ready()) continue;
        update_time();

        float dt = (float)SAMPLING_TIME;

        ctrl_diag_tick(rt_last_us, rt_count, rt_dt_max);
        ctrl_periodic_tasks();

        if (ins.available())
            last_imu_data_ms = millis();

        // IMU watchdog
        if ((millis() - last_imu_data_ms) >= IMU_TIMEOUT_MS) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // POS: altitude-hold ALWAYS on (throttle is always an alt command);
        // CH_MODE only toggles position control (AUTO) vs manual attitude.
        ControlContext cx;
        cx.dt = dt;
        cx.force_alt_hold = true;
        ctrl_read_sensors(cx);
        ControlSource ctrl = cx.ctrl;

        if (ctrl == CTRL_EMERGENCY) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            emland_active   = false;
            continue;
        }

        MtiData &d = cx.d;

        // ---- Altitude estimation (aiding + CF + FMF + h_used/v_used) ----
        ctrl_update_altitude_est(cx);
        float lidar_alt_m = cx.lidar_alt_m;
        bool  aid_valid   = cx.aid_valid;
        float yaw_rad     = cx.yaw_rad;

        if (!yaw_initialized) {
            target_yaw_rad = yaw_rad;
            yaw_initialized = true;
        }

        // Tilt protection
        if (cx.current_roll_deg > TILT_LIMIT_DEG || cx.current_roll_deg < -TILT_LIMIT_DEG ||
            cx.current_pitch_deg > TILT_LIMIT_DEG || cx.current_pitch_deg < -TILT_LIMIT_DEG) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            takeoff_ff_done = false;
            continue;
        }

        // GNSS/RTK velocity rotated to body (for u/v logging) + cpsi/spsi cache.
        // The position PID's D term uses RAW NED velocity (VEL_X/VEL_Y), not the
        // body-rotated u/v.
        ctrl_rotate_gnss_vel(cx);
        float gnss_vel_u = cx.gnss_vel_u;   // body forward (u), logging only
        float gnss_vel_v = cx.gnss_vel_v;   // body right   (v), logging only
        float vel_n = (float)d.data[VEL_X]; // RAW NED north velocity [m/s]
        float vel_e = (float)d.data[VEL_Y]; // RAW NED east  velocity [m/s]

        // ---- Target attitude (position-control core) ----
        float target_roll_rad  = 0.0f;
        float target_pitch_rad = 0.0f;
        // Yaw heading-hold: P term on the wrap_pi heading error -> yaw-rate cmd
        // (set per-branch below). AUTO holds the captured takeoff heading; emland
        // / fix-lost / MANUAL override it. Replaces the old constant rate=0 hold,
        // which had no restoring force and let the heading drift.
        float target_yaw_rate  = 0.0f;
        const float yaw_rate_max = RC_MAX_YAW_RATE_DPS * DEG_TO_RAD;

        // Telemetry (reuses PNG fields; 0 unless position control ran this step).
        float pos_px_log = 0.0f, pos_py_log = 0.0f;  // current NED position [m]
        float pos_rng_log = 0.0f;                    // distance to active WP [m]
        double pos_lat_log = 0.0, pos_lon_log = 0.0;
        float  pos_alt_log = 0.0f;
        // Position PID tilt/err diagnostics (NED axes), logged via the vel group.
        float pid_tilt_n = 0.0f, pid_tilt_e = 0.0f;
        float pid_err_n  = 0.0f, pid_err_e  = 0.0f;

        bool pos_fix_ok = png_origin_set &&
                          (d.data_available & HAS_GNSS_PVT) &&
                          (d.gnss_pvt.fix_type >= 3);

        // Altitude gate (same as VEL/PNG): position control only above enable alt.
        float pos_alt_m = (alt_ready && aid_valid)
                          ? alt_hold_active_h(lidar_alt_m) : 0.0f;
        bool pos_enabled = (alt_ready && aid_valid && pos_alt_m >= VEL_ENABLE_ALT_M);

        const bool pos_auto = (ctrl == CTRL_AUTO) || emland_active;

        if (pos_auto) {
            if (emland_active) {
                // Emergency landing: command level (zero tilt) + zero yaw. The
                // fixed U1 in ctrl_compute_thrust drives the descent.
                target_pitch_rad = 0.0f;
                target_roll_rad  = 0.0f;
                target_yaw_rate  = 0.0f;
                autopilot.reset_position();
            } else if (pos_fix_ok && pos_enabled) {
                // ---- Step setpoint (active WP) -> position PID -> roll/pitch ----
                pos_lat_log = d.gnss_pvt.lat / 1e7;
                pos_lon_log = d.gnss_pvt.lon / 1e7;
                pos_alt_log = (float)(d.gnss_pvt.height / 1000.0);

                // Current local-NED position (origin-ref) from the GNSS PVT fix.
                float pos_n, pos_e;
                png_lla_to_ned(pos_lat_log, pos_lon_log,
                               d.gnss_pvt.height / 1000.0, pos_n, pos_e);

                // Target the active waypoint directly; advance on arrival
                // (actual distance < POS_ARRIVE_R). Tilt is bounded by the
                // position PID's POS_MAX_TILT_DEG limit, so the step error does
                // not produce a violent lean.
                PosTarget tg = pos_update_target(pos_n, pos_e);

                // Position PID (NED 2-axis) -> yaw rotation -> body roll/pitch.
                autopilot.update_position(tg.sp_n, tg.sp_e, pos_n, pos_e,
                                          vel_n, vel_e, yaw_rad, dt,
                                          target_pitch_rad, target_roll_rad);

                // Yaw heading-hold: P on the wrap_pi heading error -> yaw-rate
                // cmd (saturated). Holds the captured takeoff heading so the NED
                // tilt rotation stays consistent and the path does not drift.
                float yaw_err   = wrap_pi(target_yaw_rad - yaw_rad);
                target_yaw_rate = POS_YAW_HOLD_KP * yaw_err;
                if (target_yaw_rate >  yaw_rate_max) target_yaw_rate =  yaw_rate_max;
                if (target_yaw_rate < -yaw_rate_max) target_yaw_rate = -yaw_rate_max;

                pos_px_log  = pos_n;
                pos_py_log  = pos_e;
                pos_rng_log = tg.rng;
                pid_tilt_n = autopilot.pos_n_tilt();
                pid_tilt_e = autopilot.pos_e_tilt();
                pid_err_n  = autopilot.pos_n_err();
                pid_err_e  = autopilot.pos_e_err();
            } else {
                // Fix lost / below enable alt: level hold (zero tilt). Motors stay
                // on; pilot can recover via MANUAL. Reset the position integrators.
                target_pitch_rad = 0.0f;
                target_roll_rad  = 0.0f;
                target_yaw_rate  = 0.0f;
                autopilot.reset_position();
            }
        } else {
            // MANUAL: hand-fly with RC sticks (direct angle + RC yaw rate).
            autopilot.reset_position();
            float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            float target_roll_deg  = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
            target_roll_rad  = target_roll_deg  * DEG_TO_RAD;
            target_yaw_rate  = -((int)rc_pulse_us[RC_CH_YAW] + RC_TRIM_YAW - 1500) * (RC_MAX_YAW_RATE_DPS / 400.0f) * DEG_TO_RAD;
        }

        // RC throttle -> U1 (+ alt-hold override), then takeoff-FF + attitude
        // control + mixing -> motor output. Identical to RC/VEL/PNG.
        ctrl_compute_thrust(cx);
        ctrl_apply_attitude(cx, target_roll_rad, target_pitch_rad, target_yaw_rate,
                            yaw_initialized, takeoff_ff_done);

        // ---- DebugFrame (50Hz) ----
        if (!debug_output_enabled) continue;

        DebugFrame df;
        ctrl_fill_debug_common(df, cx, target_roll_rad, target_pitch_rad, target_yaw_rate);

        // Group 3 (velocity group, reused): position PID NED-axis tilt/err.
        //   u_* <- North axis (cmd=tilt, fb=vel, err=pos_err)
        //   v_* <- East  axis
        df.u_cmd = pid_tilt_n;   // North tilt command [rad]
        df.u_fb  = vel_n;        // RAW NED north velocity [m/s]
        df.u_acc = 0.0f;
        df.u_err = pid_err_n;    // North position error [m]
        df.v_cmd = pid_tilt_e;   // East tilt command [rad]
        df.v_fb  = vel_e;        // RAW NED east velocity [m/s]
        df.v_acc = 0.0f;
        df.v_err = pid_err_e;    // East position error [m]

        // Group 4 (PNG group, reused): position-control telemetry.
        df.png_px       = pos_px_log;    // current NED north [m]
        df.png_py       = pos_py_log;    // current NED east  [m]
        df.png_rng      = pos_rng_log;   // distance to active waypoint [m]
        df.png_eta      = 0.0f;          // unused (no segment progress in step mode)
        df.png_los      = 0.0f;          // unused (no guidance LOS in step mode)
        df.png_yaw_rate = 0.0f;          // yaw held (no rate command)
        df.wp_idx       = (uint8_t)pos_wp_idx;
        df.png_lat      = pos_lat_log;
        df.png_lon      = pos_lon_log;
        df.png_alt      = pos_alt_log;

        ctrl_send_debug(df, rt_count, rt_dt_max);
    } // end while (current_mode == this_mode)
}

// ============================================================
// Vertical takeoff/landing sequencer (shared by ATL and MISSION)
// ------------------------------------------------------------
// One step of the vertical state machine. Owns ONLY the vertical axis (U1) and
// the GROUND/SPOOLUP/.../DISARMED phase transitions. The caller owns horizontal
// control (position hold / PNG guidance), reading st.state + the returned flags
// to decide what to do each step.
//
//   req_takeoff   : GCS CMD_AUTO_TAKEOFF latch (GROUND -> SPOOLUP)
//   req_mission   : GCS CMD_MISSION_START latch (HOLD -> GUIDANCE; MISSION only)
//   req_land      : GCS CMD_AUTO_LAND latch (HOLD -> LAND)
//   guidance_done : caller signals last waypoint reached (GUIDANCE -> LAND)
//   emland        : emergency-landing latch (RC loss / GCS 'e'). When set while
//                   airborne, the sequencer jumps straight to ATL_LAND_SLOW and
//                   descends at the fixed EMLAND_THRUST_FRAC*mg thrust, holding
//                   the current position, then runs the normal touchdown ->
//                   SPOOLDOWN -> DISARMED stop sequence. This makes emergency
//                   landing self-terminating (motors stop on touchdown) instead
//                   of holding thrust forever with the state frozen.
//
// Returns U1 / motors_run plus horiz_hold (caller holds a fixed point) and
// guidance_run (caller runs PNG guidance, ATL_GUIDANCE only). Behaviour for ATL
// is identical to the old inline switch as long as req_mission is never set
// (ATL never enters ATL_GUIDANCE) and emland is false.
static SeqOut atl_sequencer_step(ControlContext &cx, SeqState &st,
                                 float alt_now_m, float v_up,
                                 bool pos_fix_ok, bool aid_valid,
                                 bool req_takeoff, bool req_mission, bool req_land,
                                 bool guidance_done, bool emland)
{
    const MtiData &d = cx.d;
    const float mg   = MASS_KG * 9.81f;

    SeqOut o;
    o.U1           = 0.0f;
    o.motors_run   = true;
    o.horiz_hold   = false;
    o.guidance_run = false;

    float climb_cmd  = 0.0f;
    uint16_t phase_ms = 0;

    // Emergency landing: while airborne, divert the sequence straight to the slow
    // descent (LAND_SLOW). It holds position (horiz_hold), descends at the fixed
    // emland thrust (applied below), and reuses the touchdown -> SPOOLDOWN ->
    // DISARMED stop path so the motors stop on the ground. One-shot: only divert
    // from an airborne flight state, never from the ground/stop states.
    if (emland && (st.state == ATL_SPOOLUP || st.state == ATL_TAKEOFF ||
                   st.state == ATL_HOLD    || st.state == ATL_GUIDANCE ||
                   st.state == ATL_LAND)) {
        autopilot.reset_altitude();
        st.state       = ATL_LAND_SLOW;
        st.td_start_ms = 0;
    }

    switch (st.state) {
    case ATL_GROUND:
        // Idle on the ground; wait for a takeoff request. Motors stopped.
        o.motors_run = false;
        autopilot.reset_altitude();
        if (req_takeoff) {
            // Re-check start conditions at the actual transition.
            if (pos_fix_ok && alt_ready && aid_valid &&
                alt_now_m <= TKO_START_ALT_MAX) {
                st.state          = ATL_SPOOLUP;   // spool motors before climb
                st.phase_start_ms = millis();
                o.motors_run      = true;
            }
        }
        break;

    case ATL_SPOOLUP: {
        // Hold a sub-hover thrust (open-loop) so the ESCs/props ramp up smoothly
        // instead of snapping to a climb command (high-current step). After
        // ATL_SPOOLUP_MS, hand off to the climb controller.
        o.motors_run = true;
        o.U1 = ATL_SPOOLUP_FRAC * mg;
        autopilot.reset_altitude();            // keep the integral idle
        uint32_t el = millis() - st.phase_start_ms;
        phase_ms = (el < ATL_SPOOLUP_MS) ? (uint16_t)(ATL_SPOOLUP_MS - el) : 0;
        if (el >= (uint32_t)ATL_SPOOLUP_MS) {
            // Re-capture the NED origin at the START of liftoff. The origin
            // captured at mode entry is stale by now (ground wait + 3s spool-up
            // during which GNSS drifts). Re-anchor so the takeoff-point hold
            // (target = origin 0,0) starts from the craft's ACTUAL position.
            if (pos_fix_ok) {
                png_capture_origin(d.gnss_pvt.lat / 1e7,
                                   d.gnss_pvt.lon / 1e7,
                                   d.gnss_pvt.height / 1000.0);
                autopilot.reset_position();   // clear any stale integral
            }
            st.state = ATL_TAKEOFF;
        }
        break;
    }

    case ATL_TAKEOFF:
        // Climb at a constant rate until the target altitude, then capture the
        // current altitude as the hold setpoint (bumpless) -> HOLD.
        o.horiz_hold = true;
        climb_cmd = TKO_CLIMB_RATE;
        o.U1 = autopilot.update_climb_rate(climb_cmd, v_up,
                                           (float)d.data[FREE_ACC_Z], mg, cx.dt);
        if (alt_now_m >= TKO_TARGET_ALT) {
            st.hold_h = alt_now_m;          // capture current altitude
            autopilot.reset_altitude();     // bump-free hand-off
            st.state = ATL_HOLD;
        }
        break;

    case ATL_HOLD:
        // Altitude-hold at the captured setpoint; wait for a land or mission cmd.
        o.horiz_hold = true;
        o.U1 = autopilot.update_altitude(cx.h_used, cx.v_used,
                                         (float)d.data[FREE_ACC_Z],
                                         st.hold_h, mg, cx.dt);
        if (req_land) {
            autopilot.reset_altitude();     // clear integral before climb-rate
            autopilot.reset_position();     // LAND uses position hold: start clean
            st.state = ATL_LAND;
        } else if (req_mission) {
            // MISSION: hand horizontal control to PNG guidance (vertical stays
            // alt-hold at the captured target). ATL never sets req_mission.
            // Clear the velocity-controller integral so guidance starts from a
            // clean state (position hold ran the pos PID, not velU/velV).
            autopilot.reset_velocity();
            st.state = ATL_GUIDANCE;
        }
        break;

    case ATL_GUIDANCE:
        // (MISSION only) Vertical = alt-hold at the captured target; the caller
        // runs PNG guidance for horizontal. When the caller reaches the final
        // waypoint (guidance_done) -> descend (LAND) at the current position.
        o.guidance_run = true;
        o.U1 = autopilot.update_altitude(cx.h_used, cx.v_used,
                                         (float)d.data[FREE_ACC_Z],
                                         st.hold_h, mg, cx.dt);
        if (guidance_done || req_land) {
            autopilot.reset_altitude();     // clear before climb-rate (descent)
            autopilot.reset_position();     // LAND uses position hold; drop the
                                            // stale (frozen) integral from before
                                            // guidance so it starts clean.
            st.state = ATL_LAND;
        }
        break;

    case ATL_LAND:
        // Descend at the high rate until low altitude, then slow down.
        o.horiz_hold = true;
        climb_cmd = -LAND_SPEED_HIGH;
        o.U1 = autopilot.update_climb_rate(climb_cmd, v_up,
                                           (float)d.data[FREE_ACC_Z], mg, cx.dt);
        if (alt_now_m < LAND_SLOW_ALT) {
            st.state = ATL_LAND_SLOW;
            st.td_start_ms = 0;
        }
        break;

    case ATL_LAND_SLOW:
        // Slow descent + touchdown detection.
        // Normal landing: closed-loop slow-descent rate; touchdown = LiDAR alt
        //   low AND vertical speed settled (alt_now_m + v_up, both LiDAR-derived).
        // Emergency landing: FIXED EMLAND_THRUST_FRAC*mg open-loop descent. Its
        //   touchdown detection must NOT use the LiDAR/altitude estimate (the
        //   craft may be here precisely because the LiDAR failed - alt_now_m/v_up
        //   would then read 0 and falsely trigger touchdown in mid-air). Instead
        //   it uses ONLY the MTi GNSS/INS vertical velocity (vel_down_latest,
        //   LiDAR-independent): on the ground the descent stalls and |w|->0.
        o.horiz_hold = true;
        if (emland) {
            climb_cmd = 0.0f;                        // n/a (open-loop thrust)
            o.U1 = EMLAND_THRUST_FRAC * mg;
            autopilot.reset_altitude();              // keep the integral idle
        } else {
            climb_cmd = -LAND_SPEED_SLOW;
            o.U1 = autopilot.update_climb_rate(climb_cmd, v_up,
                                               (float)d.data[FREE_ACC_Z], mg, cx.dt);
        }
        {
            bool td_ok;
            if (emland) {
                // LiDAR-independent: MTi GNSS/INS vertical speed settled to ~0.
                td_ok = (fabsf(vel_down_latest) < EMLAND_TD_V);
            } else {
                td_ok = (alt_now_m < TD_ALT) && (fabsf(v_up) < TD_V);
            }
            uint32_t td_hold = emland ? (uint32_t)EMLAND_TD_TIME_MS
                                      : (uint32_t)TD_TIME_MS;
            if (td_ok) {
                if (st.td_start_ms == 0) st.td_start_ms = millis();
                if ((millis() - st.td_start_ms) >= td_hold) {
                    st.state = ATL_SPOOLDOWN;  // touchdown -> spool down
                    st.phase_start_ms = millis();
                }
            } else {
                st.td_start_ms = 0;            // reset persistence timer
            }
        }
        break;

    case ATL_SPOOLDOWN: {
        // After touchdown, hold a low thrust (open-loop) so the craft settles on
        // the ground before the motors stop completely.
        o.motors_run = true;
        o.U1 = ATL_SPOOLDOWN_FRAC * mg;
        autopilot.reset_position();
        autopilot.reset_altitude();
        uint32_t el = millis() - st.phase_start_ms;
        phase_ms = (el < ATL_SPOOLDOWN_MS) ? (uint16_t)(ATL_SPOOLDOWN_MS - el) : 0;
        if (el >= (uint32_t)ATL_SPOOLDOWN_MS) {
            st.state = ATL_DISARMED;
        }
        break;
    }

    case ATL_DISARMED:
        // Touched down + spooled down: motors off. Re-takeoff via command.
        o.motors_run = false;
        autopilot.reset();
        break;
    }

    st.climb_cmd_log = climb_cmd;
    st.phase_ms_log  = phase_ms;
    return o;
}

// ============================================================
// Auto Takeoff / Landing (MODE_AUTO_TKO_LAND) - dedicated 200Hz loop
// ------------------------------------------------------------
// Mirrors loop_pos_control() (shared ctrl_* helpers, same safety set) but:
//   - Horizontal: holds the takeoff-point (NED origin 0,0 captured at entry)
//     via posN/posE; yaw holds the takeoff heading (POS_YAW_HOLD_KP).
//   - Vertical: a state machine drives the AltitudePID INNER velocity loop
//     (update_climb_rate) for a constant climb/descent rate during TAKEOFF/LAND,
//     and update_altitude (alt-hold) during HOLD. GROUND/DISARMED stop motors.
//   - Triggered by GCS CMD_AUTO_TAKEOFF / CMD_AUTO_LAND (atl_*_request latches).
//   - Touchdown: LiDAR alt + vertical speed, time-persistent -> disarm.
// ctrl_compute_thrust is NOT used (it is RC-throttle/alt-hold specific); U1 is
// computed here per state. ctrl_apply_attitude is reused for FF + attitude +
// mixing, with cx.rc_throttle forced high while flying so its low-throttle
// cutoff does not fire (ATL owns the motor-stop decision).
void loop_auto_tko_land()
{
    const FlightMode this_mode = MODE_AUTO_TKO_LAND;

    static float target_yaw_rad  = 0.0f;
    static bool  yaw_initialized = false;
    static bool  takeoff_ff_done = false;

    uint32_t rc_lost_since_ms = millis();
    bool     rc_was_valid     = true;

    static uint32_t rt_last_us = 0;
    static uint16_t rt_count   = 0;
    static float    rt_dt_max  = 0.0f;

    const float mg           = MASS_KG * 9.81f;
    const float yaw_rate_max = RC_MAX_YAW_RATE_DPS * DEG_TO_RAD;

    // Vision helipad target latch (absolute NED). Local (not static) so it
    // resets to "no lock" on every mode (re-)entry. Once vision locks, the
    // landing hold follows this point; a vision dropout HOLDS the last latch
    // instead of snapping back to the takeoff origin.
    bool  vlatch_set = false;          // a valid helipad point is latched
    float vlatch_n   = 0.0f;           // latched helipad target, NED north [m]
    float vlatch_e   = 0.0f;           // latched helipad target, NED east  [m]

    while (current_mode == this_mode) {

        ctrl_housekeeping();

        // Autonomous mode -> failsafe emland descent on RC loss (stay armed),
        // never cut motors in the air.
        if (ctrl_rc_loss(rc_lost_since_ms, rc_was_valid, true)) break;

        ins.update();

        if (!time_ready()) continue;
        update_time();

        float dt = (float)SAMPLING_TIME;

        ctrl_diag_tick(rt_last_us, rt_count, rt_dt_max);
        ctrl_periodic_tasks();

        if (ins.available())
            last_imu_data_ms = millis();

        // IMU watchdog
        if ((millis() - last_imu_data_ms) >= IMU_TIMEOUT_MS) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // ATL: altitude-hold ALWAYS on (vertical is owned by the state machine);
        // CH_MODE only toggles AUTO (run sequence) vs MANUAL (hand-fly).
        ControlContext cx;
        cx.dt = dt;
        cx.force_alt_hold = true;
        ctrl_read_sensors(cx);
        ControlSource ctrl = cx.ctrl;

        if (ctrl == CTRL_EMERGENCY) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            emland_active   = false;
            continue;
        }

        MtiData &d = cx.d;

        // ---- Altitude estimation (aiding + CF + FMF + h_used/v_used) ----
        ctrl_update_altitude_est(cx);
        float lidar_alt_m = cx.lidar_alt_m;
        bool  aid_valid   = cx.aid_valid;
        float yaw_rad     = cx.yaw_rad;

        if (!yaw_initialized) {
            target_yaw_rad = yaw_rad;
            yaw_initialized = true;
        }

        // Tilt protection
        if (cx.current_roll_deg > TILT_LIMIT_DEG || cx.current_roll_deg < -TILT_LIMIT_DEG ||
            cx.current_pitch_deg > TILT_LIMIT_DEG || cx.current_pitch_deg < -TILT_LIMIT_DEG) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            takeoff_ff_done = false;
            continue;
        }

        // GNSS/RTK velocity rotated to body (logging) + RAW NED velocity (D-term).
        ctrl_rotate_gnss_vel(cx);
        float vel_n = (float)d.data[VEL_X]; // RAW NED north velocity [m/s]
        float vel_e = (float)d.data[VEL_Y]; // RAW NED east  velocity [m/s]

        // Altitude estimate (up+) and vertical speed (up+) for the state machine.
        float alt_now_m = (alt_ready && aid_valid) ? alt_hold_active_h(lidar_alt_m) : 0.0f;
        float v_up      = cx.v_used;        // up-positive vertical speed [m/s]

        // ---- Altitude-aiding loss -> automatic emergency landing ----
        // ATL has no RC fallback for alt-hold: a sustained aid loss would make the
        // alt-hold controller see alt_now=0 and command a runaway climb. If the
        // aid stays invalid this long while airborne, latch emland so the sequencer
        // diverts to a LiDAR-independent fixed-thrust descent. Brief dropouts are
        // already covered by the LiDAR spike-hold and never reach this timer.
        {
            static uint32_t aid_lost_since_ms = 0;
            const bool atl_airborne_aid = (atl_seq.state == ATL_SPOOLUP  ||
                                           atl_seq.state == ATL_TAKEOFF  ||
                                           atl_seq.state == ATL_HOLD     ||
                                           atl_seq.state == ATL_LAND     ||
                                           atl_seq.state == ATL_LAND_SLOW);
            if (aid_valid || !atl_airborne_aid) {
                aid_lost_since_ms = 0;
            } else {
                if (aid_lost_since_ms == 0) aid_lost_since_ms = millis();
                if (!emland_active &&
                    (millis() - aid_lost_since_ms) >= (uint32_t)AID_LOST_EMLAND_MS) {
                    emland_active = true;   // sequencer -> LiDAR-independent emland
                    #if USE_DEBUG_SERIAL
                    Serial.println("[ATL] altitude aid lost -> auto emland");
                    #endif
                }
            }
        }

        // ---- Target attitude ----
        float target_roll_rad  = 0.0f;
        float target_pitch_rad = 0.0f;
        float target_yaw_rate  = 0.0f;

        // Telemetry (reuses PNG/vel debug fields).
        float pos_px_log = 0.0f, pos_py_log = 0.0f;
        double pos_lat_log = 0.0, pos_lon_log = 0.0;
        float  pos_alt_log = 0.0f;
        float  pid_tilt_n = 0.0f, pid_tilt_e = 0.0f;
        float  pid_err_n  = 0.0f, pid_err_e  = 0.0f;

        // Vision precision-landing telemetry status (raw cam values are read
        // straight from the helipad_* globals when filling the DebugFrame).
        bool  vision_use    = false;   // steering to the latched helipad point
        bool  vision_reject = false;   // this step's fix rejected as a spike
        float helipad_dn_log = 0.0f;   // attitude-corrected NED-north offset [m]
        float helipad_de_log = 0.0f;   // attitude-corrected NED-east  offset [m]

        // Ground-test debug: compute the attitude-corrected (DCM) offset on EVERY
        // valid+fresh camera fix, regardless of flight phase, so dcm_dn/de can be
        // verified props-off on the bench (tilt the craft by hand and compare with
        // raw cam_x/cam_y). This is DISPLAY-ONLY - it never touches the latch or
        // the position setpoint. The flight latch path below recomputes the same
        // formula inside the LAND/LAND_SLOW gate and is what actually steers.
        #if USE_VISION_LANDING
        if (helipad_valid_raw &&
            (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS) {
            helipad_body_to_ned(helipad_x_m, helipad_y_m,
                                cx.roll_rad, cx.pitch_rad, cx.yaw_rad,
                                helipad_dn_log, helipad_de_log);
        }
        #endif

        bool pos_fix_ok = png_origin_set &&
                          (d.data_available & HAS_GNSS_PVT) &&
                          (d.gnss_pvt.fix_type >= 3);

        // Vertical control output (total thrust U1) and whether motors should run.
        float U1 = 0.0f;
        bool  motors_run = true;

        // emland (latched by RC-loss failsafe or GCS) overrides MANUAL. It is fed
        // to the sequencer, which (while airborne) diverts to ATL_LAND_SLOW and
        // runs a fixed-thrust descent that HOLDS POSITION (origin) and self-
        // terminates via the normal touchdown -> SPOOLDOWN -> DISARMED path. So
        // emland now goes through the AUTO branch (not a separate level-descent
        // block). On the ground (GROUND/DISARMED) the sequencer keeps the motors
        // stopped, so an emland on the pad never spins the props.
        const bool atl_auto = (ctrl == CTRL_AUTO) || emland_active;

        if (!atl_auto) {
            // ---- MANUAL: hand-fly with RC sticks; state machine paused ----
            // Pause (don't reset) the sequence so a brief MANUAL dip is recoverable.
            // Vertical = RC-throttle alt-hold (same as ctrl_compute_thrust path).
            autopilot.reset_position();
            float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            float target_roll_deg  = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
            target_roll_rad  = target_roll_deg  * DEG_TO_RAD;
            target_yaw_rate  = -((int)rc_pulse_us[RC_CH_YAW] + RC_TRIM_YAW - 1500) * (RC_MAX_YAW_RATE_DPS / 400.0f) * DEG_TO_RAD;
            // MANUAL vertical via the standard RC-throttle thrust path.
            ctrl_compute_thrust(cx);
            U1 = cx.U1;
            // If sticks are at idle, ctrl_apply_attitude's cutoff handles motor stop.
        } else {
            // ---- AUTO (incl. emland): run the takeoff/landing state machine ----
            // Vertical (U1 + phase transitions) is owned by the sequencer; the
            // horizontal takeoff-point hold stays here. ATL never sends a mission
            // request, so the sequencer never enters ATL_GUIDANCE. When emland is
            // set the sequencer diverts to LAND_SLOW (position-holding emergency
            // descent) instead of the inline switch.
            bool req_tko  = atl_takeoff_request; atl_takeoff_request = false;
            bool req_land = atl_land_request;    atl_land_request    = false;

            SeqOut so = atl_sequencer_step(cx, atl_seq, alt_now_m, v_up,
                                           pos_fix_ok, aid_valid,
                                           req_tko, false, req_land, false,
                                           emland_active);
            U1         = so.U1;
            motors_run = so.motors_run;

            // Horizontal: hold the takeoff point (NED origin 0,0). Enabled from a
            // low altitude (ATL_POS_ENABLE_ALT) so the craft holds position through
            // the whole low-altitude takeoff/landing (in-place takeoff/land), as
            // long as the fix is valid; below that, level hold.
            bool horiz_on = so.horiz_hold && pos_fix_ok &&
                            (alt_now_m >= ATL_POS_ENABLE_ALT);
            if (horiz_on) {
                pos_lat_log = d.gnss_pvt.lat / 1e7;
                pos_lon_log = d.gnss_pvt.lon / 1e7;
                pos_alt_log = (float)(d.gnss_pvt.height / 1000.0);
                float pos_n, pos_e;
                png_lla_to_ned(pos_lat_log, pos_lon_log,
                               d.gnss_pvt.height / 1000.0, pos_n, pos_e);

                // ---- Horizontal setpoint: GNSS takeoff-point vs vision helipad ----
                // Landing (LAND / LAND_SLOW): steer to the latched helipad point.
                //  * A fresh, valid, non-spike fix UPDATES the latch (absolute NED).
                //  * Invalid / stale / spike: HOLD the last latch (no snap-back).
                //  * No latch yet (or emland): hold the GNSS takeoff origin (0,0).
                // The latch is an ABSOLUTE NED point, so the craft keeps holding the
                // last good helipad position through a vision dropout.
                float sp_n = 0.0f, sp_e = 0.0f;
                #if USE_VISION_LANDING
                const bool land_phase = (atl_seq.state == ATL_LAND ||
                                         atl_seq.state == ATL_LAND_SLOW);
                // emland descends in place: never chase the helipad.
                if (land_phase && !emland_active) {
                    // Try to accept a new fix (fresh + valid).
                    if (helipad_valid_raw &&
                        (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS) {
                        // The tracker sends a LEVEL-body FRD offset (x=forward,
                        // y=right) with NO live attitude applied. Rotate it to NED
                        // with the FULL roll/pitch/yaw DCM (shared helper) and add
                        // the current NED position to get an absolute target.
                        float dn, de;
                        helipad_body_to_ned(helipad_x_m, helipad_y_m,
                                            cx.roll_rad, cx.pitch_rad, yaw_rad,
                                            dn, de);
                        helipad_dn_log = dn;   // attitude-corrected NED offset (debug)
                        helipad_de_log = de;
                        const float cand_n = pos_n + dn;
                        const float cand_e = pos_e + de;
                        // Tilt freeze: the level assumption (and depth geometry)
                        // breaks down past VISION_TILT_FREEZE_DEG, so don't accept
                        // a new fix while this tilted - hold the previous latch.
                        const float tilt_lim = VISION_TILT_FREEZE_DEG * DEG_TO_RAD;
                        const bool tilt_ok =
                            (cx.roll_rad  <  tilt_lim && cx.roll_rad  > -tilt_lim &&
                             cx.pitch_rad <  tilt_lim && cx.pitch_rad > -tilt_lim);
                        // Spike reject: ignore a fix that jumps the NED target more
                        // than VISION_MAX_JUMP_M from the current latch (first lock
                        // is always accepted).
                        bool accept = tilt_ok;
                        if (accept && vlatch_set) {
                            const float jn = cand_n - vlatch_n;
                            const float je = cand_e - vlatch_e;
                            accept = (jn * jn + je * je) <=
                                     (VISION_MAX_JUMP_M * VISION_MAX_JUMP_M);
                        }
                        if (accept) {
                            vlatch_n = cand_n; vlatch_e = cand_e; vlatch_set = true;
                        } else {
                            vision_reject = true;   // tilt/spike: held previous latch
                        }
                    }
                    // Use the latch if we ever locked (held through dropouts).
                    if (vlatch_set) {
                        sp_n = vlatch_n; sp_e = vlatch_e;
                        vision_use = true;
                    }
                }
                #endif // USE_VISION_LANDING

                // Target = latched helipad (vision_use) or NED origin (0,0).
                autopilot.update_position(sp_n, sp_e, pos_n, pos_e,
                                          vel_n, vel_e, yaw_rad, dt,
                                          target_pitch_rad, target_roll_rad);

                // Yaw heading-hold (same P term as POS).
                float yaw_err   = wrap_pi(target_yaw_rad - yaw_rad);
                target_yaw_rate = POS_YAW_HOLD_KP * yaw_err;
                if (target_yaw_rate >  yaw_rate_max) target_yaw_rate =  yaw_rate_max;
                if (target_yaw_rate < -yaw_rate_max) target_yaw_rate = -yaw_rate_max;

                pos_px_log = pos_n; pos_py_log = pos_e;
                pid_tilt_n = autopilot.pos_n_tilt();
                pid_tilt_e = autopilot.pos_e_tilt();
                pid_err_n  = autopilot.pos_n_err();
                pid_err_e  = autopilot.pos_e_err();
            } else {
                // Below enable alt / fix lost: level hold, integrators reset.
                target_pitch_rad = 0.0f;
                target_roll_rad  = 0.0f;
                target_yaw_rate  = 0.0f;
                autopilot.reset_position();
            }
        }

        // ---- Output ----
        if (!motors_run) {
            set_motor_output(1000, 1000, 1000, 1000);
            yaw_initialized = false;
            takeoff_ff_done = false;
        } else {
            // ctrl_apply_attitude uses cx.rc_throttle only for its low-throttle
            // cutoff. ATL drives U1 directly, so force the throttle high (>0.05)
            // to keep motors live; ATL owns the stop decision via motors_run.
            cx.U1 = U1;
            cx.rc_throttle = 1.0f;
            ctrl_apply_attitude(cx, target_roll_rad, target_pitch_rad, target_yaw_rate,
                                yaw_initialized, takeoff_ff_done);
        }

        // ---- DebugFrame (50Hz) ----
        if (!debug_output_enabled) continue;

        DebugFrame df;
        ctrl_fill_debug_common(df, cx, target_roll_rad, target_pitch_rad, target_yaw_rate);

        // Group 3 (velocity group, reused): position PID NED-axis tilt/err.
        df.u_cmd = pid_tilt_n;
        df.u_fb  = vel_n;
        df.u_acc = 0.0f;
        df.u_err = pid_err_n;
        df.v_cmd = pid_tilt_e;
        df.v_fb  = vel_e;
        df.v_acc = 0.0f;
        df.v_err = pid_err_e;

        // Group 4 (PNG group, reused for horizontal position telemetry).
        df.png_px       = pos_px_log;          // current NED north [m]
        df.png_py       = pos_py_log;          // current NED east  [m]
        df.png_rng      = 0.0f;
        df.png_eta      = 0.0f;
        df.png_los      = 0.0f;
        df.png_yaw_rate = target_yaw_rate;     // commanded yaw rate [rad/s]
        df.wp_idx       = 0;
        df.png_lat      = pos_lat_log;
        df.png_lon      = pos_lon_log;
        df.png_alt      = pos_alt_log;

        // Group 6: auto takeoff/landing state machine telemetry.
        df.atl_state     = (uint8_t)atl_seq.state;
        df.atl_climb_cmd = atl_seq.climb_cmd_log;  // commanded climb rate [m/s, up+]
        df.atl_hold_alt  = atl_seq.hold_h;         // captured hold altitude [m, up+]
        df.atl_phase_ms  = atl_seq.phase_ms_log;   // spool-up/down ms remaining
        df.wp_alt_cmd    = atl_seq.hold_h;         // ATL has no WP ramp -> = hold alt

        // Group 7: vision precision-landing.
        // (a) RAW camera input: latest received from the bridge, even if not used
        //     (so the camera link is visible whether or not we are steering).
        {
            bool raw_fresh = helipad_valid_raw &&
                             (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS;
            df.helipad_x     = helipad_x_m;          // raw body forward offset [m]
            df.helipad_y     = helipad_y_m;          // raw body right   offset [m]
            df.helipad_xy    = helipad_xy_m;         // raw horizontal error [m]
            df.helipad_gnd   = helipad_gnd_m;        // raw ground distance [m]
            df.helipad_valid = raw_fresh ? 1 : 0;    // raw valid AND fresh
        }
        // (b) attitude-corrected NED offset (full DCM applied to the raw body
        //     offset, before adding pos) - compare against raw helipad_x/y to see
        //     the roll/pitch correction. Last computed in the accept path above.
        df.helipad_dn    = helipad_dn_log;       // DCM-corrected NED north offset [m]
        df.helipad_de    = helipad_de_log;       // DCM-corrected NED east  offset [m]
        // (c) NED latch actually used by the position PID.
        df.vlatch_n      = vlatch_set ? vlatch_n : 0.0f;  // latched NED north [m]
        df.vlatch_e      = vlatch_set ? vlatch_e : 0.0f;  // latched NED east  [m]
        // (d) controller status.
        df.vision_use    = vision_use ? 1 : 0;       // 1 = steering to latch
        df.vlatch_set    = vlatch_set ? 1 : 0;       // 1 = lock acquired
        df.vision_reject = vision_reject ? 1 : 0;    // 1 = spike rejected this step

        ctrl_send_debug(df, rt_count, rt_dt_max);
    } // end while (current_mode == this_mode)
}

// ============================================================
// MISSION (MODE_MISSION) - auto takeoff + PNG guidance + auto land
// ------------------------------------------------------------
// Combines the ATL vertical sequencer (atl_sequencer_step) with PNG guidance:
//   GROUND -(t)-> SPOOLUP -> TAKEOFF -> HOLD -(m)-> GUIDANCE -> LAND -> ... -> DISARMED
// Vertical is owned by the sequencer (climb-rate / alt-hold). Horizontal:
//   - TAKEOFF/HOLD/LAND/LAND_SLOW (horiz_hold): hold a fixed NED point (takeoff
//     origin during takeoff/hold; the captured last-WP point during landing).
//   - GUIDANCE (guidance_run): run PNG proportional navigation over the uploaded
//     mission; altitude stays alt-hold at the captured target (1.5m). On the
//     final waypoint, signal guidance_done -> sequencer descends (LAND) in place.
// Triggers: GCS CMD_AUTO_TAKEOFF / CMD_MISSION_START / CMD_AUTO_LAND.
// Same safety set as ATL/PNG (EMERGENCY, IMU watchdog, tilt, RC-loss emland).
void loop_mission()
{
    const FlightMode this_mode = MODE_MISSION;

    static float target_yaw_rad  = 0.0f;
    static bool  yaw_initialized = false;
    static bool  takeoff_ff_done = false;

    // Captured horizontal hold point for the LAND phase (set when guidance ends).
    static float land_hold_n = 0.0f, land_hold_e = 0.0f;
    static bool  land_pt_captured = false;

    // GUIDANCE altitude-ramp entry latch: capture png_alt_prev from the sequencer's
    // hold altitude the first step GUIDANCE runs, so the first segment blends from
    // the takeoff-hold altitude toward WP0's altitude.
    static bool  guidance_alt_armed = false;

    // Vision helipad target latch (absolute NED). Local (not static) so it resets to
    // "no lock" on every mode (re-)entry. The final-WP landing follows this latch;
    // a vision dropout HOLDS the last latch instead of snapping back to the captured
    // land point. Mirrors loop_auto_tko_land.
    bool  vlatch_set = false;          // a valid helipad point is latched
    float vlatch_n   = 0.0f;           // latched helipad target, NED north [m]
    float vlatch_e   = 0.0f;           // latched helipad target, NED east  [m]

    uint32_t rc_lost_since_ms = millis();
    bool     rc_was_valid     = true;

    static uint32_t rt_last_us = 0;
    static uint16_t rt_count   = 0;
    static float    rt_dt_max  = 0.0f;

    const float yaw_rate_max = RC_MAX_YAW_RATE_DPS * DEG_TO_RAD;

    while (current_mode == this_mode) {

        ctrl_housekeeping();

        // Autonomous mode -> failsafe emland descent on RC loss (stay armed).
        if (ctrl_rc_loss(rc_lost_since_ms, rc_was_valid, true)) break;

        ins.update();

        if (!time_ready()) continue;
        update_time();

        float dt = (float)SAMPLING_TIME;

        ctrl_diag_tick(rt_last_us, rt_count, rt_dt_max);
        ctrl_periodic_tasks();

        if (ins.available())
            last_imu_data_ms = millis();

        // IMU watchdog
        if ((millis() - last_imu_data_ms) >= IMU_TIMEOUT_MS) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            continue;
        }

        // MISSION: altitude-hold ALWAYS on (vertical owned by the sequencer).
        ControlContext cx;
        cx.dt = dt;
        cx.force_alt_hold = true;
        ctrl_read_sensors(cx);
        ControlSource ctrl = cx.ctrl;

        if (ctrl == CTRL_EMERGENCY) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            emland_active   = false;
            continue;
        }

        MtiData &d = cx.d;

        ctrl_update_altitude_est(cx);
        float lidar_alt_m = cx.lidar_alt_m;
        bool  aid_valid   = cx.aid_valid;
        float yaw_rad     = cx.yaw_rad;

        if (!yaw_initialized) {
            target_yaw_rad = yaw_rad;
            yaw_initialized = true;
        }

        // Tilt protection
        if (cx.current_roll_deg > TILT_LIMIT_DEG || cx.current_roll_deg < -TILT_LIMIT_DEG ||
            cx.current_pitch_deg > TILT_LIMIT_DEG || cx.current_pitch_deg < -TILT_LIMIT_DEG) {
            set_motor_output(1000, 1000, 1000, 1000);
            autopilot.reset();
            yaw_initialized = false;
            takeoff_ff_done = false;
            continue;
        }

        ctrl_rotate_gnss_vel(cx);
        float gnss_vel_u = cx.gnss_vel_u, gnss_vel_v = cx.gnss_vel_v;
        float vel_n = (float)d.data[VEL_X]; // RAW NED north [m/s]
        float vel_e = (float)d.data[VEL_Y]; // RAW NED east  [m/s]

        float alt_now_m = (alt_ready && aid_valid) ? alt_hold_active_h(lidar_alt_m) : 0.0f;
        float v_up      = cx.v_used;

        // ---- Altitude-aiding loss -> automatic emergency landing ----
        // MISSION (like ATL) has no RC fallback for alt-hold. Sustained aid loss
        // while airborne -> latch emland so the sequencer diverts to a LiDAR-
        // independent fixed-thrust descent. GUIDANCE is included (it holds altitude
        // via the sequencer too). Brief dropouts are covered by the spike-hold.
        {
            static uint32_t aid_lost_since_ms = 0;
            const bool msn_airborne_aid = (atl_seq.state == ATL_SPOOLUP  ||
                                           atl_seq.state == ATL_TAKEOFF  ||
                                           atl_seq.state == ATL_HOLD     ||
                                           atl_seq.state == ATL_GUIDANCE ||
                                           atl_seq.state == ATL_LAND     ||
                                           atl_seq.state == ATL_LAND_SLOW);
            if (aid_valid || !msn_airborne_aid) {
                aid_lost_since_ms = 0;
            } else {
                if (aid_lost_since_ms == 0) aid_lost_since_ms = millis();
                if (!emland_active &&
                    (millis() - aid_lost_since_ms) >= (uint32_t)AID_LOST_EMLAND_MS) {
                    emland_active = true;   // sequencer -> LiDAR-independent emland
                    #if USE_DEBUG_SERIAL
                    Serial.println("[MISSION] altitude aid lost -> auto emland");
                    #endif
                }
            }
        }

        float target_roll_rad  = 0.0f;
        float target_pitch_rad = 0.0f;
        float target_yaw_rate  = 0.0f;

        // Telemetry (PNG/vel debug fields).
        float vc_u_cmd = 0.0f, vc_u_fb = 0.0f, vc_v_cmd = 0.0f, vc_v_fb = 0.0f;
        float png_px_log = 0.0f, png_py_log = 0.0f;
        float png_rng_log = 0.0f, png_eta_log = 0.0f, png_yaw_rate_log = 0.0f;
        float png_los_log = 0.0f;
        double png_lat_log = 0.0, png_lon_log = 0.0;
        float  png_alt_log = 0.0f;
        float  pid_tilt_n = 0.0f, pid_tilt_e = 0.0f, pid_err_n = 0.0f, pid_err_e = 0.0f;
        float  wp_alt_cmd_log = atl_seq.hold_h;  // ramped alt setpoint (GUIDANCE), else hold

        // Vision precision-landing telemetry status (raw cam values are read straight
        // from the helipad_* globals when filling the DebugFrame). Mirrors ATL.
        bool  vision_use    = false;   // steering to the latched helipad point
        bool  vision_reject = false;   // this step's fix rejected as a spike
        float helipad_dn_log = 0.0f;   // attitude-corrected NED-north offset [m]
        float helipad_de_log = 0.0f;   // attitude-corrected NED-east  offset [m]

        // Display-only DCM debug: compute the attitude-corrected NED offset on every
        // fresh+valid fix regardless of phase (bench-verifiable, props off). The
        // flight latch path in the LAND branch recomputes the same formula.
        #if USE_VISION_LANDING
        if (helipad_valid_raw &&
            (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS) {
            helipad_body_to_ned(helipad_x_m, helipad_y_m,
                                cx.roll_rad, cx.pitch_rad, cx.yaw_rad,
                                helipad_dn_log, helipad_de_log);
        }
        #endif

        bool pos_fix_ok = png_origin_set &&
                          (d.data_available & HAS_GNSS_PVT) &&
                          (d.gnss_pvt.fix_type >= 3);

        float U1 = 0.0f;
        bool  motors_run = true;

        // emland overrides MANUAL and is fed to the sequencer: while airborne it
        // diverts to ATL_LAND_SLOW (fixed-thrust descent that HOLDS the current
        // position via land_pt_captured below) and self-terminates through the
        // normal touchdown -> SPOOLDOWN -> DISARMED path. So emland runs through
        // the AUTO branch, not a separate level-descent block.
        const bool mission_auto = (ctrl == CTRL_AUTO) || emland_active;

        if (!mission_auto) {
            // MANUAL: hand-fly with RC sticks; sequence paused.
            autopilot.reset_position();
            float target_pitch_deg = -((int)rc_pulse_us[RC_CH_PITCH] + RC_TRIM_PITCH - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            float target_roll_deg  = -((int)rc_pulse_us[RC_CH_ROLL]  + RC_TRIM_ROLL  - 1500) * (RC_MAX_ANGLE_DEG / 500.0f);
            target_pitch_rad = target_pitch_deg * DEG_TO_RAD;
            target_roll_rad  = target_roll_deg  * DEG_TO_RAD;
            target_yaw_rate  = -((int)rc_pulse_us[RC_CH_YAW] + RC_TRIM_YAW - 1500) * (RC_MAX_YAW_RATE_DPS / 400.0f) * DEG_TO_RAD;
            ctrl_compute_thrust(cx);
            U1 = cx.U1;
        } else {
            // ---- AUTO (incl. emland): takeoff -> guidance -> landing sequence ----
            bool req_tko  = atl_takeoff_request; atl_takeoff_request = false;
            bool req_msn  = atl_mission_request; atl_mission_request = false;
            bool req_land = atl_land_request;    atl_land_request    = false;

            // Guidance reaches the final waypoint -> signal the sequencer to land.
            bool guidance_done = (atl_seq.state == ATL_GUIDANCE) && png_is_end;

            SeqOut so = atl_sequencer_step(cx, atl_seq, alt_now_m, v_up,
                                           pos_fix_ok, aid_valid,
                                           req_tko, req_msn, req_land, guidance_done,
                                           emland_active);
            U1         = so.U1;
            motors_run = so.motors_run;

            // Re-arm the GUIDANCE altitude-ramp seed whenever guidance isn't running
            // (ground/hold/land), so a fresh GUIDANCE entry re-captures png_alt_prev.
            if (!so.guidance_run) {
                guidance_alt_armed = false;
#if USE_OBS_AVOID
                cc.reset(); g_ccd = CCDebug{};   // not guiding -> clear avoidance latch
#endif
            }

            if (so.guidance_run) {
                // ---- GUIDANCE: PNG proportional navigation (horizontal) ----
                land_pt_captured = false;   // re-arm landing-point capture

                // Altitude ramp: on the first GUIDANCE step, seed the ramp start
                // from the sequencer's captured hold altitude (takeoff target), so
                // the first segment blends from there toward WP0's altitude.
                if (!guidance_alt_armed) {
                    png_alt_prev = atl_seq.hold_h;
                    guidance_alt_armed = true;
                }
                if (pos_fix_ok) {
                    float accN = (float)d.data[FREE_ACC_X], accE = (float)d.data[FREE_ACC_Y];
                    float u_acc =  accN * cx.cpsi_b + accE * cx.spsi_b;  // body fwd
                    float v_acc = -accN * cx.spsi_b + accE * cx.cpsi_b;  // body right

                    png_lat_log = d.gnss_pvt.lat / 1e7;
                    png_lon_log = d.gnss_pvt.lon / 1e7;
                    png_alt_log = (float)(d.gnss_pvt.height / 1000.0);
                    PngGuidance g = png_compute_guidance(png_lat_log, png_lon_log,
                                                         d.gnss_pvt.height / 1000.0,
                                                         yaw_rad, alt_now_m, gnss_vel_u);
                    png_px_log = g.px; png_py_log = g.py;
                    png_rng_log = g.rng; png_eta_log = g.eta;
                    png_yaw_rate_log = g.yaw_rate; png_los_log = g.los;

                    // Drive the vertical alt-hold setpoint with the segment ramp.
                    // The sequencer's update_altitude() reads atl_seq.hold_h, so the
                    // ramped target applies on the next step (one 5ms tick of lag,
                    // negligible vs the climb-rate limiter). g.alt_cmd is already
                    // clamped/floored against seg_len in png_compute_guidance.
                    atl_seq.hold_h = g.alt_cmd;
                    wp_alt_cmd_log = g.alt_cmd;

                    // Forward velocity -> pitch; right velocity 0 -> station-keep roll.
                    float vc_pitch_cmd, vc_roll_cmd;
                    autopilot.update_velocity(g.u_cmd, gnss_vel_u, u_acc,
                                              0.0f, gnss_vel_v, v_acc,
                                              cx.dt, vc_pitch_cmd, vc_roll_cmd);
                    target_pitch_rad = -vc_pitch_cmd;
                    target_roll_rad  = vc_roll_cmd;
                    // ---- Per-segment guidance gating (one mode per WP) ----
                    // Each WP carries exactly one behaviour flag: WP_FLAG_AVOID
                    // (0x01, collision-cone avoidance on the segment AFTER that WP)
                    // or WP_FLAG_ORBIT (0x02, orbit the next-WP midpoint). They are
                    // never combined. Avoidance is armed only when the WP we just
                    // PASSED carries WP_FLAG_AVOID; the first leg (origin -> WP0) has
                    // png_wp_prev_idx < 0 -> off; legacy missions (flags == 0) stay
                    // off. While orbiting (g.orbit_active) avoidance is FORCED off so
                    // the orbit yaw-rate is never overridden. Substitutes r_ca for
                    // g.yaw_rate while the latch is on; PNG_VX forward speed unchanged.
                    float r_wp = g.yaw_rate;
#if USE_OBS_AVOID
                    bool seg_avoid = !g.orbit_active &&
                                     (png_wp_prev_idx >= 0) &&
                                     (png_wp_flags[png_wp_prev_idx] & WP_FLAG_AVOID);
                    target_yaw_rate = ctrl_apply_avoidance(cx, seg_avoid, r_wp);
#else
                    target_yaw_rate = r_wp;
#endif
                    vc_u_cmd = g.u_cmd; vc_u_fb = gnss_vel_u;
                    vc_v_cmd = 0.0f;    vc_v_fb = gnss_vel_v;
                } else {
                    // Fix lost mid-guidance: level hold, zero command.
                    autopilot.reset_velocity();
                    target_pitch_rad = 0.0f; target_roll_rad = 0.0f; target_yaw_rate = 0.0f;
                }
            } else if (so.horiz_hold && pos_fix_ok &&
                       (alt_now_m >= ATL_POS_ENABLE_ALT)) {
                // ---- Position hold (TAKEOFF/HOLD origin, LAND last-WP point) ----
                png_lat_log = d.gnss_pvt.lat / 1e7;
                png_lon_log = d.gnss_pvt.lon / 1e7;
                png_alt_log = (float)(d.gnss_pvt.height / 1000.0);
                float pos_n, pos_e;
                png_lla_to_ned(png_lat_log, png_lon_log,
                               d.gnss_pvt.height / 1000.0, pos_n, pos_e);

                // Landing: hold the position captured when guidance ended (land in
                // place). Takeoff/hold: hold the takeoff origin (0,0).
                float sp_n = 0.0f, sp_e = 0.0f;
                const bool land_phase = (atl_seq.state == ATL_LAND ||
                                         atl_seq.state == ATL_LAND_SLOW);
                if (land_phase) {
                    if (!land_pt_captured) {
                        land_hold_n = pos_n; land_hold_e = pos_e;
                        // Re-anchor the yaw setpoint to the heading the craft ended
                        // guidance at, so it lands in place without rotating back to
                        // the (now-stale) takeoff heading.
                        target_yaw_rad = yaw_rad;
                        land_pt_captured = true;
                    }
                    // Default = the captured last-WP point (no vision / no lock yet).
                    sp_n = land_hold_n; sp_e = land_hold_e;

                    // ---- Vision precision-landing (RealSense helipad) ----
                    // Steer to the latched helipad point once vision locks. A fresh,
                    // valid, non-spike, low-tilt fix UPDATES the absolute-NED latch;
                    // invalid/stale/spike HOLDS the last latch (no snap-back). emland
                    // descends in place and never chases the helipad. Mirrors ATL.
                    #if USE_VISION_LANDING
                    if (!emland_active) {
                        if (helipad_valid_raw &&
                            (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS) {
                            float dn, de;
                            helipad_body_to_ned(helipad_x_m, helipad_y_m,
                                                cx.roll_rad, cx.pitch_rad, yaw_rad,
                                                dn, de);
                            helipad_dn_log = dn;   // attitude-corrected NED offset
                            helipad_de_log = de;
                            const float cand_n = pos_n + dn;
                            const float cand_e = pos_e + de;
                            // Tilt freeze: the level assumption breaks down past
                            // VISION_TILT_FREEZE_DEG, so don't accept new fixes then.
                            const float tilt_lim = VISION_TILT_FREEZE_DEG * DEG_TO_RAD;
                            const bool tilt_ok =
                                (cx.roll_rad  <  tilt_lim && cx.roll_rad  > -tilt_lim &&
                                 cx.pitch_rad <  tilt_lim && cx.pitch_rad > -tilt_lim);
                            // Spike reject: ignore fixes that jump > VISION_MAX_JUMP_M
                            // from the current latch (first lock is always accepted).
                            bool accept = tilt_ok;
                            if (accept && vlatch_set) {
                                const float jn = cand_n - vlatch_n;
                                const float je = cand_e - vlatch_e;
                                accept = (jn * jn + je * je) <=
                                         (VISION_MAX_JUMP_M * VISION_MAX_JUMP_M);
                            }
                            if (accept) {
                                vlatch_n = cand_n; vlatch_e = cand_e; vlatch_set = true;
                            } else {
                                vision_reject = true;
                            }
                        }
                        if (vlatch_set) {
                            sp_n = vlatch_n; sp_e = vlatch_e;
                            vision_use = true;
                        }
                    }
                    #endif // USE_VISION_LANDING
                }

                autopilot.update_position(sp_n, sp_e, pos_n, pos_e,
                                          vel_n, vel_e, yaw_rad, dt,
                                          target_pitch_rad, target_roll_rad);

                float yaw_err   = wrap_pi(target_yaw_rad - yaw_rad);
                target_yaw_rate = POS_YAW_HOLD_KP * yaw_err;
                if (target_yaw_rate >  yaw_rate_max) target_yaw_rate =  yaw_rate_max;
                if (target_yaw_rate < -yaw_rate_max) target_yaw_rate = -yaw_rate_max;

                png_px_log = pos_n; png_py_log = pos_e;
                pid_tilt_n = autopilot.pos_n_tilt();
                pid_tilt_e = autopilot.pos_e_tilt();
                pid_err_n  = autopilot.pos_n_err();
                pid_err_e  = autopilot.pos_e_err();
            } else {
                // Ground / below enable alt / fix lost: level hold.
                target_pitch_rad = 0.0f; target_roll_rad = 0.0f; target_yaw_rate = 0.0f;
                autopilot.reset_position();
            }
        }

        // ---- Output ----
        if (!motors_run) {
            set_motor_output(1000, 1000, 1000, 1000);
            yaw_initialized = false;
            takeoff_ff_done = false;
        } else {
            cx.U1 = U1;
            cx.rc_throttle = 1.0f;   // keep motors live; sequencer owns stop
            ctrl_apply_attitude(cx, target_roll_rad, target_pitch_rad, target_yaw_rate,
                                yaw_initialized, takeoff_ff_done);
        }

        // ---- DebugFrame (50Hz) ----
        if (!debug_output_enabled) continue;

        DebugFrame df;
        ctrl_fill_debug_common(df, cx, target_roll_rad, target_pitch_rad, target_yaw_rate);

        // Group 3 (velocity group): guidance velocity / position PID tilt-err.
        df.u_cmd = (atl_seq.state == ATL_GUIDANCE) ? vc_u_cmd : pid_tilt_n;
        df.u_fb  = (atl_seq.state == ATL_GUIDANCE) ? vc_u_fb  : vel_n;
        df.u_acc = 0.0f;
        df.u_err = pid_err_n;
        df.v_cmd = (atl_seq.state == ATL_GUIDANCE) ? vc_v_cmd : pid_tilt_e;
        df.v_fb  = (atl_seq.state == ATL_GUIDANCE) ? vc_v_fb  : vel_e;
        df.v_acc = 0.0f;
        df.v_err = pid_err_e;

        // Group 4 (PNG group): position / guidance telemetry.
        df.png_px       = png_px_log;
        df.png_py       = png_py_log;
        df.png_rng      = png_rng_log;
        df.png_eta      = png_eta_log;
        df.png_los      = png_los_log;
        df.png_yaw_rate = (atl_seq.state == ATL_GUIDANCE) ? png_yaw_rate_log : target_yaw_rate;
        df.wp_idx       = (uint8_t)png_wp_idx;
        df.png_lat      = png_lat_log;
        df.png_lon      = png_lon_log;
        df.png_alt      = png_alt_log;

        // Group 6: sequencer telemetry.
        df.atl_state     = (uint8_t)atl_seq.state;
        df.atl_climb_cmd = atl_seq.climb_cmd_log;
        df.atl_hold_alt  = atl_seq.hold_h;
        df.atl_phase_ms  = atl_seq.phase_ms_log;
        df.wp_alt_cmd    = wp_alt_cmd_log;         // ramped WP altitude setpoint [m]

        // Group 9: uploaded mission converted to local NED at mode entry. Static
        // for the whole flight (computed once in enter_mode), so the same values
        // repeat every frame; only the first wp_ned_count entries are valid.
        {
            int n = (png_wp_size < 0) ? 0
                  : (png_wp_size > PNG_MAX_WAYPOINTS ? PNG_MAX_WAYPOINTS
                                                     : png_wp_size);
            df.wp_ned_count = (uint8_t)n;
            for (int i = 0; i < n; i++) {
                df.wp_north[i] = png_wp_north[i];
                df.wp_east[i]  = png_wp_east[i];
            }
            // entries [n..PNG_MAX_WAYPOINTS) stay 0 from the frame-start memset.
        }

        // Group 7: vision precision-landing (mirrors loop_auto_tko_land).
        {
            bool raw_fresh = helipad_valid_raw &&
                             (millis() - helipad_ms) < (uint32_t)VISION_STALE_MS;
            df.helipad_x     = helipad_x_m;          // raw body forward offset [m]
            df.helipad_y     = helipad_y_m;          // raw body right   offset [m]
            df.helipad_xy    = helipad_xy_m;         // raw horizontal error [m]
            df.helipad_gnd   = helipad_gnd_m;        // raw ground distance [m]
            df.helipad_valid = raw_fresh ? 1 : 0;    // raw valid AND fresh
        }
        
        df.helipad_dn    = helipad_dn_log;       // DCM-corrected NED north offset [m]
        df.helipad_de    = helipad_de_log;       // DCM-corrected NED east  offset [m]
        df.vlatch_n      = vlatch_set ? vlatch_n : 0.0f;  // latched NED north [m]
        df.vlatch_e      = vlatch_set ? vlatch_e : 0.0f;  // latched NED east  [m]
        df.vision_use    = vision_use ? 1 : 0;       // 1 = steering to latch
        df.vlatch_set    = vlatch_set ? 1 : 0;       // 1 = lock acquired
        df.vision_reject = vision_reject ? 1 : 0;    // 1 = spike rejected this step

        ctrl_send_debug(df, rt_count, rt_dt_max);
    } // end while (current_mode == this_mode)
}

void loop_rtk_main()
{
    if (!time_ready()) return;

    ins.update();

    #if USE_DEBUG_SERIAL
    static uint32_t last_print_ms = 0;
    uint32_t now = millis();
    if (now - last_print_ms >= 200) {
        last_print_ms = now;
        MtiData d = ins.getData();
        Serial.print("T="); Serial.print(get_elapsed_seconds(), 2);
        if (d.data_available & HAS_LATLON) {
            Serial.print(" Lat="); Serial.print(d.data[LATITUDE], 9);
            Serial.print(" Lon="); Serial.print(d.data[LONGITUDE], 9);
        }
        if (d.data_available & HAS_ALT_MSL) {
            Serial.print(" Alt="); Serial.print(d.data[ALT_MSL], 2);
        }
        if (d.data_available & HAS_STATUS_WORD) {
            Serial.print(" Fix="); Serial.print((int)d.data[GNSS_FIX]);
            Serial.print(" SV=");  Serial.print((int)d.data[GNSS_NUM_SV]);
            int rtk = (int)d.data[RTK_STATUS];
            Serial.print(" RTK=");
            if (rtk == 2)      Serial.print("Fixed");
            else if (rtk == 1) Serial.print("Float");
            else               Serial.print("None");
        }
        Serial.println();
    }
    #endif

    update_time();
}

void loop_pwm_test()
{
    // PWM commands handled via P8 uplink and USB serial
}

void loop_esc_cal()
{
    // ESC calibration: RC throttle drives all 4 ESCs to the rail.
    //   throttle >= 1500us -> 2000us (full)
    //   throttle <  1500us -> 1000us (min)
    // EMERGENCY (CH5) and loss of RC force min throttle for safety.

    bool rc_valid = (rc_pulse_us[RC_CH_THROTTLE] >= 900 &&
                     rc_pulse_us[RC_CH_THROTTLE] <= 2200);

    bool emergency = (rc_pulse_us[RC_CH_EMERGENCY] >= RC_EMERGENCY_THRESHOLD);

    uint16_t out = 1000;
    if (rc_valid && !emergency && rc_pulse_us[RC_CH_THROTTLE] >= 1500)
        out = 2000;

    set_motor_output(out, out, out, out);
}

void loop_mti_test()
{
    if (!time_ready()) return;

    ins.update();

    #if USE_DEBUG_SERIAL
    static uint32_t last_hz_time = 0;
    static uint32_t loop_count = 0;
    static uint32_t last_hz = 0;

    loop_count++;
    uint32_t now_ms = millis();
    if (now_ms - last_hz_time >= 1000) {
        last_hz = loop_count;
        loop_count = 0;
        last_hz_time = now_ms;
    }

    if (ins.available() && TIME.idx % 10 == 0) {
        MtiData d = ins.getData();
        Serial.print(get_elapsed_seconds(), 3); Serial.print("\t");
        Serial.print(d.data[ROLL], 2);  Serial.print("\t");
        Serial.print(d.data[PITCH], 2); Serial.print("\t");
        Serial.print(d.data[YAW], 2);   Serial.print("\t");
        Serial.print(d.data[ACC_Z], 3); Serial.print("\t");
        Serial.println(last_hz);
    }
    #endif

    update_time();
}
