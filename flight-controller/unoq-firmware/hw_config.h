//
// AVC UNO Q Hardware Configuration
// Arduino UNO Q (STM32U585 Cortex-M33 160MHz)
//
// All hardware pin / UART / PWM / tuning definitions are collected here.
// Macro names and values must stay in sync with the firmware; edit here when
// changing wiring, peripheral mapping, or tuning.
//

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <Arduino.h>

// ============================================================
// 1. System Mode
// ============================================================

// RTK mode: 1 = RTK correction GPS, 0 = GNSS only
#define USE_RTK    1
// RouterBridge RPC (get_status MPU-debug handler) always available
#define USE_BRIDGE 1

// ============================================================
// 2. Control / Telemetry Loop
// ============================================================
// Control-loop rate is the SINGLE SOURCE OF TRUTH: realtime tick, integration
// dt, FMF gains, and rate-LPF alpha are all derived from CONTROL_FREQ_HZ.
// Defined early because controller tuning below (e.g. VEL_ACC_LPF) uses it.
#define CONTROL_FREQ_HZ  200
#define CONTROL_DT_S     (1.0f / (float)CONTROL_FREQ_HZ)   // control period [s]
#define TELEM_FREQ_HZ    20                                // downlink rate [Hz] (DebugFrame + TelemFrame)

// ============================================================
// 3. UART Configuration
// ============================================================

// Debug output (USB Serial via Bridge / Serial Monitor)
#ifndef USE_DEBUG_SERIAL
#define USE_DEBUG_SERIAL 1
#endif
#define DEBUG_BAUD       115200

// MTi-680G IMU (USART1 = D1/PB6 TX, D0/PB7 RX, AF7; needs MAX3232 RS-232 shift)
#include "Usart1Serial.h"
extern Usart1Serial Usart1Ser;
#define IMU_SERIAL          Usart1Ser
#define IMU_BAUD            230400
#define IMU_UART_SWAP_RXTX  0    // 1 = swap TX/RX in HW (USART1 CR2.SWAP)

// P8 Telemetry UART (USART3 = PA7 TX / PA5 RX, AF7)
#include "Usart3Serial.h"
extern Usart3Serial P8Serial;
#define TELEM_BAUD       460800

// ============================================================
// 4. Motor PWM (X-configuration Quadrotor)
// ============================================================
// 4-channel ESC output via Servo library. JDIGITAL pin -> MCU pin -> Timer:
//   CH0 = D3 (PB0,  TIM3_CH3) Front-Left
//   CH1 = D5 (PA11, TIM1_CH4) Rear-Left
//   CH2 = D6 (PB1,  TIM3_CH4) Rear-Right
//   CH3 = D9 (PB8,  TIM4_CH3) Front-Right
#define MOTOR_PIN_0      3
#define MOTOR_PIN_1      5
#define MOTOR_PIN_2      6
#define MOTOR_PIN_3      9
#define SERVO_PIN        11   // D11 (PA15, TIM2_CH1)

// ============================================================
// 5. LED (UNO Q on-board RGB)
// ============================================================
#ifndef LEDB
#define LEDB LED3_B
#define LEDG LED3_G
#define LEDR LED4_R
#endif

// ============================================================
// 6. RC Receiver (PWM input via JDIGITAL GPIO, attachInterrupt EXTI)
// ============================================================
//   D4  = PA12 (EXTI12) CH1 Throttle
//   D2  = PB3  (EXTI3)  CH2 Pitch (Elevator)
//   D7  = PB2  (EXTI2)  CH3 Roll (Aileron)
//   D10 = PB9  (EXTI9)  CH4 MODE (LOW=MANUAL, HIGH=AUTO)
//   D8  = PB4  (EXTI4)  CH5 EMERGENCY (>=1500us = STOP)
//   D12 = PB14          CH6 Yaw-rate command
#define RC_NUM_CHANNELS  6
#define RC_CH1_PIN       4
#define RC_CH2_PIN       2
#define RC_CH3_PIN       7
#define RC_CH4_PIN       10
#define RC_CH5_PIN       8
#define RC_CH6_PIN       12

// RC channel indices
#define RC_CH_THROTTLE   0
#define RC_CH_PITCH      1
#define RC_CH_ROLL       2
#define RC_CH_MODE       3
#define RC_CH_EMERGENCY  4
#define RC_CH_YAW        5

// Switch thresholds [us]
#define RC_EMERGENCY_THRESHOLD  1500
#define RC_MODE_THRESHOLD       1500

// Trim offset added to raw us so stick-center reads 1500us
#define RC_TRIM_THROTTLE  0
#define RC_TRIM_PITCH     0
#define RC_TRIM_ROLL      -98   // raw center 1598 -> 1500
#define RC_TRIM_YAW       101   // raw center 1399 -> 1500

// Stick command limits
#define RC_MAX_ANGLE_DEG     20.0f    // max roll/pitch command [deg]
#define RC_MAX_YAW_RATE_DPS  90.0f    // max yaw-rate command [deg/s]

// RC signal-loss timeout [ms]
#define RC_LOST_TIMEOUT_MS   500

// ============================================================
// 7. Flight Mode Tuning
// ============================================================

// ---- Velocity controller (MODE_VEL_CONTROL) ----
// theta_cmd = kp*(u_cmd-u_out) + ki*int - kd*a_out (X=fwd/pitch, Y=right/roll).
// PID gains live with velU/velV in Autopilot.cpp; only shared limits are here.
#define VEL_MAX_TILT_DEG  20.0f      // theta_cmd saturation [deg]
#define VEL_MAX_CMD_MS    2.0f       // full-stick velocity command [m/s]
#define VEL_ENABLE_ALT_M  1.0f       // min altitude to run velocity ctrl [m, up+]
// 1st-order LPF on the noisy accel feedback before the -kd*a damping term.
#define VEL_ACC_LPF_FC_HZ  12.0f     // cutoff [Hz]; <=0 disables
#define VEL_ACC_LPF_TAU    (1.0f / (2.0f * 3.14159265358979f * VEL_ACC_LPF_FC_HZ))
#define VEL_ACC_LPF_ALPHA  (CONTROL_DT_S / (VEL_ACC_LPF_TAU + CONTROL_DT_S))

// ---- PNG guidance (MODE_PNG_GUIDANCE) ----
// Proportional Navigation over RTK-GNSS local NED waypoints.
// yaw_rate = PNG_KP * Vx * sin(eta) / range; forward speed fixed at PNG_VX_MS.
#define PNG_KP            3.0f        // navigation gain
#define PNG_VX_MS         1.5f        // fixed forward speed command [m/s]
#define PNG_WP_CAPTURE_R  1.5f        // waypoint capture radius [m]
// PNG yaw-rate denominator floor. MUST be >= PNG_WP_CAPTURE_R: inside the
// capture radius the WP is already captured and advanced, so flooring rng at
// the capture radius stops sin(eta)/rng from blowing up when the craft passes
// just outside the WP (eta -> +/-180 deg while rng stays small). Without this
// floor the yaw-rate saturates at RC_MAX_YAW_RATE and the craft orbits the WP
// instead of capturing it.
#define PNG_RNG_MIN       2.0f        // min range in PNG denom (>= capture R) [m]

// ---- Position control (MODE_POS_CONTROL) ----
// Step-setpoint controller over RTK-GNSS local NED waypoints (target = active
// waypoint directly). posN/posE PID (gains in Autopilot.cpp) outputs tilt,
// capped by POS_MAX_TILT_DEG. WP reached when actual distance < POS_ARRIVE_R.
// Origin / waypoint NED cache shared with PNG.
#define POS_MAX_TILT_DEG  15.0f       // roll/pitch command limit [deg]
#define POS_ARRIVE_R      0.3f        // waypoint arrival radius [m]
#define POS_I_ENABLE_R    1.0f        // integral runs only within this radius [m]
// Yaw heading-hold: P term on heading error -> yaw-rate cmd for the inner loop,
// holding the captured takeoff heading so the path does not drift. Sized vs the
// 1.5 Hz rate loop (sim): ~8x separation, ~0% overshoot, ~36 deg/s peak cmd.
#define POS_YAW_HOLD_KP   1.2f        // heading-error -> yaw-rate gain [1/s]
// #define POS_YAW_HOLD_KP   0.0f        // heading-error -> yaw-rate gain [1/s]

// ---- Auto takeoff / landing (MODE_AUTO_TKO_LAND) ----
// State machine reuses update_climb_rate for climb/descent, posN/posE for the
// takeoff-point hold, and POS_YAW_HOLD_KP for heading. Tune after ground test.
#define TKO_CLIMB_RATE      0.5f      // takeoff climb rate [m/s, up+]
#define TKO_TARGET_ALT      2.0f      // takeoff target altitude [m]
#define TKO_START_ALT_MAX   0.3f      // max alt allowed to start takeoff [m]
#define ATL_POS_ENABLE_ALT  0.05f      // min alt to run position hold [m, up+]
#define LAND_SPEED_HIGH     0.4f      // descent rate above LAND_SLOW_ALT [m/s]
#define LAND_SPEED_SLOW     0.4f      // descent rate below LAND_SLOW_ALT [m/s]
#define LAND_SLOW_ALT       0.3f      // switch to slow descent below this alt [m]
#define TD_ALT              0.2f     // touchdown altitude threshold [m]
#define TD_V                0.35f     // touchdown vertical-speed threshold [m/s]
#define TD_TIME_MS          1000      // touchdown must persist this long [ms]
// Spool-up: hold sub-hover thrust so props ramp smoothly before climb.
#define ATL_SPOOLUP_FRAC    0.70f     // spool-up thrust = frac * m*g
#define ATL_SPOOLUP_MS      3000      // spool-up duration before climb [ms]
// Spool-down: hold low thrust after touchdown so the craft settles, then stop.
#define ATL_SPOOLDOWN_FRAC  0.35f     // spool-down thrust = frac * m*g
#define ATL_SPOOLDOWN_MS    3000      // spool-down duration after touchdown [ms]

// ---- Vision precision landing (RealSense helipad target, ATL LAND/LAND_SLOW) ----
// During the final descent (ATL_LAND / ATL_LAND_SLOW) the horizontal hold target
// is the RealSense helipad centre. The helipad offset (body FRD x=forward,
// y=right) is rotated by yaw into an ABSOLUTE NED point and LATCHED:
//   - When a fresh, valid (and not a spike) vision fix arrives, the latched NED
//     target is UPDATED to the new helipad position.
//   - When vision is invalid / stale / a spike, the LAST latched NED target is
//     HELD (the craft keeps holding the last good helipad point via GNSS), so a
//     dropout never yanks the craft back to the takeoff origin.
//   - Before the FIRST valid fix, there is nothing to hold, so the controller
//     falls back to the GNSS takeoff-point (NED origin 0,0) as before.
// The latched target is fed to the SAME position PID (update_position). Vertical
// descent + touchdown stay on the existing LiDAR/speed path - vision only steers
// the horizontal alignment. emland disables vision (descend in place).
#define USE_VISION_LANDING     1      // 1 = enable helipad vision steering in ATL
#define VISION_STALE_MS        400    // ignore vision fix older than this [ms]
// Spike reject: if a new fix moves the helipad's NED point more than this from
// the current latched target, drop that frame (hold the previous latch). Guards
// against body-frame drift / attitude glitches producing a bad single sample.
#define VISION_MAX_JUMP_M      1.0f   // max NED move per accepted fix [m]
// Tilt freeze: the vision offset is a LEVEL-body measurement (no live attitude
// from the tracker), so the MCU rotates it to NED with the full roll/pitch/yaw
// DCM. Beyond this tilt the level assumption breaks down (and depth/geometry
// gets noisy), so reject new fixes and hold the last latch while this tilted.
#define VISION_TILT_FREEZE_DEG 15.0f  // skip vision update if |roll|/|pitch| > this [deg]

// ---- Emergency landing (GCS CMD_EMERGENCY_LAND, PNG mode) ----
// Latches U1 to a fixed fraction of hover so the craft descends gently while the
// velocity controllers hold it level. Up 0.85-0.9 if too fast, 0.7 if too slow.
#define EMLAND_THRUST_FRAC  0.9f      // U1 = frac * m*g during emergency landing

// Emergency-landing touchdown detection (ATL/MISSION). Open-loop emland descends
// at a FIXED thrust and must NOT depend on the LiDAR/altitude aiding source (it
// may be the very thing that failed). Detection therefore uses ONLY the MTi
// GNSS/INS vertical velocity (vel_down_latest = VEL_Z, NED down+), which is
// independent of LiDAR: on the ground the craft cannot keep sinking, so the
// vertical speed collapses to ~0. No altitude threshold is used.
#define EMLAND_TD_V         0.20f     // |vertical speed| below this = settled [m/s]
#define EMLAND_TD_TIME_MS   1200      // must persist this long -> touchdown [ms]

// ---- AUTO throttle-stick -> altitude command ----
//   0..SPLIT : U1 ramps 0 -> SPLIT_THRUST_FRAC*mg (gentle takeoff/landing)
//   SPLIT..1 : linear map to target altitude (alt-hold controller)
#define ALT_CMD_THR_SPLIT         0.20f  // ramp -> altitude switch point
#define ALT_CMD_SPLIT_THRUST_FRAC 0.8f   // U1 = frac*mg at SPLIT
#define ALT_CMD_MAX_M             2.0f   // target altitude at full throttle [m, up+]

// ---- Takeoff tilt feed-forward ----
// Added to roll/pitch setpoint near ground, faded to 0 over [FADE_H_LO, H_HI].
#define TAKEOFF_TRIM_ROLL_DEG   -2.5f   // + leans right
#define TAKEOFF_TRIM_PITCH_DEG   4.0f   // + leans forward
#define TAKEOFF_FADE_H_LO        0.10f  // full comp at/below this alt [m]
#define TAKEOFF_FADE_H_HI        1.00f  // zero comp at/above this alt [m]

// ---- Safety ----
#define TILT_LIMIT_DEG    40.0f         // auto motor-stop above this angle [deg]

// ============================================================
// 8. Battery Voltage Sensing (voltage divider module, 0-25V)
//    Register-level ADC1 to avoid Zephyr driver / USART3 ISR conflict
// ============================================================
#define BATT_MAIN_PIN    A2     // PA6, sub power (2S LiPo)
#define BATT_SUB_PIN     A0     // PA4, main power (4S LiPo)
#define ADC_MAX          1023   // 10-bit (0-1023)
#define ADC_VREF_MV      3300   // VREF+ [mV]
#define BATT_DIVIDER_RATIO  5.0f   // 25V module: R1=30k, R2=7.5k -> 5.0
#define BATT_SAMPLE_HZ   2      // battery read rate [Hz]

// ============================================================
// 9. Vehicle Physical Parameters
// ============================================================
#define MASS_KG         2.300f      // total mass incl. battery [kg]
#define LX_M            0.171722f   // arm length X (geometric center) [m]
#define LY_M            0.171722f   // arm length Y (geometric center) [m]

// CG offset from geometric center (top-view: x=forward, y=right) [m]
#define CG_OFFSET_X    0.05f        // + = CG forward
#define CG_OFFSET_Y    0.05f        // + = CG right

// Per-motor arm lengths (CG-referenced)
#define LX_FR  (LX_M - CG_OFFSET_X)   // M1 Front-Right
#define LY_FR  (LY_M - CG_OFFSET_Y)
#define LX_RR  (LX_M + CG_OFFSET_X)   // M2 Rear-Right
#define LY_RR  (LY_M - CG_OFFSET_Y)
#define LX_RL  (LX_M + CG_OFFSET_X)   // M3 Rear-Left
#define LY_RL  (LY_M + CG_OFFSET_Y)
#define LX_FL  (LX_M - CG_OFFSET_X)   // M4 Front-Left
#define LY_FL  (LY_M + CG_OFFSET_Y)

#define CT              1.104e-5f       // thrust coefficient [N/(rad/s)^2]
#define KD_DRAG         2.0e-7f         // drag coefficient [Nm/(rad/s)^2]
#define GAMMA           (KD_DRAG / CT)  // yaw coupling ratio

// omega-duty linear model: omega = OMEGA_SLOPE * duty + OMEGA_OFFSET
#define OMEGA_SLOPE     10.4713f    // [rad/s per %]
#define OMEGA_OFFSET    107.2435f   // [rad/s] at duty=0%
#define DUTY_MAX        80.0f       // max duty safety limit [%]

#define DEG_TO_RAD      0.01745329252f
#define RAD_TO_DEG      57.29577951f

// ============================================================
// 10. Altitude Estimator
// ============================================================
#define ALT_CF_TAU        3.0f   // complementary filter time constant [s]
#define ALT_BARO_CAL_SEC  3.0f   // P0 baro calibration duration [s]

// CF aiding source (compile-time): BARO = MTi pressure->ISA (outdoor),
// LIDAR = TF-Nova downward (QWIIC = Wire1).
#define ALT_AID_BARO       0
#define ALT_AID_LIDAR      1
#define ALT_AIDING_SOURCE  ALT_AID_LIDAR   // <-- select indoor/outdoor here

// LiDAR usage (ALT_AID_LIDAR only): 0 = feed CF (alt_filter.h/v),
// 1 = raw LiDAR direct (v=0).
#define ALT_LIDAR_DIRECT   1

// Fading Memory Filter on LiDAR altitude: estimates height + vertical speed.
//   USE_LIDAR_FMF 1 : feed h_fmf/v_fmf (up+) to alt controller
//   FMF_MODE        : CONST_VEL (control) or CONST_ACC (a_fmf logged only)
//   FMF_BETA        : memory in (0,1); 0.85 faster/noisier, 0.95 slower/smoother
#include "filter.h"   // FmfMode enum for FMF_MODE
#define USE_LIDAR_FMF   1
#define FMF_MODE        FMF_CONST_VEL
#define FMF_BETA        0.85f

// ============================================================
// 11. LiDAR Altimeter (TF-Nova, I2C via QWIIC = Wire1)
// ============================================================
// TF-Nova I2C mode (addr 0x10) on QWIIC (I2C4): SDA=PD13, SCL=PD12, 3.3V.
// Mounted down: vertical altitude = range * cos(roll) * cos(pitch).
#include <Wire.h>
#define LIDAR_WIRE           Wire1    // QWIIC = I2C4
#define LIDAR_I2C_ADDR       0x10     // TF-Nova 7-bit I2C address
#define LIDAR_REG_DIST       0x00     // distance low byte (cm, little-endian)
#define LIDAR_ALT_MIN_M      0.05f    // min valid range [m]
#define LIDAR_ALT_MAX_M      12.0f    // max valid range [m] (TF-Nova max ~14m)
#define LIDAR_ALT_TIMEOUT_MS 200      // stale if no fresh valid sample in window
#define LIDAR_READ_DIV       4        // read every 4th 200Hz tick (~50Hz)
// "No valid measurement" sentinels returned by the TF-Nova (gated out):
#define LIDAR_INVALID_6554   6554     // 0x199A
#define LIDAR_INVALID_FFFF   0xFFFF
// Spike rejection on raw range: if per-tick change > LIDAR_SPIKE_M, hold the
// previous value up to LIDAR_SPIKE_MAX ticks, then accept (never latches).
// More conservative hold (longer): ride through brief LiDAR glitches/dropouts
// without letting a single spike corrupt the estimate. At ~50Hz, 15 ticks ~300ms.
#define LIDAR_SPIKE_M        0.5f     // max accepted per-tick change [m]
#define LIDAR_SPIKE_MAX      15       // max consecutive rejects (~300ms @50Hz)

// ---- ATL/MISSION altitude-aiding loss -> automatic emergency landing ----
// ATL and MISSION are fully autonomous (no RC-throttle fallback for alt-hold,
// unlike RC/VEL/POS). If the altitude aiding source (LiDAR) stays invalid for
// longer than the spike-hold can cover, the alt-hold controller would otherwise
// see alt_now=0 and command a runaway climb. Instead, after the aid has been
// continuously lost this long while AIRBORNE, latch emland_active so the
// sequencer diverts to a LiDAR-independent fixed-thrust descent (vel_down-based
// touchdown). Must exceed the spike-hold window (LIDAR_SPIKE_MAX @50Hz ~300ms)
// so a brief dropout absorbed by spike-rejection never trips it.
#define AID_LOST_EMLAND_MS   500      // aid invalid this long airborne -> emland

// ============================================================
// 12. H7 Obstacle LiDAR (I2C slave on the same QWIIC bus = Wire1)
// ============================================================
// The H7 runs the STL-27L obstacle segmentation and exposes the latest result
// as an I2C slave (32-byte ObstaclePacket). UNO Q polls it as bus master,
// alongside the TF-Nova altimeter (0x10). See ObstacleRx / ObstacleI2C.h.
#define USE_OBSTACLE_RX      1        // 1 = poll the H7 obstacle slave
#define OBSTACLE_WIRE        Wire1    // same QWIIC bus as the TF-Nova
#define OBSTACLE_I2C_ADDR    0x20     // H7 obstacle slave 7-bit address
#define OBSTACLE_READ_DIV    4        // poll every 4th 200Hz tick (~50Hz)
#define OBSTACLE_TIMEOUT_MS  300      // stale if no valid packet in this window
// Distance gate (applied on the UNO Q after receive): obstacles farther than
// this are dropped from the decoded list. H7 sends everything within its own
// 20m raw range; this trims it to the avoidance-relevant near field. Tune here.
#define OBSTACLE_MAX_DIST_MM 5000     // keep only obstacles within 5 m

// ============================================================
// 13. Collision-cone yaw-rate 장애물 회피 (MODE_PNG_GUIDANCE)
// ============================================================
// Collision-cone guidance turns the nearest H7 obstacle (range R_o + body
// heading-error e_o) into a yaw-rate command r_ca = (1+sqrt(1+K))*theta_dot,
// substituted for the PNG waypoint yaw-rate while avoid-mode is latched ON.
// Inputs are R_o (m), e_o (rad, body bearing as the H7 reports it), and v_a
// (NED horizontal speed magnitude). See CollisionCone.h and
// etc/collision_cone_png_yawrate_implementation_prompt.md.
#define USE_OBS_AVOID      1          // 1 = enable PNG collision-cone avoidance
#define OBS_AVOID_K        0.6f       // PPT guidance gain (N=1+sqrt(1+K)=2.265)
#define OBS_AVOID_RP       1.5f       // obstacle safety radius [m] (= radius+margin)
#define OBS_AVOID_RDETECT  5.0f       // avoidance start range [m] (within H7 5m gate)
#define OBS_AVOID_ROFF     6.0f       // avoidance end range [m] (Rdetect+1, hysteresis)
#define OBS_EPS_R          0.01f      // divide-by-zero guard distance [m]
// Frontal obstacle (e_o ~ 0): the cone-boundary sign is ambiguous and would
// chatter L/R, so when |e_o| < deadzone the avoidance direction is FORCED to a
// fixed side. OBS_FIXED_SIDE: +1 = turn right (yaw+), -1 = turn left (yaw-).
// VERIFY the actual turn direction on a ground test and flip the sign if wrong.
#define OBS_FIXED_SIDE     (-1.0f)    // +1 = right avoidance, -1 = left avoidance
// ---- Tunables in degrees (auto-converted to rad below; edit the *_DEG values) ----
#define OBS_AVOID_RMAX_DPS    45.0f   // yaw-rate saturation [deg/s] (avoidance turn speed)
#define OBS_GAMMA_MARGIN_DEG  10.0f    // cone-OFF hysteresis margin [deg]
#define OBS_FRONT_DEADZONE_DEG 10.0f  // |e_o| below this = frontal -> fixed side
// Derived rad values consumed by CollisionCone (do not edit; tune the *_DEG above).
#define OBS_AVOID_RMAX     (OBS_AVOID_RMAX_DPS * DEG_TO_RAD)         // [rad/s]
#define OBS_GAMMA_MARGIN   (OBS_GAMMA_MARGIN_DEG * DEG_TO_RAD)       // [rad]
#define OBS_FRONT_DEADZONE (OBS_FRONT_DEADZONE_DEG * DEG_TO_RAD)     // [rad]

// ---- Per-waypoint flags (CMD_SET_WAYPOINTS byte p[12], stored in png_wp_flags[]) ----
// MISSION mode gates collision-cone avoidance per segment: when waypoint WP[k]
// carries WP_FLAG_AVOID, avoidance is armed for the segment AFTER passing WP[k]
// (i.e. while flying WP[k] -> WP[k+1]). The first leg (origin -> WP0) has no
// "passed" waypoint, so avoidance stays off there. PNG_GUIDANCE standalone mode
// ignores these flags and keeps avoidance always on (test/manual use).
// Missions with no flags column (legacy .txt) decode to 0 -> avoidance off (safe).
#define WP_FLAG_AVOID      0x01          // bit0: arm avoidance on the post-WP segment
#define WP_FLAG_ORBIT      0x02          // bit1: on reaching this WP, orbit (U-turn)
                                         // around the midpoint of this WP and the
                                         // NEXT WP until the next WP capture radius.
// bit2..7 reserved for future per-WP behaviour (speed change, etc.)

// ---- Orbit / U-turn guidance (WP_FLAG_ORBIT, inside MODE_PNG_GUIDANCE) ----
// On reaching a WP that carries WP_FLAG_ORBIT, the craft orbits the midpoint of
// that WP and the next WP with radius = dist(WP[k],WP[k+1])/2, then resumes PNG
// straight guidance. yaw-rate law (orbit):
//   psi_dot = -(V/r)*sin(eta_cmd) - K2*(V/r)*sin(eta - eta_cmd)
//   eta = wrap_pi(LOS_to_center - yaw); eta_cmd = +/-90 deg.
// The speed term V in the law MUST equal the actual forward speed (PNG_VX_MS),
// otherwise the steady orbit radius (= V_actual / |psi_dot|) does not converge
// to r_cmd. Forward speed (u_cmd -> pitch) stays PNG_VX_MS; only yaw-rate changes.
// Turn direction is AUTO-SELECTED at orbit entry from the side the centre is on
// (eta_cmd = +PI/2 if centre is on the left at entry -> CW, else -PI/2 -> CCW),
// so the WP layout / approach heading chooses the direction. See Mode.cpp.
// Orbit ends when EITHER the next straight WP is within PNG_WP_CAPTURE_R, OR the
// accumulated turn since orbit entry reaches ORBIT_EXIT_TURN_DEG (so large radii
// that don't graze the next WP still exit after a half-turn).
#define ORBIT_K2           1.5f          // radius-error correction gain (K2)
#define ORBIT_R_MIN        0.5f          // min orbit radius (div-by-0 guard) [m]
#define ORBIT_EXIT_TURN_DEG 180.0f       // exit after this much accumulated turn [deg]

#endif // HW_CONFIG_H
