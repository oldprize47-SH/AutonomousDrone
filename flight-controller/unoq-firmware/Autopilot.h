//
// Autopilot - Attitude control module
//
// Encapsulates the cascaded attitude controllers (roll/pitch Angle PI ->
// body-rate, inner Rate PID -> torque) and the yaw-rate controller.
//
// Responsibility boundary:
//   - Owns: roll/pitch angle PI, inner rate PID (roll/pitch/yaw), rate LPF.
//   - Produces: torques U2 (roll), U3 (pitch), U4 (yaw) [Nm].
//   - Does NOT handle: U1 (thrust/altitude), mixing, thrust->PWM, motor output,
//                      tilt protection, RC parsing, debug frames.
//
// Setpoints are stored via set_angle()/set_yaw_rate() and consumed by update(),
// which is called from the 200Hz control loop with the current IMU state.
//
// The free function mix_to_thrust() combines U1..U4 into per-motor thrust [N].
//

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "PIDControl.h"

class Autopilot {
public:
    Autopilot();

    // Setpoint storage (safe to call from anywhere, asynchronous to update())
    void set_angle(float roll_rad, float pitch_rad);  // target roll/pitch [rad]
    void set_yaw_rate(float yaw_rate_rps);            // target yaw rate [rad/s]

    // Called from the 200Hz loop. Current state in, torques out.
    //   roll/pitch       : current attitude [rad] (calibration already applied)
    //   roll/pitch/yaw_rate : current body rates [rad/s] (raw gyro)
    // Outputs U2/U3/U4 [Nm] via reference.
    void update(float roll_rad, float pitch_rad,
                float roll_rate, float pitch_rate, float yaw_rate,
                float dt,
                float& U2, float& U3, float& U4);

    // Altitude-hold controller -> total thrust U1 [N].
    //   h      : altitude estimate     [m], up-positive (e.g. alt_filter.h)
    //   v      : vertical speed         [m/s], up-positive (e.g. alt_filter.v)
    //   a_down : vertical acceleration  [m/s^2], NED down-positive, gravity
    //            removed (e.g. FREE_ACC_Z). Drives the acceleration-feedback
    //            damping term; pass 0 to disable it.
    //   h_cmd  : altitude setpoint      [m], up-positive (hold altitude)
    //   mg     : hover thrust m*g       [N]
    // NED conversion (up+ -> down+) is done internally. Returns T_cmd [N] (>=0).
    float update_altitude(float h, float v, float a_down, float h_cmd, float mg, float dt);

    // Climb-rate altitude control (auto takeoff/landing). Commands a constant
    // vertical rate through the AltitudePID inner velocity loop, bypassing the
    // outer altitude P(I). climb_rate_up / v_up are up-positive (+ = climbing);
    // a_down is NED down+ (FREE_ACC_Z), passed through as in update_altitude.
    float update_climb_rate(float climb_rate_up, float v_up, float a_down,
                            float mg, float dt);

    // Reset the altitude integrator only (e.g. on alt-hold toggle ON edge).
    void reset_altitude();

    // Velocity controller (accel-sensor based) -> attitude commands [rad].
    //   u_*  : body forward (x) axis (velocity command/feedback, free accel)
    //   v_*  : body right   (y) axis
    //   *_cmd: velocity setpoint [m/s], *_fb: measured velocity feedback [m/s],
    //   *_acc: free-accel feedback [m/s^2] (gravity removed), same body axis.
    // Outputs pitch_cmd/roll_cmd [rad], each saturated to +/-VEL_MAX_TILT.
    // Sign of pitch/roll mapping is validated on the ground (see Mode.cpp).
    void update_velocity(float u_cmd, float u_fb, float u_acc,
                         float v_cmd, float v_fb, float v_acc,
                         float dt,
                         float& pitch_cmd, float& roll_cmd);

    // Reset only the velocity controllers (e.g. below the enable altitude).
    void reset_velocity();

    // Position controller (RTK-GNSS based) -> attitude commands [rad].
    // Two independent NED-axis position PIDs (posN, posE) output tilt commands
    // [rad] (gains absorb 1/g), then rotated by yaw into body roll/pitch.
    //   sp_n/sp_e   : target NED position [m]
    //   pos_n/pos_e : current NED position [m]
    //   vel_n/vel_e : GNSS NED velocity [m/s] (D-term feedback)
    //   yaw         : current heading [rad] (RAW NED)
    // Outputs pitch_cmd/roll_cmd [rad]; sign mapping matches VEL_CONTROL/PNG and
    // MUST be validated on the ground.
    void update_position(float sp_n, float sp_e, float pos_n, float pos_e,
                         float vel_n, float vel_e, float yaw,
                         float dt,
                         float& pitch_cmd, float& roll_cmd);

    // Reset only the position controllers (e.g. on mode entry / fix loss).
    void reset_position();

    // Reset all internal integrators / LPF / prev_error state.
    void reset();

    // Diagnostics (yaw rate controller internals, for debug frame)
    float yaw_last_error() const { return yawRate.last_error; }
    float yaw_integral()   const { return yawRate.integral; }
    float yaw_last_I()     const { return yawRate.last_I; }

    // Diagnostics (altitude controller internals, for debug frame)
    float alt_last_error()  const { return altPID.last_alt_error; }
    float alt_last_hdot()   const { return altPID.last_hdot_cmd; }
    float alt_last_DeltaT() const { return altPID.last_DeltaT; }

    // Diagnostics (velocity controller internals, for debug frame)
    //   u_* : body forward (x) axis, v_* : body right (y) axis
    float vel_u_error() const { return velU.last_u_error; }
    float vel_u_cmd()   const { return velU.last_theta_cmd; }
    float vel_v_error() const { return velV.last_u_error; }
    float vel_v_cmd()   const { return velV.last_theta_cmd; }
    // Filtered accel feedback actually used in the -kd*a_f damping term.
    float vel_u_acc_f() const { return velU.last_a_f; }
    float vel_v_acc_f() const { return velV.last_a_f; }

    // Diagnostics (position controller internals, for debug frame).
    //   N = NED north axis, E = NED east axis. tilt is the pre-rotation
    //   NED-axis tilt command [rad]; err is the position error [m].
    float pos_n_err()  const { return posN.last_error; }
    float pos_e_err()  const { return posE.last_error; }
    float pos_n_tilt() const { return posN.last_tilt; }
    float pos_e_tilt() const { return posE.last_tilt; }

    // Diagnostics (attitude inner-loop internals, for debug frame).
    // roll/pitch_rate_cmd : outer Angle PI output (inner rate-loop setpoint)
    // roll/pitch_rate_f   : 35Hz-LPF body rate (the feedback the inner loop saw)
    float roll_rate_cmd()  const { return last_roll_rate_cmd; }
    float pitch_rate_cmd() const { return last_pitch_rate_cmd; }
    float roll_rate_f()    const { return last_roll_rate_f; }
    float pitch_rate_f()   const { return last_pitch_rate_f; }

    // Diagnostics (outer Angle PI internals, for debug frame). The outer loop
    // input is the ff-included target, so error = (target+ff) - angle. These
    // expose the firmware integrator state directly so the ground side needs no
    // int0 back-out or feed-forward reconstruction.
    float roll_angle_error()  const { return rollAngle.prev_error; }
    float pitch_angle_error() const { return pitchAngle.prev_error; }
    float roll_angle_integral()  const { return rollAngle.integral; }
    float pitch_angle_integral() const { return pitchAngle.integral; }

private:
    AnglePID rollAngle, pitchAngle;          // outer PI
    RatePID  rollRate, pitchRate, yawRate;   // inner P(ID)
    LowPassFilter rollRateLPF, pitchRateLPF; // rate feedback LPF (yaw is raw)
    AltitudePID altPID;                      // altitude-hold (cascaded P + Kv)
    VelocityController velU, velV;           // accel-based velocity -> attitude
                                             // velU = body fwd (u/pitch),
                                             // velV = body right (v/roll)
    PositionController posN, posE;           // RTK-GNSS position -> attitude
                                             // posN = NED north, posE = NED east

    float target_roll_rad  = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate  = 0.0f;

    // Attitude inner-loop diagnostics (written by update()).
    float last_roll_rate_cmd  = 0.0f;  // outer Angle PI output (roll)  [rad/s]
    float last_pitch_rate_cmd = 0.0f;  // outer Angle PI output (pitch) [rad/s]
    float last_roll_rate_f    = 0.0f;  // 35Hz-LPF roll rate feedback   [rad/s]
    float last_pitch_rate_f   = 0.0f;  // 35Hz-LPF pitch rate feedback  [rad/s]
};

extern Autopilot autopilot;

// Mixing matrix: torques + total thrust -> per-motor thrust [N].
// Negative results are clamped to 0. PWM conversion is the caller's job.
// Motor mapping matches the legacy inline implementation.
void mix_to_thrust(float U1, float U2, float U3, float U4,
                   float& F_FR, float& F_RR, float& F_RL, float& F_FL);

#endif // AUTOPILOT_H
