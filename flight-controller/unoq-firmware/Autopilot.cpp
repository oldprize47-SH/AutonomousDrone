//
// Autopilot - Attitude control module implementation
//

#include "Autopilot.h"
#include "hw_config.h"
#include <math.h>

// Gains carried over verbatim from the previous global PID instances.
//
//   AnglePID members: { kp, kd, ki, integral, prev_error, int_limit, out_limit }
//   RatePID  members: { kp, ki, kd, integral, prev_error, int_limit, out_limit,
//                       last_error, last_P, last_I, last_out_unsat }
//   LowPassFilter:    { cutoff_hz, dt }
//
// roll/pitch Angle PI : BW 2Hz, n=0.01
// roll/pitch Rate     : BW 5Hz, P-only
// yaw Rate            : BW 1.5Hz, PI
Autopilot::Autopilot()
    : rollAngle  { 4.188790204786391f,  0.0f, 0.5f, 0.0f, 0.0f, 100.0f, 4.0f }
    , pitchAngle { 4.188790204786391f,  0.0f, 0.5f, 0.0f, 0.0f, 100.0f, 4.0f }
    , rollRate   { 0.351348182555114f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    , pitchRate  { 0.347367156344485f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    , yawRate    { 0.1299690681f, 0.03f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    , rollRateLPF  { 35.0f, CONTROL_DT_S }
    , pitchRateLPF { 35.0f, CONTROL_DT_S }
    // AltitudePID members: { kh, ki, kv, ka, hdot_limit, i_vel_limit, integral,
    //                        last_alt_error, last_hdot_cmd, last_vel_error, last_DeltaT }
    // ka = acceleration-feedback (damping) gain; start at 0 and tune up.
    , altPID { 1.584956f,  0.01f, 13.325663f, 0.228000f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    // VelocityController members:
    //   { kp, ki, kd, out_limit, int_limit, integral, prev_error,
    //     acc_alpha, acc_lpf, last_u_error, last_theta_cmd, last_a_f }
    //   out_limit = int_limit = VEL_MAX_TILT_DEG * DEG_TO_RAD (attitude
    //   saturation + I-term anti-windup, rad).
    //   velU = X (body forward u / pitch), velV = Y (body right v / roll).
    //   ki = 0 -> pure-P law (integral unused); tune ki up to remove SS error.
    //   acc_alpha = 1st-order LPF coeff on the accel-feedback (damping) term.
    //   fc = 12 Hz @ CONTROL_DT_S: alpha = dt/(tau+dt), tau = 1/(2*pi*fc).
    , velU     { 0.206157010269972f, 0.08f, 0.015991335372069f,
                 VEL_MAX_TILT_DEG * DEG_TO_RAD, (VEL_MAX_TILT_DEG - 10) * DEG_TO_RAD,
                 0.0f, 0.0f, VEL_ACC_LPF_ALPHA, 0.0f, 0.0f, 0.0f, 0.0f }
    , velV     { 0.168284166111444f, 0.06f, 0.010356778797146f,
                 VEL_MAX_TILT_DEG * DEG_TO_RAD, (VEL_MAX_TILT_DEG - 10) * DEG_TO_RAD,
                 0.0f, 0.0f, VEL_ACC_LPF_ALPHA, 0.0f, 0.0f, 0.0f, 0.0f }
    // PositionController members:
    //   { kp, ki, kd, out_limit, int_limit, integral, prev_error,
    //     last_error, last_tilt }
    //   Single-cascade position->angle: gains absorb 1/g so output is tilt [rad].
    //   out_limit = POS_MAX_TILT_DEG (dedicated POS_CONTROL angle limit, rad) so
    //     a large step position error never produces a violent lean.
    //   int_limit = (POS_MAX_TILT_DEG - 5) (I-term anti-windup, rad).
    //   D term = -kd*vel (GNSS NED velocity damping, no LPF).
    //   Integral separation: ki runs only within POS_I_ENABLE_R of the target
    //   (gated in update_position), so it removes the near-target steady-state
    //   error without winding up over the long step move. Same gains both axes.
    //   ** Verify sign mapping + tune on the ground before flight. **
    , posN     { 0.150571615765267f, 0.024526982593270f, 0.190186447004476f,
                 POS_MAX_TILT_DEG * DEG_TO_RAD, (POS_MAX_TILT_DEG - 5) * DEG_TO_RAD,
                 0.0f, 0.0f, 0.0f, 0.0f }
    , posE     { 0.150571615765267f, 0.024526982593270f, 0.190186447004476f,
                 POS_MAX_TILT_DEG * DEG_TO_RAD, (POS_MAX_TILT_DEG - 5) * DEG_TO_RAD,
                 0.0f, 0.0f, 0.0f, 0.0f }
{
}

void Autopilot::set_angle(float roll_rad, float pitch_rad)
{
    target_roll_rad  = roll_rad;
    target_pitch_rad = pitch_rad;
}

void Autopilot::set_yaw_rate(float yaw_rate_rps)
{
    target_yaw_rate = yaw_rate_rps;
}

void Autopilot::update(float roll, float pitch,
                       float roll_rate, float pitch_rate, float yaw_rate,
                       float dt, float& U2, float& U3, float& U4)
{
    // 35Hz LPF on roll/pitch rate feedback; yaw stays raw (legacy behavior).
    float roll_rate_f  = rollRateLPF.update(roll_rate);
    float pitch_rate_f = pitchRateLPF.update(pitch_rate);

    // Outer Angle PI -> target body rate (rad/s)
    float roll_rate_cmd  = rollAngle.updatePI(target_roll_rad, roll, dt);
    float pitch_rate_cmd = pitchAngle.updatePI(target_pitch_rad, pitch, dt);

    // Inner Rate PID -> torque (Nm)
    U2 = rollRate.update(roll_rate_cmd, roll_rate_f, dt);
    U3 = pitchRate.update(pitch_rate_cmd, pitch_rate_f, dt);
    U4 = yawRate.update(target_yaw_rate, yaw_rate, dt);

    // Diagnostics for the debug frame (inner-loop setpoint + filtered feedback).
    last_roll_rate_cmd  = roll_rate_cmd;
    last_pitch_rate_cmd = pitch_rate_cmd;
    last_roll_rate_f    = roll_rate_f;
    last_pitch_rate_f   = pitch_rate_f;
}

float Autopilot::update_altitude(float h, float v, float a_down, float h_cmd, float mg, float dt)
{
    // alt_filter state is up-positive; the controller works in NED (down+).
    //   h_ned = -h,  h_cmd_ned = -h_cmd,  w_ned = -v
    // a_down (FREE_ACC_Z) is already NED down+, so it is passed through as-is.
    return altPID.update(-h_cmd, -h, -v, a_down, mg, dt);
}

float Autopilot::update_climb_rate(float climb_rate_up, float v_up, float a_down,
                                   float mg, float dt)
{
    // up+ -> NED down+ (same convention as update_altitude). A positive
    // climb_rate_up (climbing) becomes a negative NED hdot_cmd, which the inner
    // loop turns into T_cmd > mg (climb). a_down is already NED down+.
    return altPID.update_climb_rate(-climb_rate_up, -v_up, a_down, mg, dt);
}

void Autopilot::reset_altitude()
{
    altPID.reset();
}

void Autopilot::update_velocity(float u_cmd, float u_fb, float u_acc,
                                float v_cmd, float v_fb, float v_acc,
                                float dt,
                                float& pitch_cmd, float& roll_cmd)
{
    // Body forward (u) velocity -> pitch command; body right (v) velocity -> roll.
    // Each axis saturates internally to +/-VEL_MAX_TILT. The sign that maps a
    // forward velocity error to nose-down pitch is validated on the ground.
    pitch_cmd = velU.update(u_cmd, u_fb, u_acc, dt);
    roll_cmd  = velV.update(v_cmd, v_fb, v_acc, dt);
}

void Autopilot::reset_velocity()
{
    velU.reset();
    velV.reset();
}

void Autopilot::update_position(float sp_n, float sp_e, float pos_n, float pos_e,
                                float vel_n, float vel_e, float yaw,
                                float dt,
                                float& pitch_cmd, float& roll_cmd)
{
    float err_n = sp_n - pos_n;
    float err_e = sp_e - pos_e;

    // Integral separation: enable the integrator on BOTH axes only when the
    // craft is within POS_I_ENABLE_R of the target (gate on the horizontal
    // distance, not the per-axis error). Far from the target the integral is
    // frozen so it cannot wind up over a long step move; near the target it
    // removes the residual steady-state error.
    bool i_enable = (err_n * err_n + err_e * err_e)
                    <= (POS_I_ENABLE_R * POS_I_ENABLE_R);

    // Per-axis NED position PID -> RAW (unclipped) tilt command [rad] (gains
    // absorb 1/g). D term is the GNSS NED velocity fed back directly (-kd*vel).
    float tilt_n = posN.update_raw(err_n, vel_n, dt, i_enable);   // NED north tilt [rad]
    float tilt_e = posE.update_raw(err_e, vel_e, dt, i_enable);   // NED east  tilt [rad]

    // ---- VECTOR (circular) tilt saturation ----
    // Limit the MAGNITUDE of the combined (N,E) tilt vector, NOT each axis. A
    // per-axis (box) clip would scale the two axes unequally and bend the
    // commanded direction off the straight line to the target; scaling both axes
    // by the same factor preserves the N:E ratio (= direction), so the craft
    // leans straight at the waypoint. This is rotation-invariant, so doing it in
    // NED is identical to doing it in body frame.
    const float lim = posN.out_limit;                  // both axes share the limit
    float mag = sqrtf(tilt_n * tilt_n + tilt_e * tilt_e);
    float scale = 1.0f;
    if (mag > lim && mag > 1e-6f) {
        scale  = lim / mag;
        tilt_n *= scale;
        tilt_e *= scale;
    }
    // Vector anti-windup: back each axis' integral off by the same scale so a
    // frozen (saturated) integrator stays consistent with the delivered output.
    posN.apply_awu(scale);
    posE.apply_awu(scale);
    posN.note_tilt(tilt_n);   // diagnostics reflect the delivered (clipped) tilt
    posE.note_tilt(tilt_e);

    // Rotate the NED tilt command into the body frame by yaw. Same rotation
    // convention as ctrl_rotate_gnss_vel()/VEL_CONTROL (cpsi=cos, spsi=sin):
    //   forward(u) =  N*cos(psi) + E*sin(psi)
    //   right(v)   = -N*sin(psi) + E*cos(psi)
    // Forward demand maps to nose-down pitch -> negate (same sign as PNG/VEL,
    // Mode.cpp:1617). VERIFY ON GROUND before flight.
    float cpsi = cosf(yaw), spsi = sinf(yaw);
    pitch_cmd = -( tilt_n * cpsi + tilt_e * spsi);   // body forward axis
    roll_cmd  =  (-tilt_n * spsi + tilt_e * cpsi);   // body right axis
}

void Autopilot::reset_position()
{
    posN.reset();
    posE.reset();
}

void Autopilot::reset()
{
    rollAngle.reset();
    pitchAngle.reset();
    rollRate.reset();
    pitchRate.reset();
    yawRate.reset();
    rollRateLPF.reset();
    pitchRateLPF.reset();
    altPID.reset();
    velU.reset();
    velV.reset();
    posN.reset();
    posE.reset();

    last_roll_rate_cmd  = 0.0f;
    last_pitch_rate_cmd = 0.0f;
    last_roll_rate_f    = 0.0f;
    last_pitch_rate_f   = 0.0f;
}

Autopilot autopilot;

// ============================================================
// Mixing matrix: torques + total thrust -> per-motor thrust [N]
// ------------------------------------------------------------
// Mapping identical to the legacy inline implementation:
//   F = (U1 +/- U2/LY +/- U3/LX +/- U4/GAMMA) / 4, then clamp >= 0.
// ============================================================
void mix_to_thrust(float U1, float U2, float U3, float U4,
                   float& F_FR, float& F_RR, float& F_RL, float& F_FL)
{
    float inv_Gamma = 1.0f / GAMMA;

    F_FR = (U1 - U2 / LY_FR + U3 / LX_FR + U4 * inv_Gamma) / 4.0f;
    F_RR = (U1 - U2 / LY_RR - U3 / LX_RR - U4 * inv_Gamma) / 4.0f;
    F_RL = (U1 + U2 / LY_RL - U3 / LX_RL + U4 * inv_Gamma) / 4.0f;
    F_FL = (U1 + U2 / LY_FL + U3 / LX_FL - U4 * inv_Gamma) / 4.0f;

    if (F_FR < 0.0f) F_FR = 0.0f;
    if (F_RR < 0.0f) F_RR = 0.0f;
    if (F_RL < 0.0f) F_RL = 0.0f;
    if (F_FL < 0.0f) F_FL = 0.0f;
}
