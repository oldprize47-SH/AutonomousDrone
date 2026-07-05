//
// PID Controller - Cascaded (Angle + Rate) Architecture
//
// Outer loop: AnglePID (PI) - angle error -> target rate (rad/s)
// Inner loop: RatePID (PID) - rate error -> torque (Nm)
//

#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#define PI 3.14159265358979323846f

inline float wrap_pi(float x)
{
    while (x > PI)  x -= 2.0f * PI;
    while (x < -PI) x += 2.0f * PI;
    return x;
}

// ============================================================
// Outer loop: Angle -> Target Rate
// ============================================================
struct LowPassFilter {

    float cutoff_hz;
    float alpha;
    float state;

    LowPassFilter(float fc, float dt)
        : cutoff_hz(fc), state(0.0f)
    {
        float tau = 1.0f / (2.0f * PI * cutoff_hz);
        alpha = dt / (tau + dt);
    }

    float update(float input)
    {
        state = alpha * input + (1.0f - alpha) * state;
        return state;
    }

    void reset()
    {
        state = 0.0f;
    }
};

struct AnglePID {

    float kp;
	float kd;
    float ki;

    float integral;
	float prev_error;

    float int_limit;
    float out_limit;

    float update(
        float target_angle,
        float current_angle,
        float gyro_rate,
        float dt)
    {
        float error =
            target_angle - current_angle;

        integral += 0.5f * (error + prev_error) * dt;
		prev_error = error;

        if (integral > int_limit)
            integral = int_limit;

        if (integral < -int_limit)
            integral = -int_limit;

        float out =
            kp * error
            + ki * integral
            - kd * gyro_rate;

        if (out > out_limit)
            out = out_limit;

        if (out < -out_limit)
            out = -out_limit;

        return out;
    }

    // Cascaded outer loop: pure PI (no D), output = target rate (rad/s).
    // Integral uses Tustin (trapezoidal) discretization.
    // out_limit is the maximum commanded body rate (rad/s).
    float updatePI(
        float target_angle,
        float current_angle,
        float dt)
    {
        float error = target_angle - current_angle;

        integral += 0.5f * (error + prev_error) * dt;
        prev_error = error;

        if (integral > int_limit)
            integral = int_limit;
        if (integral < -int_limit)
            integral = -int_limit;

        float out = kp * error + ki * integral;

        if (out > out_limit)
            out = out_limit;
        if (out < -out_limit)
            out = -out_limit;

        return out;
    }
 
    float updateWrapped(
        float target_angle,
        float current_angle,
        float gyro_rate,
        float dt)
    {
        float error =
            wrap_pi(target_angle - current_angle);

        integral += error * dt;

        if (integral > int_limit)
            integral = int_limit;

        if (integral < -int_limit)
            integral = -int_limit;

        float out =
            kp * error
            + ki * integral
            - kd * gyro_rate;

        if (out > out_limit)
            out = out_limit;

        if (out < -out_limit)
            out = -out_limit;

        return out;
    }

    void reset()
    {
    	integral = 0.0f;
    	prev_error = 0.0f;
    }
};

// ============================================================
// Inner loop: Rate -> Torque
// ============================================================

struct RatePID {

    float kp, ki, kd;
    float integral;
    float prev_error;
    float int_limit;   // integral windup limit
    float out_limit;   // output clamp (max torque, Nm)

    // Diagnostic outputs (written by update())
    float last_error;    // rate error
    float last_P;        // proportional term
    float last_I;        // integral term
    float last_out_unsat; // output before saturation

    float update(float target_rate, float current_rate, float dt) {
        float error = target_rate - current_rate;
        integral += 0.5f * (error + prev_error) * dt;
        float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
        prev_error = error;
        float P = kp * error;
        float I = ki * integral;
        float out = P + I + (kd * derivative);
        last_error = error;
        last_P = P;
        last_I = I;
        last_out_unsat = out;
        if (out >  out_limit) out =  out_limit;
        if (out < -out_limit) out = -out_limit;
        return out;
    }

    void reset() { integral = 0.0f; prev_error = 0.0f;
                    last_error = 0.0f; last_P = 0.0f; last_I = 0.0f; last_out_unsat = 0.0f; }
};



// ============================================================
// Altitude controller (cascaded: outer alt P(I) -> hdot_cmd, inner Kv -> DeltaT)
// ------------------------------------------------------------
// All quantities NED (down-positive):
//   h        = altitude feedback        (down+, = -alt_filter.h)
//   h_cmd    = altitude command         (down+, = -hold_alt at toggle ON)
//   w        = body-z vertical speed    (down+); pass 0 to disable inner feedback
// Output is total thrust command T_cmd [N] = m*g - DeltaT_cmd, saturated >= 0.
// Sign check (climb): alt_error<0, hdot_cmd<0, DeltaT_cmd<0, T_cmd>mg.
// ============================================================
struct AltitudePID {

    float kh;          // outer P gain (alt_error -> hdot_cmd)
    float ki;          // outer I gain (0 = disabled)
    float kv;          // inner P gain (vel_error -> DeltaT_cmd)
    float ka;          // acceleration feedback (damping) gain (0 = disabled)

    float hdot_limit;  // |hdot_cmd| saturation [m/s]
    float i_vel_limit; // |I-term hdot component| saturation [m/s] (anti-windup)

    float integral;    // alt-error integral state

    // Diagnostics (written by update())
    float last_alt_error;
    float last_hdot_cmd;
    float last_vel_error;
    float last_DeltaT;

    // h, h_cmd, w, a are NED (down+). mg = m*g [N]. Returns saturated T_cmd [N].
    //   a = vertical acceleration (gravity removed, e.g. FREE_ACC_Z); pass 0 to
    //   disable the acceleration-feedback damping term.
    float update(float h_cmd, float h, float w, float a, float mg, float dt)
    {
        float alt_error = h_cmd - h;

        // Outer: P(I) -> hdot_cmd
        float i_term = 0.0f;
        if (ki > 0.0f) {
            integral += alt_error * dt;
            i_term = ki * integral;
            if (i_term >  i_vel_limit) { i_term =  i_vel_limit; }
            if (i_term < -i_vel_limit) { i_term = -i_vel_limit; }
        }

        float hdot_cmd = kh * alt_error + i_term;

        // Saturate hdot_cmd; anti-windup: hold integral when saturated
        bool saturated = false;
        if (hdot_cmd >  hdot_limit) { hdot_cmd =  hdot_limit; saturated = true; }
        if (hdot_cmd < -hdot_limit) { hdot_cmd = -hdot_limit; saturated = true; }
        if (ki > 0.0f && saturated) {
            integral -= alt_error * dt;   // undo this step's accumulation
        }

        // Inner: P on vel_error + acceleration feedback -> DeltaT_cmd.
        //   vel term : kv * (hdot_cmd - w)   (w=0 disables velocity feedback)
        //   acc term : -ka * a               (a=0 disables damping)
        // Sign (NED down+): downward acceleration a>0 lowers DeltaT_cmd, so
        // T_cmd = mg - DeltaT_cmd rises and resists the drop. ka=0 -> no change.
        float vel_error  = hdot_cmd - w;
        float DeltaT_cmd = kv * vel_error - ka * a;

        float T_cmd = mg - DeltaT_cmd;
        if (T_cmd < 0.0f) T_cmd = 0.0f;

        last_alt_error = alt_error;
        last_hdot_cmd  = hdot_cmd;
        last_vel_error = vel_error;
        last_DeltaT    = DeltaT_cmd;
        return T_cmd;
    }

    // Climb-rate control: drive the inner velocity loop DIRECTLY with a
    // commanded vertical rate, bypassing the outer altitude P(I). Used by auto
    // takeoff/landing (MODE_AUTO_TKO_LAND) so the descent/climb speed is held
    // constant instead of derived from an altitude error. The inner kv/ka loop
    // and the T_cmd = mg - DeltaT clamp are identical to update(); only the
    // hdot_cmd source differs. All args NED down+ (hdot_cmd>0 descends).
    float update_climb_rate(float hdot_cmd, float w, float a, float mg, float dt)
    {
        (void)dt;
        if (hdot_cmd >  hdot_limit) hdot_cmd =  hdot_limit;
        if (hdot_cmd < -hdot_limit) hdot_cmd = -hdot_limit;

        float vel_error  = hdot_cmd - w;
        float DeltaT_cmd = kv * vel_error - ka * a;   // same inner loop as update()

        float T_cmd = mg - DeltaT_cmd;
        if (T_cmd < 0.0f) T_cmd = 0.0f;

        // No outer alt error in this mode; keep the integral idle for a bump-free
        // hand-off back to altitude-hold (the caller resets it on transition).
        last_alt_error = 0.0f;
        last_hdot_cmd  = hdot_cmd;
        last_vel_error = vel_error;
        last_DeltaT    = DeltaT_cmd;
        return T_cmd;
    }

    void reset()
    {
        integral = 0.0f;
        last_alt_error = 0.0f;
        last_hdot_cmd  = 0.0f;
        last_vel_error = 0.0f;
        last_DeltaT    = 0.0f;
    }
};

// ============================================================
// Velocity controller (accel-sensor based, single axis)
// ------------------------------------------------------------
//   theta_cmd = kp*(u_cmd - u_out) + ki*integral - kd*a_lpf
// The integral (Tustin/trapezoidal) removes the steady-state
// velocity error left by a pure-P law against drag/trim. The
// derivative term is NOT numerically differentiated; the
// same-axis free acceleration a_out is used directly, but first
// passed through a 1st-order low-pass filter (acc_alpha) so that
// noisy accelerometer feedback does not shake the attitude command.
//   u_cmd  : velocity command   [m/s]
//   u_out  : measured velocity  [m/s] (same body axis as a_out)
//   a_out  : free acceleration  [m/s^2] (gravity removed, same axis)
// Output: attitude command [rad], saturated to +/- out_limit.
// Anti-windup: integral is held (not accumulated) while the output
// is saturated. With ki=0 this is a pure-P law (integral unused).
// acc_alpha = dt/(tau+dt), tau = 1/(2*pi*fc). acc_alpha>=1 (or fc<=0)
// disables filtering (passes a_out through unchanged).
// ============================================================
struct VelocityController {
    float kp;
    float ki;
    float kd;
    float out_limit;        // [rad] = VEL_MAX_TILT_DEG * DEG_TO_RAD
    float int_limit;        // |ki*integral| clamp [rad] (anti-windup)

    float integral;         // velocity-error integral state
    float prev_error;       // previous error (Tustin)

    float acc_alpha;        // accel-feedback LPF coefficient (1 = no filtering)
    float acc_lpf;          // accel-feedback LPF state [m/s^2]

    // Diagnostics (written by update())
    float last_u_error;
    float last_theta_cmd;
    float last_a_f;         // filtered accel feedback actually used in -kd*a_f

    float update(float u_cmd, float u_out, float a_out, float dt) {
        float u_error = u_cmd - u_out;

        // 1st-order LPF on the acceleration feedback (damping term input).
        float a_f = a_out;
        if (acc_alpha < 1.0f) {
            acc_lpf += acc_alpha * (a_out - acc_lpf);
            a_f = acc_lpf;
        }
        last_a_f = a_f;

        // Tustin integral; clamp the I *contribution* (ki*integral) so the
        // anti-windup limit is in output (rad) units regardless of ki.
        if (ki > 0.0f) {
            integral += 0.5f * (u_error + prev_error) * dt;
            float i_term = ki * integral;
            if (i_term >  int_limit) { integral =  int_limit / ki; }
            if (i_term < -int_limit) { integral = -int_limit / ki; }
        }
        prev_error = u_error;

        float i_term = (ki > 0.0f) ? ki * integral : 0.0f;
        float theta  = kp * u_error + i_term - kd * a_f;

        // Saturate output; hold the integral when saturated (anti-windup).
        bool saturated = false;
        if (theta >  out_limit) { theta =  out_limit; saturated = true; }
        if (theta < -out_limit) { theta = -out_limit; saturated = true; }
        if (ki > 0.0f && saturated) {
            integral -= 0.5f * (u_error + prev_error) * dt;  // undo this step
        }

        last_u_error   = u_error;
        last_theta_cmd = theta;
        return theta;
    }

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
        acc_lpf = 0.0f;
        last_u_error = 0.0f;
        last_theta_cmd = 0.0f;
        last_a_f = 0.0f;
    }
};

// ============================================================
// Position controller (RTK-GNSS based, single axis, NED)
// ------------------------------------------------------------
//   tilt = kp*pos_err + ki*integral - kd*vel
// Single-cascade position->angle law: the position PID outputs a tilt
// command [rad] DIRECTLY (no velocity inner loop). The gains already absorb
// the 1/g small-angle conversion (accel -> tilt), so the output, integral
// and int_limit are all in radians.
//   pos_err : position error [m]  (sp - pos, one NED axis)
//   vel     : GNSS velocity  [m/s] (same NED axis)
// The D term is GNSS velocity fed back directly (NOT numerically differentiated
// position error): -kd*vel provides the velocity damping the absent inner loop
// would have given. No LPF on vel (unlike VelocityController's accel feedback).
// Output: tilt command [rad], saturated to +/- out_limit.
// Anti-windup: i_term (ki*integral) clamped to +/- int_limit (rad), and the
// integral is held (this step's accumulation undone) while the output is
// saturated. With ki=0 this is a pure PD law (integral unused).
// Integral separation: the caller passes i_enable=false when far from the
// target (large position error) so the integral neither accumulates nor
// contributes, then i_enable=true only near the target to remove the residual
// steady-state error. This prevents the integral from winding up over long
// step moves. The integral state is frozen (not reset) while disabled.
// ============================================================
struct PositionController {
    float kp;
    float ki;
    float kd;
    float out_limit;        // [rad] = VEL_MAX_TILT_DEG * DEG_TO_RAD
    float int_limit;        // |ki*integral| clamp [rad] (anti-windup)

    float integral;         // position-error integral state
    float prev_error;       // previous error (Tustin)

    // Diagnostics (written by update())
    float last_error;       // last position error [m]
    float last_tilt;        // last tilt command [rad]

    // i_step: this step's Tustin increment, cached so a later VECTOR anti-windup
    //   (apply_awu) can undo it by the same scale on both NED axes.
    float i_step;
    // i_enabled_last: whether the integral was actually integrated this step
    //   (ki>0 && in the enable band). apply_awu only backs off when true.
    bool  i_enabled_last;

    // ------------------------------------------------------------------
    // Two-axis position control must saturate the COMBINED (N,E) tilt as a
    // VECTOR, not per-axis: a per-axis (box) clip changes the N:E ratio and
    // therefore bends the commanded direction off the straight line. So the
    // per-axis controller no longer clips here. update_raw() returns the raw,
    // UNCLIPPED tilt (kp*e + ki*I - kd*v); the caller (Autopilot::update_position)
    // limits the vector magnitude, then calls apply_awu() to back the integral
    // off by the same scale the vector clip applied (vector anti-windup).
    // ------------------------------------------------------------------

    // i_enable: integral separation gate. false -> the integral is frozen (no
    // accumulation, no contribution) for this step, so it cannot wind up while
    // far from the target. true -> normal Tustin integration.
    float update_raw(float pos_err, float vel, float dt, bool i_enable) {
        i_step = 0.0f;
        i_enabled_last = (ki > 0.0f && i_enable);

        // Tustin integral (only inside the enable band); clamp the I
        // *contribution* (ki*integral) so the per-axis windup limit is in output
        // (rad) units regardless of ki. The vector clip is applied later.
        if (i_enabled_last) {
            i_step = 0.5f * (pos_err + prev_error) * dt;
            integral += i_step;
            float i_term = ki * integral;
            if (i_term >  int_limit) { integral =  int_limit / ki; i_step = 0.0f; }
            if (i_term < -int_limit) { integral = -int_limit / ki; i_step = 0.0f; }
        }
        prev_error = pos_err;

        float i_term = i_enabled_last ? ki * integral : 0.0f;
        // D term: GNSS velocity fed back directly (velocity damping).
        float tilt = kp * pos_err + i_term - kd * vel;

        last_error = pos_err;
        last_tilt  = tilt;   // overwritten with the clipped value by note_tilt()
        return tilt;
    }

    // Vector anti-windup: after the caller scales the (N,E) tilt vector by
    // `scale` (<1 when the vector was clipped), undo part of this step's integral
    // increment so the frozen integrator matches the delivered output. scale>=1
    // (no clip) leaves the integral untouched.
    void apply_awu(float scale) {
        if (i_enabled_last && scale < 1.0f) {
            integral -= (1.0f - scale) * i_step;
        }
    }

    // Record the actually-delivered (post vector-clip) tilt for diagnostics.
    void note_tilt(float tilt) { last_tilt = tilt; }

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
        last_error = 0.0f;
        last_tilt = 0.0f;
        i_step = 0.0f;
        i_enabled_last = false;
    }
};

// All controller instances (attitude + altitude + velocity) are owned by the
// Autopilot module (Autopilot.h/.cpp). Only the struct definitions above are
// exported.

#endif // PID_CONTROL_H
