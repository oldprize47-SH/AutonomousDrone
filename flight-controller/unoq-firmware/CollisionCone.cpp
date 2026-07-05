//
// CollisionCone - PPT collision-cone yaw-rate obstacle avoidance (impl)
//

#include "CollisionCone.h"
#include "hw_config.h"   // OBS_AVOID_* tuning parameters

static inline float cc_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float CollisionCone::update(bool valid, float R_o, float e_o, float v_a,
                            float r_wp, CCDebug& dbg)
{
    // ---- Invalid / no obstacle: drop avoidance, pass the waypoint command ----
    if (!valid || R_o <= 0.0f) {
        _avoid_mode = false;
        dbg.r_ca       = 0.0f;
        dbg.Ro         = R_o;
        dbg.e_o        = e_o;
        dbg.gamma      = 0.0f;
        dbg.Ro_dot     = 0.0f;
        dbg.theta_dot  = 0.0f;
        dbg.avoid_mode = false;
        dbg.valid      = false;
        return r_wp;
    }

    const float R_p = OBS_AVOID_RP;

    // ---- Effective range (numerical guard for R_o <= R_p) ----
    float Ro_eff = fmaxf(R_o, R_p + OBS_EPS_R);

    // ---- Collision-cone half angle: gamma = asin(R_p / Ro_eff) ----
    // asin(x) = atan2(x, sqrt(1 - x^2)), avoiding asinf (nano libc).
    float ratio = cc_clampf(R_p / Ro_eff, 0.0f, 0.999999f);
    float gamma = atan2f(ratio, sqrtf(fmaxf(1.0f - ratio * ratio, 1e-6f)));

    // ---- Static-obstacle relative kinematics from e_o only ----
    float Ro_dot  = -v_a * cosf(e_o);
    float phi_dot = -v_a * sinf(e_o) / Ro_eff;

    float sqrt_arg  = fmaxf(Ro_eff * Ro_eff - R_p * R_p, 1e-6f);
    float gamma_dot = -R_p * Ro_dot / (Ro_eff * sqrtf(sqrt_arg));

    // ---- Cone-boundary angle rate (sign picks the avoidance side) ----
    // Normally the side follows e_o (obstacle left/right). For a near-frontal
    // obstacle (|e_o| < deadzone) the sign is ambiguous and would chatter L/R,
    // so force it to OBS_FIXED_SIDE (+1 right / -1 left). Outside the deadzone
    // pick the boundary that steers AWAY from the obstacle:
    //   e_o > 0 (obstacle right) -> use (phi_dot - gamma_dot) i.e. side = -1
    //   e_o < 0 (obstacle left)  -> use (phi_dot + gamma_dot) i.e. side = +1
    float side;
    if (fabsf(e_o) < OBS_FRONT_DEADZONE) {
        side = OBS_FIXED_SIDE;        // frontal: fixed avoidance direction
    } else {
        side = (e_o > 0.0f) ? -1.0f : 1.0f;   // turn away from the obstacle
    }
    float theta_dot = phi_dot + side * gamma_dot;

    // ---- Guidance gain + saturated avoidance yaw-rate ----
    float N_ca = 1.0f + sqrtf(1.0f + cc_clampf(OBS_AVOID_K, 0.0f, 10.0f));
    float r_ca = cc_clampf(N_ca * theta_dot, -OBS_AVOID_RMAX, OBS_AVOID_RMAX);

    // ---- Avoid-mode switching (hysteresis) ----
    bool cone_in     = fabsf(e_o) < gamma;
    bool cone_out    = fabsf(e_o) > gamma + OBS_GAMMA_MARGIN;
    bool approaching = Ro_dot < 0.0f;

    bool on  = (R_o < OBS_AVOID_RDETECT) && cone_in && approaching;
    bool off = (R_o > OBS_AVOID_ROFF) || cone_out;

    if (!_avoid_mode) {
        if (on)  _avoid_mode = true;
    } else {
        if (off) _avoid_mode = false;
    }

    // ---- Diagnostics ----
    dbg.r_ca       = r_ca;
    dbg.Ro         = R_o;
    dbg.e_o        = e_o;
    dbg.gamma      = gamma;
    dbg.Ro_dot     = Ro_dot;
    dbg.theta_dot  = theta_dot;
    dbg.avoid_mode = _avoid_mode;
    dbg.valid      = true;

    return _avoid_mode ? r_ca : r_wp;
}
