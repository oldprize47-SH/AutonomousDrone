//
// CollisionCone - PPT collision-cone yaw-rate obstacle avoidance
//
// Turns one obstacle (range R_o, body heading-error e_o) plus the craft's
// horizontal speed v_a into a yaw-rate command that steers the craft along the
// collision-cone boundary:
//
//   r_ca = sat( (1 + sqrt(1 + K)) * theta_dot, -r_max, r_max )
//
// where theta_dot = phi_dot +/- gamma_dot (sign from e_o). The module owns the
// avoid-mode latch (hysteresis on range + cone), and returns the waypoint
// yaw-rate r_wp unchanged whenever avoidance is not active.
//
// Reference: etc/collision_cone_png_yawrate_implementation_prompt.md.
// Convention (per the spec): e_o > 0 -> obstacle to the left of heading.
// The H7 obstacle bearing is fed in as-is; flip the sign at the call site if a
// ground test shows the craft turning toward the obstacle.
//
// nano libc note: asin() is avoided (uncertain link availability); the cone
// half-angle uses the identity asin(x) = atan2(x, sqrt(1 - x^2)). Only
// sinf/cosf/atan2f/sqrtf/fabsf/fmaxf are used.
//

#ifndef COLLISION_CONE_H
#define COLLISION_CONE_H

#include <Arduino.h>

// Per-step diagnostics (logged to the DebugFrame; see Telem.h Group 4b).
struct CCDebug {
    float r_ca;        // avoidance yaw-rate command [rad/s] (saturated)
    float Ro;          // obstacle range [m]
    float e_o;         // obstacle heading error [rad]
    float gamma;       // collision-cone half angle [rad]
    float Ro_dot;      // range rate [m/s] (<0 = approaching)
    float theta_dot;   // cone-boundary angle rate [rad/s]
    bool  avoid_mode;  // avoidance latch active
    bool  valid;       // valid obstacle input this step
};

class CollisionCone {
public:
    // One control step. When valid is false (no fresh obstacle), the avoid-mode
    // latch is cleared and r_wp is returned unchanged. Otherwise computes the
    // collision-cone yaw-rate and returns r_ca while avoid-mode is latched ON,
    // r_wp otherwise. dbg is always filled.
    //   valid : fresh, in-gate obstacle present
    //   R_o   : obstacle range [m]
    //   e_o   : obstacle body heading-error [rad]
    //   v_a   : NED horizontal speed magnitude [m/s]
    //   r_wp  : nominal waypoint yaw-rate command [rad/s]
    float update(bool valid, float R_o, float e_o, float v_a,
                 float r_wp, CCDebug& dbg);

    // Clear the avoid-mode latch (call on mode enter/exit and HOLD/MANUAL).
    void reset() { _avoid_mode = false; }

    bool avoidMode() const { return _avoid_mode; }

private:
    bool _avoid_mode = false;
};

#endif // COLLISION_CONE_H
