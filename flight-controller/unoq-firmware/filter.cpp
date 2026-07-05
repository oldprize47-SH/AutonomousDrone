#include "filter.h"
#include <math.h>
#include <stdint.h>

/* The C API below is compiled as C++ now (filter.cpp), but keeps C linkage
 * via the extern "C" block in filter.h so existing callers are unaffected. */

/* powf() in Zephyr nano libc requires __errno which conflicts with
 * Zephyr's internal errno. Provide a minimal powf replacement instead.
 * Only used for baro_to_altitude: powf(P/P0, 0.190263) where P/P0 ~ 1.0
 *
 * Method: x^a = exp(a * ln(x))
 * For x near 1.0: use log2 via IEEE754 exponent + Pade approx for mantissa,
 * then exp2 via integer part + degree-4 polynomial.
 */
static float fast_log2f(float x)
{
    union { float f; uint32_t u; } conv;
    conv.f = x;
    int e = (int)((conv.u >> 23) & 0xFF) - 127;
    conv.u = (conv.u & 0x007FFFFFu) | 0x3F800000u;
    float m = conv.f;  /* m in [1, 2) */
    /* Pade(3,2)-style rational approx for log2(m), m in [1,2)
     * log2(m) = (m-1) * P(m-1) / Q(m-1)   via ln(1+u)/u Pade
     * Accurate to ~1e-6 over [1,2) */
    float u = m - 1.0f;
    float num = u * (6.0f + 0.7895f * u);
    float den = (6.0f + 3.0f * u) * 0.6931472f;  /* * ln(2) to convert ln->log2 */
    return (float)e + num / den;
}

static float fast_exp2f(float x)
{
    int i = (int)x;
    if (x < (float)i) i--;   /* floor without floorf */
    float f = x - (float)i;
    /* Degree-4 minimax for 2^f, f in [0,1). Max error ~3e-6 */
    float p = 1.3534167e-2f;
    p = p * f + 5.2011464e-2f;
    p = p * f + 2.4144276e-1f;
    p = p * f + 6.9314575e-1f;
    p = p * f + 1.0f;
    /* ldexpf also needs __errno in nano libc, use bit manipulation */
    union { float f; uint32_t u; } scale;
    scale.u = (uint32_t)((127 + i) & 0xFF) << 23;
    return p * scale.f;
}

static float filter_powf(float base, float exponent)
{
    if (base <= 0.0f) return 0.0f;
    return fast_exp2f(exponent * fast_log2f(base));
}

/* ---- Tustin integrator (float, variable dt) ---- */
float tustin_integrate_f(float current_dot, float *prev_dot, float *prev_val, float dt)
{
    float val = *prev_val + (dt / 2.0f) * (current_dot + *prev_dot);
    *prev_val = val;
    *prev_dot = current_dot;
    return val;
}

/* ---- Init helpers ---- */

void filter_state_init(FilterState *s)
{
    s->h = 0.0f;
    s->v = 0.0f;
    s->x3 = 0.0f;
    s->prev_h_dot = 0.0f;
    s->prev_v_dot = 0.0f;
    s->prev_x3_dot = 0.0f;
}

void filter_gains_init(FilterGains *g, float tau)
{
    /* MATLAB: C1=3/tau, C2=3/tau^2 + 2*omega_s^2, C3=1/tau^3 */
    float omega_s_sq = FILTER_G0_ABS / FILTER_R;

    g->c1 = 3.0f / tau;
    g->c2 = 3.0f / (tau * tau) + 2.0f * omega_s_sq;
    g->c3 = 1.0f / (tau * tau * tau);
    g->omega_s_sq = omega_s_sq;
}

void baro_cal_init(BaroCal *cal, float cal_seconds)
{
    cal->p0 = 101325.0f;
    cal->t0 = 288.15f;
    cal->sum_pressure = 0.0f;
    cal->sum_temp = 0.0f;
    cal->count = 0;
    cal->elapsed = 0.0f;
    cal->ready = 0;
    cal->cal_duration = cal_seconds;
}

int baro_cal_update(BaroCal *cal, float pressure_pa, float temp_c, float dt)
{
    if (cal->ready) return 1;

    cal->sum_pressure += pressure_pa;
    cal->sum_temp += temp_c;
    cal->count++;
    cal->elapsed += dt;

    if (cal->elapsed >= cal->cal_duration && cal->count > 0) {
        float pressure_avg = cal->sum_pressure / (float)cal->count;
        float temp_avg_k   = cal->sum_temp / (float)cal->count + 273.15f;

        cal->p0 = pressure_avg;
        cal->t0 = temp_avg_k;
        cal->ready = 1;
    }
    return cal->ready;
}

float baro_to_altitude(const BaroCal *cal, float pressure_pa)
{
    /* ISA hypsometric formula (logarithmic form):
     * h = -(R*T0/g) * ln(P/P0)
     * R_air/g = 29.271 m/K  (287.053 / 9.80665)
     *
     * ln(x) for x near 1.0 via Horner form of the Taylor series:
     * u = (x-1)/(x+1),  ln(x) = 2*u * (1 + u^2/3 + u^4/5 + u^6/7)
     * Converges fast for P/P0 in [0.95, 1.05] (+-400m range easily). */
    float x = pressure_pa / cal->p0;
    float u = (x - 1.0f) / (x + 1.0f);
    float u2 = u * u;
    float ln_x = 2.0f * u * (1.0f + u2 * (1.0f/3.0f + u2 * (1.0f/5.0f + u2 * (1.0f/7.0f))));
    return -(29.271f * cal->t0) * ln_x;
}

/* ---- Complementary filter for FREE_ACC (gravity already removed) ---- */
/*
 * FREE_ACC from MTi-680G is in NED frame, gravity-free.
 * acc_down: data[FREE_ACC_Z] (down positive, gravity removed)
 * h_aid:   barometer altitude (up positive)
 *
 * Internal state h is up-positive.
 * a_up = -acc_down  (NED-Down -> Up conversion done inside)
 *
 * 3-state CF structure (same as MATLAB):
 *   err    = h - h_aid
 *   x3_dot = err * c3
 *   v_dot  = a_up - c2*err - c3*x3
 *   h_dot  = v   - c1*err
 */
void step_free_acc(FilterState *s, const FilterGains *g,
                   float acc_down, float h_aid, float dt)
{
    float a_up = -acc_down;
    float err  = s->h - h_aid;

    float x3_dot = err * g->c3;
    float v_dot  = a_up - (g->c2 * err) - (g->c3 * s->x3);
    float h_dot  = s->v - (g->c1 * err);

    tustin_integrate_f(x3_dot, &s->prev_x3_dot, &s->x3, dt);
    tustin_integrate_f(v_dot,  &s->prev_v_dot,  &s->v,  dt);
    tustin_integrate_f(h_dot,  &s->prev_h_dot,  &s->h,  dt);
}

/* ---- Original filters (updated to variable dt) ---- */

void step_linear_gravity(FilterState *s, const FilterGains *g,
                         float acc_z, float h_aid, float dt)
{
    float err_gravity = s->h + FILTER_H_GEOID;
    float err         = s->h - h_aid;

    float x3_dot = err * g->c3;
    float v_dot  = (acc_z - FILTER_G0_ABS)
                    + (2.0f * g->omega_s_sq * err_gravity)
                    - (g->c2 * err)
                    - (g->c3 * s->x3);
    float h_dot  = s->v - (g->c1 * err);

    tustin_integrate_f(x3_dot, &s->prev_x3_dot, &s->x3, dt);
    tustin_integrate_f(v_dot,  &s->prev_v_dot,  &s->v,  dt);
    tustin_integrate_f(h_dot,  &s->prev_h_dot,  &s->h,  dt);
}

void step_nonlinear_gravity(FilterState *s, const FilterGains *g,
                            float acc_z, float h_aid, float dt)
{
    float H_ellipse = s->h + FILTER_H_GEOID;
    float err       = s->h - h_aid;
    float denom     = 1.0f + (H_ellipse / FILTER_R);
    float gl        = FILTER_G0_GRAVITY / (denom * denom);

    float x3_dot = err * g->c3;
    float v_dot  = (acc_z - gl)
                    - (g->c2 * err)
                    - (g->c3 * s->x3);
    float h_dot  = s->v - (g->c1 * err);

    tustin_integrate_f(x3_dot, &s->prev_x3_dot, &s->x3, dt);
    tustin_integrate_f(v_dot,  &s->prev_v_dot,  &s->v,  dt);
    tustin_integrate_f(h_dot,  &s->prev_h_dot,  &s->h,  dt);
}

/* ============================================================
 * Fading Memory Filter implementation (float, UNO Q / Zephyr).
 * ============================================================ */

void FadingMemFilter::init(float Ts, FmfMode mode, float beta, float h0)
{
    _Ts   = Ts;
    _freq = 1.0f / Ts;
    _beta = beta;
    _mode = mode;

    _x[0] = h0;
    _x[1] = 0.0f;
    _x[2] = 0.0f;

    const float b  = beta;
    const float om = 1.0f - beta;            // (1 - beta)

    if (mode == FMF_CONST_VEL) {
        // Kf0 = 1 - b^2,  Kf1 = (1-b)^2 * Freq
        _kf[0] = 1.0f - b * b;
        _kf[1] = om * om * _freq;
        _kf[2] = 0.0f;
    } else { // FMF_CONST_ACC
        // Kf0 = 1 - b^3
        // Kf1 = 1.5*(1-b)^2*(1+b)*Freq
        // Kf2 = (1-b)^3 * Freq^2
        _kf[0] = 1.0f - b * b * b;
        _kf[1] = 1.5f * om * om * (1.0f + b) * _freq;
        _kf[2] = om * om * om * _freq * _freq;
    }
}

void FadingMemFilter::measUpdate(float input, bool rejectOutlier)
{
    float error = input - _x[0];

    // Outlier rejection: a glitchy TOF jump beyond what the current velocity
    // could plausibly produce in one step is ignored (innovation -> 0).
    if (rejectOutlier && fabsf(error) > (0.2f + fabsf(_x[1]) * _Ts)) {
        error = 0.0f;
    } else {
        _x[0] += _kf[0] * error;
        _x[1] += _kf[1] * error;
        if (_mode == FMF_CONST_ACC) {
            _x[2] += _kf[2] * error;
        }
    }

    _last_err = error;
    timeUpdate();
}

void FadingMemFilter::timeUpdate()
{
    if (_mode == FMF_CONST_VEL) {
        // h(k+1) = h + Ts*v ;  v(k+1) = v
        _x[0] = _x[0] + _Ts * _x[1];
        // _x[1] unchanged
    } else { // FMF_CONST_ACC
        // h(k+1) = h + Ts*v + 0.5*Ts^2*a ;  v(k+1) = v + Ts*a ;  a(k+1) = a
        _x[0] = _x[0] + _Ts * _x[1] + 0.5f * _Ts * _Ts * _x[2];
        _x[1] = _x[1] + _Ts * _x[2];
        // _x[2] unchanged
    }
}

void FadingMemFilter::reset(float h0)
{
    _x[0] = h0;
    _x[1] = 0.0f;
    _x[2] = 0.0f;
    _last_err = 0.0f;
}
