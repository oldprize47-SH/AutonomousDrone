#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Constants ---- */
#define FILTER_R            6378137.0f      /* Earth radius [m]                 */
#define FILTER_G0_ABS       9.7803267714f   /* normal gravity [m/s^2]           */
#define FILTER_G0_GRAVITY   9.7983f         /* local gravity (Pohang) [m/s^2]   */
#define FILTER_H_GEOID      29.33f          /* geoid undulation [m]             */

/* ISA atmosphere */
#define FILTER_ISA_GAMMA    0.0065f         /* lapse rate [K/m]                 */
#define FILTER_ISA_EXPONENT 0.190263f       /* pressure-altitude exponent       */

/* ---- Data types ---- */
typedef struct FilterState {
    float h;               /* height          */
    float v;               /* vertical speed  */
    float x3;              /* integrator (bias estimator) */
    float prev_h_dot;
    float prev_v_dot;
    float prev_x3_dot;
} FilterState;

typedef struct FilterGains {
    float c1;              /* h_dot feedback   = 3/tau             */
    float c2;              /* v_dot feedback   = 3/tau^2 + 2*ws^2  */
    float c3;              /* x3_dot feedback  = 1/tau^3            */
    float omega_s_sq;      /* Schuler freq^2  = g0/R               */
} FilterGains;

/* Barometer P0 calibration state */
typedef struct BaroCal {
    float p0;              /* reference pressure [Pa]              */
    float t0;              /* reference temperature [K]            */
    float sum_pressure;
    float sum_temp;
    int   count;
    float elapsed;
    int   ready;           /* 1 = calibration complete             */
    float cal_duration;    /* calibration duration [s]             */
} BaroCal;

/* ---- Public API ---- */

/* Tustin (bilinear) integrator - single step */
float tustin_integrate_f(float current_dot, float *prev_dot, float *prev_val, float dt);

/* Initialize filter state to zero */
void filter_state_init(FilterState *s);

/* Initialize gains from time constant tau [s] */
void filter_gains_init(FilterGains *g, float tau);

/* Initialize barometer calibration (cal_seconds = P0 averaging duration) */
void baro_cal_init(BaroCal *cal, float cal_seconds);

/* Feed barometer pressure [Pa] and temperature [C]. */
int  baro_cal_update(BaroCal *cal, float pressure_pa, float temp_c, float dt);

/* Convert pressure [Pa] to altitude [m] using calibrated P0/T0 */
float baro_to_altitude(const BaroCal *cal, float pressure_pa);

/* Complementary filter step for FREE_ACC input (gravity already removed). */
void step_free_acc(FilterState *s, const FilterGains *g,
                   float acc_down, float h_aid, float dt);

#ifdef __cplusplus
}
#endif

/* ============================================================
 * Fading Memory Filter (alpha-beta / alpha-beta-gamma) for LiDAR
 * altitude + vertical-speed estimation. C++-only (class), so it lives
 * outside the extern "C" block above.
 *
 * Ported from the desktop DT_Filter.cpp FadingMemFilter for the UNO Q
 * (STM32U585 / Zephyr): float math, no printf, no heap, fixed-size state.
 *
 * Single scalar input (altitude, up-positive). Estimates:
 *   X[0] = height   [m]   (up-positive, same frame as the input)
 *   X[1] = velocity [m/s] (up-positive)
 *   X[2] = accel    [m/s^2] (Constant_Acc only; analysis/log, not control)
 *
 * Steady-state fading-memory gains (memory parameter beta in (0,1)):
 *   Constant_Vel : Kf0 = 1-b^2,           Kf1 = (1-b)^2 * Freq
 *   Constant_Acc : Kf0 = 1-b^3,           Kf1 = 1.5*(1-b)^2*(1+b)*Freq,
 *                  Kf2 = (1-b)^3 * Freq^2
 * Smaller beta -> trusts new measurements more (faster, noisier).
 * Larger  beta -> keeps past estimate more     (slower, smoother).
 * ============================================================ */
#ifdef __cplusplus

enum FmfMode { FMF_CONST_VEL = 0, FMF_CONST_ACC = 1 };

class FadingMemFilter {
public:
    // Configure the filter. Ts = sample period [s], beta in (0,1), h0 = initial
    // height. Recomputes the gains; safe to call again to re-initialize.
    void init(float Ts, FmfMode mode, float beta, float h0);

    // Measurement update with one altitude sample (up-positive [m]).
    // When rejectOutlier is true, a sample whose innovation exceeds
    // (0.2 + |v|*Ts) is treated as an outlier (innovation forced to 0) so a
    // TOF glitch does not corrupt the estimate. Then runs the time update.
    // Outputs are read via height()/velocity()/accel().
    void measUpdate(float input, bool rejectOutlier);

    // Re-seed the height; zero velocity/accel.
    void reset(float h0);

    float height()   const { return _x[0]; }   // up-positive [m]
    float velocity() const { return _x[1]; }   // up-positive [m/s]
    float accel()    const { return _x[2]; }   // up-positive [m/s^2] (ConstAcc)
    float lastError() const { return _last_err; }

private:
    void timeUpdate();

    FmfMode _mode  = FMF_CONST_VEL;
    float   _Ts    = 0.005f;
    float   _freq  = 200.0f;
    float   _beta  = 0.90f;
    float   _kf[3] = {0.0f, 0.0f, 0.0f};
    float   _x[3]  = {0.0f, 0.0f, 0.0f};
    float   _last_err = 0.0f;
};

#endif /* __cplusplus */

#endif /* FILTER_H */
