//
// RealTime timer
// Uses micros() - platform independent (Arduino API)
//

#ifndef REALTIME_H
#define REALTIME_H

#include <Arduino.h>
#include "hw_config.h"   // CONTROL_FREQ_HZ: single source of truth for loop rate

#ifdef __cplusplus
extern "C" {
#endif

// Derived from CONTROL_FREQ_HZ in hw_config.h so the realtime tick period and
// the integration dt stay locked to the one control-rate definition.
#define SAMPLING_FREQUENCY (double)(CONTROL_FREQ_HZ)
#define SAMPLING_TIME      (double)(1.0 / SAMPLING_FREQUENCY)
#define FINAL_TIME         (double)(30.0)
#define N_TIME             (int)(FINAL_TIME * SAMPLING_FREQUENCY)

typedef struct {
    int    idx;
    double cur;
    double Ti;
    double Tf;
    double cnt;
} RealTimeState;

extern RealTimeState TIME;

void init_timer(void);
void update_time(void);
double get_elapsed_seconds(void);

// Returns true if the scheduled time has arrived (non-blocking).
// Call this instead of the old busy-wait time_idling().
bool time_ready(void);

#ifdef __cplusplus
}
#endif

#endif // REALTIME_H
