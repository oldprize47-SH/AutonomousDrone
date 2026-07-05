//
// RealTime timer - Windows implementation
// Uses QueryPerformanceCounter for high-resolution timing
//

#ifndef REALTIME_H
#define REALTIME_H

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLING_FREQUENCY (double)(500.0)
#define SAMPLING_TIME      (double)(1.0 / SAMPLING_FREQUENCY)

typedef struct {
    int    idx;
    double cur;
    double Ti;
    double Tf;
    double cnt;
} RealTimeState;

extern RealTimeState TIME;

void   init_timer(void);
void   update_time(void);
void   time_idling(void);
double get_elapsed_seconds(void);

#ifdef __cplusplus
}
#endif

#endif // REALTIME_H
