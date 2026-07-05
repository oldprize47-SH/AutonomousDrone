//
// RealTime timer - Windows implementation
// Uses QueryPerformanceCounter for high-resolution timing
//

#include "RealTime.h"
#include <string.h>

RealTimeState TIME;

static LARGE_INTEGER _freq;
static LARGE_INTEGER _timer_start;

double get_elapsed_seconds(void)
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - _timer_start.QuadPart) / (double)_freq.QuadPart;
}

void init_timer(void)
{
    memset(&TIME, 0, sizeof(RealTimeState));
    TIME.idx = 0;
    TIME.cnt = 0.0;

    QueryPerformanceFrequency(&_freq);
    QueryPerformanceCounter(&_timer_start);

    TIME.cnt = get_elapsed_seconds();
    TIME.Ti  = TIME.cnt;
}

void update_time(void)
{
    TIME.idx++;
    TIME.cnt += SAMPLING_TIME;
}

void time_idling(void)
{
    while (TIME.cur <= TIME.cnt) {
        TIME.cur = get_elapsed_seconds();
    }
}
