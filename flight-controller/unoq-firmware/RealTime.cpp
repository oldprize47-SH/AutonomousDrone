//
// RealTime timer implementation
// Uses micros() - platform independent (Arduino API)
//

#include "RealTime.h"
#include <string.h>

RealTimeState TIME;

static uint32_t _timer_start_us;
static uint32_t _next_tick_us;

#define SAMPLING_PERIOD_US  ((uint32_t)(1000000.0 / SAMPLING_FREQUENCY))  // 5000 us

double get_elapsed_seconds(void)
{
    return (double)(micros() - _timer_start_us) * 1e-6;
}

void init_timer(void)
{
    memset(&TIME, 0, sizeof(RealTimeState));
    TIME.idx = 0;
    TIME.cnt = 0.0;

    _timer_start_us = micros();
    _next_tick_us = _timer_start_us + SAMPLING_PERIOD_US;
    TIME.cnt = 0.0;
    TIME.Ti  = 0.0;
}

void update_time(void)
{
    TIME.idx++;
    TIME.cnt += SAMPLING_TIME;
    _next_tick_us += SAMPLING_PERIOD_US;
}

bool time_ready(void)
{
    uint32_t now = micros();
    TIME.cur = (double)(now - _timer_start_us) * 1e-6;
    return ((int32_t)(now - _next_tick_us) >= 0);
}
