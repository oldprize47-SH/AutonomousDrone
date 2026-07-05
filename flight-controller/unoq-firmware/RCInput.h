//
// RC Receiver Input - 5-Channel PWM via ISR
//

#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>
#include "hw_config.h"

extern volatile uint16_t rc_pulse_us[RC_NUM_CHANNELS];
extern bool rc_active;

void rc_init();
void rc_detach();

#endif // RC_INPUT_H
