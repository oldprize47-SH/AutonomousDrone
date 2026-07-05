//
// RC Receiver Input Implementation
//

#include "RCInput.h"

static const uint8_t rc_pins[RC_NUM_CHANNELS] = {
    RC_CH1_PIN, RC_CH2_PIN, RC_CH3_PIN, RC_CH4_PIN, RC_CH5_PIN, RC_CH6_PIN
};
static volatile uint32_t rc_rise_time[RC_NUM_CHANNELS];
volatile uint16_t rc_pulse_us[RC_NUM_CHANNELS];
bool rc_active = false;

static void rc_isr(int ch) {
    if (digitalRead(rc_pins[ch]) == HIGH) {
        rc_rise_time[ch] = micros();
    } else {
        uint32_t dt = micros() - rc_rise_time[ch];
        if (dt >= 800 && dt <= 2200) rc_pulse_us[ch] = (uint16_t)dt;
    }
}

static void isr0() { rc_isr(0); }
static void isr1() { rc_isr(1); }
static void isr2() { rc_isr(2); }
static void isr3() { rc_isr(3); }
static void isr4() { rc_isr(4); }
static void isr5() { rc_isr(5); }
static void (*isr_table[RC_NUM_CHANNELS])() = { isr0, isr1, isr2, isr3, isr4, isr5 };

void rc_init() {
    if (rc_active) return;  // already initialized
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        rc_pulse_us[i] = 0;
        rc_rise_time[i] = 0;
        pinMode(rc_pins[i], INPUT);
        attachInterrupt(digitalPinToInterrupt(rc_pins[i]), isr_table[i], CHANGE);
    }
    rc_active = true;
}

void rc_detach() {
    if (!rc_active) return;
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        detachInterrupt(digitalPinToInterrupt(rc_pins[i]));
    }
    rc_active = false;
}
