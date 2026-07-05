//
// QuadPWM - 4-Channel Hardware PWM output for quadrotor ESC control
// Arduino UNO Q (STM32U585) / Direct timer register control
//
// Pin assignment (JDIGITAL header):
//   CH0 = D3  (PB0,  TIM3_CH3) - Front-Left
//   CH1 = D5  (PA11, TIM1_CH4) - Rear-Left
//   CH2 = D6  (PB1,  TIM3_CH4) - Rear-Right
//   CH3 = D9  (PB8,  TIM4_CH3) - Front-Right
//

#ifndef QUAD_PWM_H
#define QUAD_PWM_H

#include <Arduino.h>
#include <stdint.h>

#define QUAD_PWM_NUM_CHANNELS 4

enum PwmProtocol {
    PWM_STANDARD,    // 50 Hz,   1000-2000 us
    PWM_ONESHOT125   // Not supported - fallback to standard
};

class QuadPWM {
public:
    QuadPWM();
    ~QuadPWM();

    bool begin(PwmProtocol protocol = PWM_STANDARD);
    void write(uint8_t channel, float throttle);
    void writeAll(float m1, float m2, float m3, float m4);
    void disarm();
    void writeMicroseconds(uint8_t channel, uint16_t us);

    PwmProtocol getProtocol()                   const;
    uint16_t    getPulseWidth(uint8_t channel)   const;
    uint16_t    getMinPulse()                    const { return _min_us; }
    uint16_t    getMaxPulse()                    const { return _max_us; }
    bool        isInitialized()                  const { return _initialized; }

private:
    PwmProtocol _protocol;
    uint16_t    _min_us;
    uint16_t    _max_us;
    uint16_t    _current_us[QUAD_PWM_NUM_CHANNELS];
    bool        _initialized;

    uint16_t throttleToPulse(float t) const;

    void initTimerPwm();
    void setChannelUs(uint8_t channel, uint16_t us);
};

#endif // QUAD_PWM_H
