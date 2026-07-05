//
// QuadPWM - 4-Channel Hardware PWM (STM32U585 timer registers)
//
// Uses TIM1, TIM3, TIM4 in PWM mode 1 for jitter-free 50Hz ESC signals.
// No dependency on Arduino Servo library.
//
// Timer mapping:
//   CH0 = D3  -> PB0,  TIM3_CH3  (AF2)
//   CH1 = D5  -> PA11, TIM1_CH4  (AF1)
//   CH2 = D6  -> PB1,  TIM3_CH4  (AF2)
//   CH3 = D9  -> PB8,  TIM4_CH3  (AF2)
//
// Clock: 160 MHz system clock
// PSC  = 159  -> timer tick = 1 us
// ARR  = 19999 -> period = 20 ms (50 Hz)
// CCRx = pulse width in us (1000..2000)
//

#include "QuadPWM.h"
#include "hw_config.h"
#include <stm32u5xx.h>

// ---- GPIO AF helpers ----

// Set pin to AF mode with specified alternate function number
static void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    // MODER: 10 = alternate function
    port->MODER &= ~(3U << (pin * 2));
    port->MODER |=  (2U << (pin * 2));

    // OSPEEDR: very high speed
    port->OSPEEDR |= (3U << (pin * 2));

    // PUPDR: no pull
    port->PUPDR &= ~(3U << (pin * 2));

    // AFR[0] for pin 0-7, AFR[1] for pin 8-15
    uint8_t idx = (pin < 8) ? 0 : 1;
    uint8_t pos = (pin & 7) * 4;
    port->AFR[idx] &= ~(0xFU << pos);
    port->AFR[idx] |=  ((uint32_t)af << pos);
}

// ---- Timer PSC/ARR constants for 50Hz @ 160MHz ----
static const uint16_t PWM_PSC = 159;    // 160MHz / (159+1) = 1MHz -> 1us tick
static const uint16_t PWM_ARR = 19999;  // (19999+1) * 1us = 20ms -> 50Hz

// ============================================================
// Constructor / Destructor
// ============================================================

QuadPWM::QuadPWM()
    : _protocol(PWM_STANDARD)
    , _min_us(1000)
    , _max_us(2000)
    , _initialized(false)
{
    for (int i = 0; i < QUAD_PWM_NUM_CHANNELS; i++) {
        _current_us[i] = 0;
    }
}

QuadPWM::~QuadPWM()
{
    if (_initialized) disarm();
}

// ============================================================
// initTimerPwm() - Configure GPIO AF and timer registers
// ============================================================

void QuadPWM::initTimerPwm()
{
    // ---- Enable GPIO clocks ----
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN;
    volatile uint32_t dummy;
    dummy = RCC->AHB2ENR1;
    (void)dummy;

    // ---- Enable timer clocks ----
    // TIM1 on APB2, TIM3/TIM4 on APB1
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN;
    dummy = RCC->APB1ENR1;
    (void)dummy;

    // ---- Configure GPIO pins as AF ----
    // PB0 -> TIM3_CH3, AF2
    gpio_set_af(GPIOB, 0, 2);
    // PA11 -> TIM1_CH4, AF1
    gpio_set_af(GPIOA, 11, 1);
    // PB1 -> TIM3_CH4, AF2
    gpio_set_af(GPIOB, 1, 2);
    // PB8 -> TIM4_CH3, AF2
    gpio_set_af(GPIOB, 8, 2);

    // ============================================================
    // TIM3: CH3 (PB0) + CH4 (PB1)
    // ============================================================
    TIM3->CR1 = 0;              // Stop timer
    TIM3->PSC = PWM_PSC;
    TIM3->ARR = PWM_ARR;

    // CH3: PWM mode 1, preload enable
    // CCMR2 bits [6:4] = OC3M = 110 (PWM mode 1), bit 3 = OC3PE
    TIM3->CCMR2 &= ~(0xFF);
    TIM3->CCMR2 |= (6U << 4) | (1U << 3);  // OC3M=110, OC3PE=1

    // CH4: PWM mode 1, preload enable
    // CCMR2 bits [14:12] = OC4M = 110, bit 11 = OC4PE
    TIM3->CCMR2 &= ~(0xFF00);
    TIM3->CCMR2 |= (6U << 12) | (1U << 11);  // OC4M=110, OC4PE=1

    TIM3->CCR3 = _min_us;       // Initial pulse = 1000us
    TIM3->CCR4 = _min_us;

    // Enable CH3 and CH4 output
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Auto-reload preload, start timer
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;    // Force update to load PSC/ARR
    TIM3->CR1 |= TIM_CR1_CEN;

    // ============================================================
    // TIM1: CH4 (PA11) - Advanced timer, needs MOE
    // ============================================================
    TIM1->CR1 = 0;
    TIM1->PSC = PWM_PSC;
    TIM1->ARR = PWM_ARR;

    // CH4: PWM mode 1, preload enable (CCMR2 high byte)
    TIM1->CCMR2 &= ~(0xFF00);
    TIM1->CCMR2 |= (6U << 12) | (1U << 11);  // OC4M=110, OC4PE=1

    TIM1->CCR4 = _min_us;

    // Enable CH4 output
    TIM1->CCER |= TIM_CCER_CC4E;

    // Advanced timer: enable Main Output Enable (MOE)
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;

    // ============================================================
    // TIM4: CH3 (PB8)
    // ============================================================
    TIM4->CR1 = 0;
    TIM4->PSC = PWM_PSC;
    TIM4->ARR = PWM_ARR;

    // CH3: PWM mode 1, preload enable
    TIM4->CCMR2 &= ~(0xFF);
    TIM4->CCMR2 |= (6U << 4) | (1U << 3);  // OC3M=110, OC3PE=1

    TIM4->CCR3 = _min_us;

    TIM4->CCER |= TIM_CCER_CC3E;

    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

// ============================================================
// setChannelUs() - Write CCR value for a channel
// ============================================================

void QuadPWM::setChannelUs(uint8_t channel, uint16_t us)
{
    switch (channel) {
    case 0: TIM3->CCR3 = us; break;  // PB0
    case 1: TIM1->CCR4 = us; break;  // PA11
    case 2: TIM3->CCR4 = us; break;  // PB1
    case 3: TIM4->CCR3 = us; break;  // PB8
    }
}

// ============================================================
// begin()
// ============================================================

bool QuadPWM::begin(PwmProtocol protocol)
{
    _protocol = protocol;
    _min_us = 1000;
    _max_us = 2000;

    initTimerPwm();

    for (int i = 0; i < QUAD_PWM_NUM_CHANNELS; i++) {
        _current_us[i] = _min_us;
    }

    _initialized = true;
    return true;
}

// ============================================================
// Throttle helpers
// ============================================================

uint16_t QuadPWM::throttleToPulse(float t) const
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return _min_us + (uint16_t)(t * (float)(_max_us - _min_us));
}

// ============================================================
// write() / writeAll()
// ============================================================

void QuadPWM::write(uint8_t channel, float throttle)
{
    if (!_initialized || channel >= QUAD_PWM_NUM_CHANNELS)
        return;

    uint16_t us = throttleToPulse(throttle);
    setChannelUs(channel, us);
    _current_us[channel] = us;
}

void QuadPWM::writeAll(float m1, float m2, float m3, float m4)
{
    write(0, m1);
    write(1, m2);
    write(2, m3);
    write(3, m4);
}

// ============================================================
// disarm()
// ============================================================

void QuadPWM::disarm()
{
    if (!_initialized)
        return;

    for (int i = 0; i < QUAD_PWM_NUM_CHANNELS; i++) {
        setChannelUs(i, _min_us);
        _current_us[i] = _min_us;
    }
}

// ============================================================
// writeMicroseconds()
// ============================================================

void QuadPWM::writeMicroseconds(uint8_t channel, uint16_t us)
{
    if (!_initialized || channel >= QUAD_PWM_NUM_CHANNELS)
        return;

    if (us < _min_us) us = _min_us;
    if (us > _max_us) us = _max_us;

    setChannelUs(channel, us);
    _current_us[channel] = us;
}

// ============================================================
// Query helpers
// ============================================================

PwmProtocol QuadPWM::getProtocol() const
{
    return _protocol;
}

uint16_t QuadPWM::getPulseWidth(uint8_t channel) const
{
    if (channel >= QUAD_PWM_NUM_CHANNELS)
        return 0;
    return _current_us[channel];
}
