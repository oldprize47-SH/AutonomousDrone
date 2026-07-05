//
// USART3 Stream wrapper for Arduino UNO Q (STM32U585)
// Register-level control: PA7(TX) / PA5(RX) / AF7
// RX: IRQ + ring buffer, TX: IRQ + ring buffer
//

#ifndef USART3_SERIAL_H
#define USART3_SERIAL_H

#include <Arduino.h>
#include <stm32u5xx.h>

#define USART3_RX_BUF_SIZE 2048
#define USART3_TX_BUF_SIZE 2048

class Usart3Serial : public Stream {
public:
    void begin(uint32_t baud);
    void end();

    int available() override;
    int read() override;
    int peek() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buf, size_t size) override;
    void flush() override;

    // Called from ISR - do not call directly
    void _irqHandler();

private:
    volatile uint8_t  _rxBuf[USART3_RX_BUF_SIZE];
    volatile uint16_t _rxHead = 0;
    volatile uint16_t _rxTail = 0;
    volatile uint8_t  _txBuf[USART3_TX_BUF_SIZE];
    volatile uint16_t _txHead = 0;
    volatile uint16_t _txTail = 0;
    bool _begun = false;
};

#endif // USART3_SERIAL_H
