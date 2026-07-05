//
// USART1 Stream wrapper implementation
// D1=PB6(TX) / D0=PB7(RX) / AF7 (JDIGITAL header)
//

#include "Usart1Serial.h"
#include "hw_config.h"
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

// Allow swapping TX/RX pins in hardware if wiring is reversed (USART1 CR2.SWAP).
#ifndef IMU_UART_SWAP_RXTX
#define IMU_UART_SWAP_RXTX 0
#endif

// Singleton pointer for ISR access
static Usart1Serial *_usart1_instance = nullptr;

// Global ISR (C style)
extern "C" void USART1_IRQHandler(const void *arg) {
    if (_usart1_instance) {
        _usart1_instance->_irqHandler();
    } else {
        USART1->ICR = 0xFFFFFFFF;
        volatile uint32_t dummy = USART1->RDR;
        (void)dummy;
    }
}

// ISR member function
void Usart1Serial::_irqHandler()
{
    uint32_t isr = USART1->ISR;

    // Error handling: ORE, FE, NE
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART1->ICR |= (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF);
        volatile uint32_t dummy = USART1->RDR;
        (void)dummy;
        return;
    }

    // Normal receive (RXNE)
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)(USART1->RDR & 0xFF);
        uint16_t next = (_rxHead + 1) % USART1_RX_BUF_SIZE;
        if (next != _rxTail) {
            _rxBuf[_rxHead] = c;
            _rxHead = next;
        }
    }
	// TX interrupt
	if (isr & USART_ISR_TXE_TXFNF) {

   	 	// 보낼 데이터 존재
    	if (_txHead != _txTail) {

			USART1->TDR = _txBuf[_txTail];

       	  	_txTail = (_txTail + 1) % USART1_TX_BUF_SIZE;
   	 	}
   	 	else {
        	// 전송 완료 -> TX interrupt disable
        	USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    	}
	}
}

void Usart1Serial::begin(uint32_t baud)
{
    if (_begun) return;
    _usart1_instance = this;
    _rxHead = 0;
    _rxTail = 0;
	_txHead = 0;
	_txTail = 0;

    // 1. Enable GPIOB clock
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;

    // 2. Enable USART1 clock (APB2)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    volatile uint32_t dummy = RCC->APB2ENR;
    (void)dummy;

    // 3. Configure PB6 = D1 (TX) as AF7
    GPIOB->MODER   &= ~(3U << (6 * 2));
    GPIOB->MODER   |=  (2U << (6 * 2));   // AF mode
    GPIOB->OSPEEDR |=  (3U << (6 * 2));   // Very high speed
    GPIOB->PUPDR   &= ~(3U << (6 * 2));
    GPIOB->PUPDR   |=  (1U << (6 * 2));   // Pull-up
    GPIOB->AFR[0]  &= ~(0xFU << (6 * 4));
    GPIOB->AFR[0]  |=  (7U   << (6 * 4)); // AF7

    // 4. Configure PB7 = D0 (RX) as AF7
    GPIOB->MODER   &= ~(3U << (7 * 2));
    GPIOB->MODER   |=  (2U << (7 * 2));   // AF mode
    GPIOB->PUPDR   &= ~(3U << (7 * 2));
    GPIOB->PUPDR   |=  (1U << (7 * 2));   // Pull-up
    GPIOB->AFR[0]  &= ~(0xFU << (7 * 4));
    GPIOB->AFR[0]  |=  (7U   << (7 * 4)); // AF7

    // 5. Configure USART1 (must be done while UE=0)
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

#if IMU_UART_SWAP_RXTX
    // Swap TX/RX internally if board wiring is reversed (PB6<->PB7)
    USART1->CR2 |= USART_CR2_SWAP;
#endif

	// 추가
	USART1->ICR = 0xFFFFFFFF;

    // BRR: APB2 clock = 160MHz
    USART1->BRR = 160000000UL / baud;

    // [ORDER CRITICAL] Connect + enable the NVIC IRQ line BEFORE the USART can
    // generate any interrupt (see Usart3Serial::begin for the full rationale).
    irq_connect_dynamic(USART1_IRQn, 6, USART1_IRQHandler, NULL, 0);
    irq_enable(USART1_IRQn);

    // Flush any stale RX data so the first real RXNE is clean.
    (void)USART1->RDR;
    USART1->ICR = 0xFFFFFFFF;

    // Now enable TX, RX, RXNE interrupt, and the USART itself.
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

    _begun = true;
}

void Usart1Serial::end()
{
    if (!_begun) return;

	// 추가
	flush();

    irq_disable(USART1_IRQn);
    USART1->CR1 = 0;
    _begun = false;
    _usart1_instance = nullptr;
}

int Usart1Serial::available()
{
    return (USART1_RX_BUF_SIZE + _rxHead - _rxTail) % USART1_RX_BUF_SIZE;
}

int Usart1Serial::read()
{
    if (_rxHead == _rxTail) return -1;
    uint8_t c = _rxBuf[_rxTail];
    _rxTail = (_rxTail + 1) % USART1_RX_BUF_SIZE;
    return c;
}

int Usart1Serial::peek()
{
    if (_rxHead == _rxTail) return -1;
    return _rxBuf[_rxTail];
}

size_t Usart1Serial::write(uint8_t c)
{
    uint16_t next = (_txHead + 1) % USART1_TX_BUF_SIZE;

    // TX buffer full
    if (next == _txTail) {
        return 0;
    }

    _txBuf[_txHead] = c;
    _txHead = next;

    // TX interrupt enable
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;

    return 1;
}

size_t Usart1Serial::write(const uint8_t *buf, size_t size)
{
    size_t written = 0;

    for (size_t i = 0; i < size; i++) {

        if (write(buf[i])) {
            written++;
        }
        else {
            break;
        }
    }

    return written;
}

void Usart1Serial::flush()
{

	uint32_t start = millis();

    while (_txHead != _txTail) {

        if ((millis() - start) > 100) {
            break;
        }
    }
    start = millis();

    while (!(USART1->ISR & USART_ISR_TC)) {

        if ((millis() - start) > 100) {
            break;
        }
    }
}
