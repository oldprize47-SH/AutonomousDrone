//
// USART3 Stream wrapper implementation
//

#include "Usart3Serial.h"
#include <zephyr/kernel.h>  // 추가
#include <zephyr/irq.h>     // 추가

// Singleton pointer for ISR access
static Usart3Serial *_instance = nullptr;

// 1. 하드웨어가 실제로 호출하는 전역 ISR (C 스타일)
extern "C" void USART3_IRQHandler(const void *arg) {
    if (_instance) {
        _instance->_irqHandler();
    } else {
        USART3->ICR = 0xFFFFFFFF;
        volatile uint32_t dummy = USART3->RDR;
        (void)dummy;
    }
}

// 2. 실제 데이터 처리 및 에러 방어를 수행하는 클래스 멤버 함수 (C++ 스타일)
void Usart3Serial::_irqHandler()
{
    uint32_t isr = USART3->ISR;

    // [방어 코드] 오버런(ORE), 프레이밍(FE), 노이즈(NE) 에러가 발생했는지 확인
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        // 플래그 클리어 (무한 루프 프리징 방지)
        USART3->ICR |= (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF);

        // 찌꺼기 데이터가 남아있을 수 있으므로 RDR을 한 번 읽어서 허공에 버림 (안전장치)
        volatile uint32_t dummy = USART3->RDR;
        (void)dummy;

        return; // 에러가 난 바이트는 버림
    }

    // [정상 수신] 수신 버퍼에 데이터가 있을 때 (RXNE)
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)(USART3->RDR & 0xFF); // 레지스터에서 1바이트 읽기

        // Ring Buffer에 저장
        uint16_t next = (_rxHead + 1) % USART3_RX_BUF_SIZE;
        if (next != _rxTail) {
            _rxBuf[_rxHead] = c;
            _rxHead = next;
        }
        // 만약 next == _rxTail 이라면 링 버퍼가 꽉 찬 상태입니다.
        // 이때는 새로 들어온 데이터를 버려서(Drop) 소프트웨어 오버런을 방지합니다.
    }
    // [TX interrupt] 송신 가능 상태
    if (isr & USART_ISR_TXE_TXFNF) {

        // 보낼 데이터가 있으면
        if (_txHead != _txTail) {

            USART3->TDR = _txBuf[_txTail];

            _txTail = (_txTail + 1) % USART3_TX_BUF_SIZE;
        }
        else {
            // 보낼 데이터가 더 없으면 TX interrupt 비활성화
            USART3->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        }
    }
}

void Usart3Serial::begin(uint32_t baud)
{
    if (_begun) return;
    _instance = this;
    _rxHead = 0;
    _rxTail = 0;
    _txHead = 0;
    _txTail = 0;

    // 1. Enable GPIOA clock
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;

    // 2. Enable USART3 clock (APB1)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
    volatile uint32_t dummy = RCC->APB1ENR1;
    (void)dummy;

    // 3. Configure PA7 (TX) as AF7
    GPIOA->MODER   &= ~(3U << (7 * 2));
    GPIOA->MODER   |=  (2U << (7 * 2));   // AF mode
    GPIOA->OSPEEDR |=  (3U << (7 * 2));   // Very high speed
    GPIOA->PUPDR   &= ~(3U << (7 * 2));
    GPIOA->PUPDR   |=  (1U << (7 * 2));   // Pull-up
    GPIOA->AFR[0]  &= ~(0xFU << (7 * 4));
    GPIOA->AFR[0]  |=  (7U   << (7 * 4)); // AF7

    // 4. Configure PA5 (RX) as AF7
    GPIOA->MODER   &= ~(3U << (5 * 2));
    GPIOA->MODER   |=  (2U << (5 * 2));   // AF mode
    GPIOA->PUPDR   &= ~(3U << (5 * 2));
    GPIOA->PUPDR   |=  (1U << (5 * 2));   // Pull-up
    GPIOA->AFR[0]  &= ~(0xFU << (5 * 4));
    GPIOA->AFR[0]  |=  (7U   << (5 * 4)); // AF7

    // 5. Configure USART3
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;

    USART3->ICR = 0xFFFFFFFF;

    // BRR: APB1 clock = 160MHz
    USART3->BRR = 160000000UL / baud;

    // [ORDER CRITICAL] Connect + enable the NVIC IRQ line BEFORE the USART can
    // generate any interrupt. Otherwise, if the peer (GCS) is already sending,
    // enabling UE|RXNEIE below latches a pending IRQ with no handler attached
    // yet -> hard fault at boot (LED never lights). This was the cause of the
    // intermittent "board hangs when GCS is running" symptom.
    irq_connect_dynamic(USART3_IRQn, 6, USART3_IRQHandler, NULL, 0);
    irq_enable(USART3_IRQn);

    // Flush any stale RX data so the first real RXNE is clean.
    (void)USART3->RDR;
    USART3->ICR = 0xFFFFFFFF;

    // Now enable TX, RX, RXNE interrupt, and the USART itself.
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

    _begun = true;
}

void Usart3Serial::end()
{
    if (!_begun) return;

    // 추가
    flush();

    irq_disable(USART3_IRQn);
    USART3->CR1 = 0;
    _begun = false;
    _instance = nullptr;
}

int Usart3Serial::available()
{
    return (USART3_RX_BUF_SIZE + _rxHead - _rxTail) % USART3_RX_BUF_SIZE;
}

int Usart3Serial::read()
{
    if (_rxHead == _rxTail) return -1;
    uint8_t c = _rxBuf[_rxTail];
    _rxTail = (_rxTail + 1) % USART3_RX_BUF_SIZE;
    return c;
}

int Usart3Serial::peek()
{
    if (_rxHead == _rxTail) return -1;
    return _rxBuf[_rxTail];
}

//size_t Usart3Serial::write(uint8_t c)
//{
//    while (!(USART3->ISR & USART_ISR_TXE_TXFNF)) {}
//    USART3->TDR = c;
//    return 1;
//}

size_t Usart3Serial::write(uint8_t c)
{
    uint16_t next = (_txHead + 1) % USART3_TX_BUF_SIZE;

    // TX buffer full
    if (next == _txTail) {
        return 0;
    }

    _txBuf[_txHead] = c;
    _txHead = next;

    // TX interrupt enable
    USART3->CR1 |= USART_CR1_TXEIE_TXFNFIE;

    return 1;
}


//size_t Usart3Serial::write(const uint8_t *buf, size_t size)
//{
//   for (size_t i = 0; i < size; i++) {
//        write(buf[i]);
//    }
//    return size;
//}

size_t Usart3Serial::write(const uint8_t *buf, size_t size)
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

//void Usart3Serial::flush()
//{
//
//    uint32_t start = millis();
//
//    while (_txHead != _txTail) {
//
//        if ((millis() - start) > 100) {
//            break;
//        }
//    }
//
//    while (!(USART3->ISR & USART_ISR_TC)) {}
//}

void Usart3Serial::flush()
{
    uint32_t start = millis();

    while (_txHead != _txTail) {

        if ((millis() - start) > 100) {
            break;
        }
    }

    start = millis();

    while (!(USART3->ISR & USART_ISR_TC)) {

        if ((millis() - start) > 100) {
            break;
        }
    }
}