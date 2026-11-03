#include "uart.h"
#include <stdlib.h>

#define TX_SIZE 128 // potencias de 2
#define RX_SIZE 32
#define UART_UBRR ((F_CPU + 4UL * UART_BAUD) / (8UL * UART_BAUD) - 1UL)

static volatile uint8_t tx_buf[TX_SIZE];
static volatile uint8_t tx_head = 0, tx_tail = 0;
static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

void uart_init(void)
{
    UCSR0A = (1 << U2X0);
    UBRR0H = (uint8_t)(UART_UBRR >> 8);
    UBRR0L = (uint8_t)(UART_UBRR & 0xFF);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

ISR(USART_UDRE_vect)
{
    if (tx_head == tx_tail)
    {
        UCSR0B &= ~(1 << UDRIE0); // nada más para enviar
        return;
    }
    UDR0 = tx_buf[tx_tail];
    tx_tail = (tx_tail + 1) & (TX_SIZE - 1);
}

ISR(USART_RX_vect)
{
    uint8_t d = UDR0;
    uint8_t sig = (rx_head + 1) & (RX_SIZE - 1);
    if (sig != rx_tail)
    {
        rx_buf[rx_head] = d;
        rx_head = sig;
    }
}

void uart_putc(char c)
{
    uint8_t sig = (tx_head + 1) & (TX_SIZE - 1);
    while (sig == tx_tail)
        ; // buffer lleno: espera a que la ISR lo vacíe
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = sig;
    UCSR0B |= (1 << UDRIE0);
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

void uart_puts_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++)))
        uart_putc(c);
}

void uart_put_i(int32_t v)
{
    char b[14];
    ltoa(v, b, 10);
    uart_puts(b);
}

void uart_put_f(float v, uint8_t decimales)
{
    char b[16];
    if (v > 99999.0f)
        v = 99999.0f;
    if (v < -99999.0f)
        v = -99999.0f;
    dtostrf(v, 1, decimales, b);
    uart_puts(b);
}

int16_t uart_getc(void)
{
    if (rx_head == rx_tail)
        return -1;
    uint8_t d = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_SIZE - 1);
    return d;
}
