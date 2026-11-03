#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include <avr/pgmspace.h>

#define UART_BAUD 115200UL

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_puts_P(const char *s); // string en flash
void uart_put_i(int32_t v);
void uart_put_f(float v, uint8_t decimales);
int16_t uart_getc(void); // -1 si no hay datos

#define UART_PRINT(s) uart_puts_P(PSTR(s))

#ifdef __cplusplus
}
#endif

#endif
