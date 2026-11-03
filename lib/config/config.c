#include "config.h"
#include "timer.h"

void config_timer0(void)
{
    TCCR0A = 0b00000010; // modo CTC
    TIMSK0 = 0b00000010; // habilitar interrupción de comparación A
    OCR0A = 249;         // 16 MHz / 64 / 250 = 1 kHz -> 1 ms
    TCCR0B = 0b00000011; // prescaler 64
}

void config_IO(void)
{
    set_bit(DDRB, MOSFET_PIN);   // D9 salida
    clear_bit(PORTB, MOSFET_PIN); // motor apagado
}

// get_time() es de 32 bits y el micro de 8: hay que leerlo sin que la ISR lo cambie
uint32_t tiempo_ms(void)
{
    uint8_t sreg = SREG;
    cli();
    uint32_t t = get_time();
    SREG = sreg;
    return t;
}
