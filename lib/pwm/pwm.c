#include "pwm.h"
#include "config.h"

// Timer1, Fast PWM 10 bits (modo 7), prescaler 1 -> 16 MHz / 1024 = 15,6 kHz en D9 (OC1A)
void pwm_init(void)
{
    TCCR1A = (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS10);
    OCR1A = 0;
}

void pwm_set(uint16_t duty)
{
    if (duty > 1023)
        duty = 1023;
    if (duty == 0)
    {
        // OCR=0 en fast PWM deja un pico: se desconecta el pin y se fuerza a 0
        clear_bit(TCCR1A, COM1A1);
        clear_bit(PORTB, MOSFET_PIN);
    }
    else
    {
        OCR1A = duty;
        set_bit(TCCR1A, COM1A1); // PWM no invertido
    }
}

void pwm_set_porcentaje(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;
    pwm_set((uint16_t)(pct * 10.23f + 0.5f));
}
