#include <Arduino.h>
#include "config.h"
#include "uart.h"
#include "pwm.h"
#include "control.h"
#include "comandos.h"

//---------- Control PID de velocidad de un motor ----------//
// Actuador : MOSFET en D9 (Timer1, PWM 15,6 kHz)
// Sensor   : AS5600 por I2C (SDA = A4, SCL = A5) -> velocidad en rpm
// Terminal : 115200 baud. Con '?' se ve la ayuda.
//
// Sintonía (punto 3 de la consigna):
//   1) t <u0> <u1>  -> ensayo al escalón: K, tau, theta + ZN y CHR  (a z | a c)
//   2) o ZN lazo cerrado: i 0, d 0, subir p hasta oscilar -> z <Ku> <Tu> (a k)
//   3) ajuste fino manual con p / i / d mientras se mira el log (t_ms,sp,rpm,u_pct)
// Requiere timer.c / timer.h (los tuyos, sin cambios).
int main()
{
    config_IO();
    config_timer0(); // base de tiempo de 1 ms -> get_time()
    uart_init();
    pwm_init();
    sei();
    control_init(); // I2C + AS5600 + PID

    while (1)
    {
        asm("NOP"); // ILUVIT
        comandos_tarea();
        control_tarea();
    }
}
