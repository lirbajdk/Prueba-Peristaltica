#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

//--------------------//
#ifndef set_bit
#define set_bit(SFR, bit) SFR |= (1 << bit)
#define clear_bit(SFR, bit) SFR &= ~(1 << bit)
#endif
//--------------------//

//-----------PINES-----------//
#define MOSFET_PIN PB1 // D9 = OC1A (Timer1 PWM)
// SDA = A4 (PC4), SCL = A5 (PC5) -> TWI por hardware

//--------TEMPORIZACIÓN-------//
#define TS_PID_MS 20 // período del lazo PID [ms]
#define TS_PID_S (TS_PID_MS / 1000.0f)
#define TS_SENSOR_MS 2 // período de lectura del AS5600 [ms]
// OJO: entre dos lecturas el eje no puede girar más de media vuelta:
// rpm_max = 30000 / TS_SENSOR_MS  (2 ms -> 15000 rpm)

//----------SENSOR-----------//
#define SENTIDO_SENSOR 1         // poner -1 si la velocidad da negativa
#define RPM_FILTRO_ALFA 0.5f     // filtro de la medición (1.0 = sin filtro)
#define RPM_SP_MAX 10000.0f      // máximo setpoint aceptado [rpm]
#define RPM_MAX_SEGURIDAD 12000.0f // si se pasa, corta el motor [rpm]
#define MAX_ERRORES_SENSOR 10    // errores I2C seguidos -> falla

//------------PID------------//
#define U_MIN 0.0f   // salida mínima [%]
#define U_MAX 100.0f // salida máxima [%]
#define PID_TF_S 0.05f // constante del filtro de la derivada [s]
#define KP_INICIAL 0.02f // [% / rpm]
#define KI_INICIAL 0.0f  // [% / (rpm*s)]
#define KD_INICIAL 0.0f  // [% * s / rpm]

void config_timer0(void);
void config_IO(void);
uint32_t tiempo_ms(void); // get_time() leído de forma atómica

#ifdef __cplusplus
}
#endif

#endif
