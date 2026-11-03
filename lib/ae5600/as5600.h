#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define AS5600_DIR 0x36
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_CUENTAS 4096.0f // 12 bits por vuelta

#define AS5600_MH (1 << 3) // imán muy fuerte
#define AS5600_ML (1 << 4) // imán muy débil
#define AS5600_MD (1 << 5) // imán detectado
#define AS5600_ERR_IMAN 0x10

uint8_t as5600_init(void);       // 0 = OK
uint8_t as5600_actualizar(void); // lee el ángulo y acumula la posición (0 = OK)
uint32_t as5600_posicion(void);  // cuentas acumuladas (usar la resta con signo)
uint8_t as5600_estado(void);     // último registro STATUS

#ifdef __cplusplus
}
#endif

#endif
