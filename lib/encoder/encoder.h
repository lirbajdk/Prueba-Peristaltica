#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

// Encoder rotativo de cuadratura (tipo KY-040 / EC11)
//   A (CLK) -> D2 = PD2 (INT0)
//   B (DT)  -> D3 = PD3 (INT1)
//   común   -> GND
// Configuración (sentido, transiciones por paso) en config.h

void encoder_init(void);
int8_t encoder_leer_pasos(void); // pasos (detents) girados desde la última lectura, con signo

#ifdef __cplusplus
}
#endif

#endif
