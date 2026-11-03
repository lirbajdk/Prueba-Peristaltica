#ifndef SINTONIA_H
#define SINTONIA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define SINT_OK 0
#define SINT_AVISO_NO_ESTAC 1 // calculó, pero la respuesta no llegó al estacionario
#define SINT_ERR_DATOS 2
#define SINT_ERR_SIN_RESP 3   // casi no respondió al escalón
#define SINT_ERR_SIGNO 4      // ganancia negativa: revisar SENTIDO_SENSOR

typedef struct
{
    float kp, ki, kd;
} ganancias_t;

// Modelo de 1er orden con retardo: G(s) = K e^(-theta s) / (tau s + 1)
typedef struct
{
    float K;     // [rpm / %]
    float tau;   // [s]
    float theta; // [s]
    float y0, yf;
} ident_t;

// y[0] = valor antes del escalón; y[i] a t = i*ts. delta_u = salto de la entrada [%]
uint8_t sint_identificar(const int16_t *y, uint16_t n, float ts, float delta_u, ident_t *r);

void sint_zn_abierto(const ident_t *r, ganancias_t *g);  // Ziegler-Nichols curva de reacción
void sint_chr_0(const ident_t *r, ganancias_t *g);       // Chien-Hrones-Reswick, 0 % sobreimpulso
void sint_zn_cerrado(float ku, float tu, ganancias_t *g); // Ziegler-Nichols oscilación sostenida
void sint_desde_ti_td(float kp, float ti, float td, ganancias_t *g); // Ki = Kp/Ti, Kd = Kp*Td

#ifdef __cplusplus
}
#endif

#endif
