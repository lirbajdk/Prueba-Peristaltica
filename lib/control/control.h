#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include "sintonia.h"

typedef enum
{
    CTRL_PARADO,
    CTRL_MANUAL, // lazo abierto
    CTRL_PID,    // lazo cerrado
    CTRL_ENSAYO, // escalón para identificar la planta
    CTRL_FALLA
} ctrl_modo_t;

void control_init(void);
void control_tarea(void); // llamar en el while(1)

void control_parar(void); // también borra una falla
uint8_t control_iniciar_pid(void);
uint8_t control_set_sp(float rpm);
uint8_t control_manual(float pct);
uint8_t control_ensayo(float u0, float u1);
void control_set_ganancias(const ganancias_t *g);
void control_get_ganancias(ganancias_t *g);
void control_calcular_zn_cerrado(float ku, float tu);
uint8_t control_aplicar_sugerencia(char tipo); // 'z', 'c' o 'k'
uint8_t control_log_toggle(void);
void control_estado(void);

#ifdef __cplusplus
}
#endif

#endif
