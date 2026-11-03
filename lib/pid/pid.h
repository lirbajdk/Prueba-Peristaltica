#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

// u = Kp*e + Ki*Integral(e dt) + Kd*de/dt     (forma paralela)
typedef struct
{
    float kp, ki, kd;
    float ts;            // período de muestreo [s]
    float tf;            // constante del filtro de la derivada [s]
    float out_min, out_max;
    float i_term;        // integral ya multiplicada por Ki
    float pv_ant;
    float d_filt;        // derivada de la medición filtrada
    float p_term, d_term, out;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float ts, float out_min, float out_max);
void pid_set_ganancias(pid_ctrl_t *p, float kp, float ki, float kd);
void pid_set_filtro_d(pid_ctrl_t *p, float tf);
void pid_reset(pid_ctrl_t *p, float pv, float out_actual); // arranque sin salto
float pid_calcular(pid_ctrl_t *p, float sp, float pv);

#ifdef __cplusplus
}
#endif

#endif
