#include "pid.h"

static float limitar(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float ts, float out_min, float out_max)
{
    p->ts = ts;
    p->tf = 5.0f * ts;
    p->out_min = out_min;
    p->out_max = out_max;
    pid_set_ganancias(p, kp, ki, kd);
    pid_reset(p, 0.0f, 0.0f);
}

void pid_set_ganancias(pid_ctrl_t *p, float kp, float ki, float kd)
{
    // i_term acumula Ki*e*dt, por eso cambiar Ki en caliente no produce saltos
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void pid_set_filtro_d(pid_ctrl_t *p, float tf)
{
    p->tf = tf;
}

void pid_reset(pid_ctrl_t *p, float pv, float out_actual)
{
    p->i_term = limitar(out_actual, p->out_min, p->out_max);
    p->pv_ant = pv;
    p->d_filt = 0.0f;
    p->p_term = 0.0f;
    p->d_term = 0.0f;
    p->out = out_actual;
}

float pid_calcular(pid_ctrl_t *p, float sp, float pv)
{
    float e = sp - pv;

    // P: actúa sobre el error presente
    p->p_term = p->kp * e;

    // D: sobre la medición (con SP constante de/dt = -dpv/dt). Evita el golpe
    // derivativo al cambiar el SP. Filtro pasa-bajos de 1er orden contra el ruido.
    float dpv = (pv - p->pv_ant) / p->ts;
    p->pv_ant = pv;
    p->d_filt += (p->ts / (p->tf + p->ts)) * ((-dpv) - p->d_filt);
    p->d_term = p->kd * p->d_filt;

    // I: acumula el error pasado. Anti-windup: si la salida satura y el error
    // sigue empujando hacia afuera, no se integra.
    float i_nuevo = p->i_term + p->ki * e * p->ts;
    float u = p->p_term + i_nuevo + p->d_term;
    if (u > p->out_max)
    {
        u = p->out_max;
        if (e > 0.0f)
            i_nuevo = p->i_term;
    }
    else if (u < p->out_min)
    {
        u = p->out_min;
        if (e < 0.0f)
            i_nuevo = p->i_term;
    }
    p->i_term = limitar(i_nuevo, p->out_min, p->out_max);
    p->out = u;
    return u;
}
