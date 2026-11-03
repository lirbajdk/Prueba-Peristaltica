#include "sintonia.h"
#include <math.h>

#define DY_MIN 10.0f // mínimo cambio de velocidad para considerar válido el ensayo [rpm]

// instante en que la respuesta cruza y0 + frac*dy (interpolación lineal); -1 si no cruza
static float cruce(const int16_t *y, uint16_t n, float y0, float dy, float frac, float ts)
{
    float obj = y0 + frac * dy;
    for (uint16_t i = 1; i < n; i++)
    {
        float a = y[i - 1], b = y[i];
        if ((dy > 0.0f && b >= obj) || (dy < 0.0f && b <= obj))
        {
            float den = b - a;
            float f = (den != 0.0f) ? (obj - a) / den : 1.0f;
            return ((float)(i - 1) + f) * ts;
        }
    }
    return -1.0f;
}

uint8_t sint_identificar(const int16_t *y, uint16_t n, float ts, float delta_u, ident_t *r)
{
    if (n < 40 || delta_u == 0.0f)
        return SINT_ERR_DATOS;

    float yf = 0.0f, yp = 0.0f;
    for (uint8_t i = 0; i < 10; i++)
    {
        yf += y[n - 1 - i];
        yp += y[n - 11 - i];
    }
    yf /= 10.0f;
    yp /= 10.0f;

    float y0 = y[0];
    float dy = yf - y0;
    r->y0 = y0;
    r->yf = yf;
    if (fabsf(dy) < DY_MIN)
        return SINT_ERR_SIN_RESP;
    r->K = dy / delta_u;
    if (r->K <= 0.0f)
        return SINT_ERR_SIGNO;

    // Método de dos puntos (Smith): t en 28,3 % y 63,2 % de la respuesta
    float t28 = cruce(y, n, y0, dy, 0.283f, ts);
    float t63 = cruce(y, n, y0, dy, 0.632f, ts);
    if (t28 < 0.0f || t63 < 0.0f)
        return SINT_ERR_SIN_RESP;

    r->tau = 1.5f * (t63 - t28);
    r->theta = t63 - r->tau;
    if (r->tau < ts)
        r->tau = ts;
    if (r->theta < ts) // el lazo digital no puede ver menos de una muestra de retardo
        r->theta = ts;

    return (fabsf(yf - yp) <= 0.02f * fabsf(dy)) ? SINT_OK : SINT_AVISO_NO_ESTAC;
}

void sint_desde_ti_td(float kp, float ti, float td, ganancias_t *g)
{
    g->kp = kp;
    g->ki = (ti > 0.0f) ? kp / ti : 0.0f;
    g->kd = kp * td;
}

void sint_zn_abierto(const ident_t *r, ganancias_t *g)
{
    float kp = 1.2f * r->tau / (r->K * r->theta);
    sint_desde_ti_td(kp, 2.0f * r->theta, 0.5f * r->theta, g);
}

void sint_chr_0(const ident_t *r, ganancias_t *g)
{
    float kp = 0.6f * r->tau / (r->K * r->theta);
    sint_desde_ti_td(kp, r->tau, 0.5f * r->theta, g);
}

void sint_zn_cerrado(float ku, float tu, ganancias_t *g)
{
    sint_desde_ti_td(0.6f * ku, 0.5f * tu, 0.125f * tu, g);
}
