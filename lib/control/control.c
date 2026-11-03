#include "control.h"
#include "config.h"
#include "uart.h"
#include "pwm.h"
#include "as5600.h"
#include "pid.h"
#include <math.h>

//-------ENSAYO AL ESCALÓN-------//
#define ENSAYO_N 250                         // muestras (250 * 20 ms = 5 s)
#define ENS_VENTANA_TICKS (500 / TS_PID_MS)  // cada 0,5 s revisa si se estabilizó
#define ENS_VENTANAS_MAX 20                  // espera máxima 10 s

typedef enum
{
    ENS_ESPERA,
    ENS_ESCALON
} ens_fase_t;

static struct
{
    ens_fase_t fase;
    float u0, u1, pv_ref;
    uint16_t cnt, ventanas, n;
    int16_t y[ENSAYO_N];
    ident_t id;
} ens;

//-------------ESTADO-------------//
static ctrl_modo_t modo = CTRL_PARADO;
static pid_ctrl_t pid;
static ganancias_t sug_zn, sug_chr, sug_zc;
static uint8_t sug_ok = 0; // bit0 = z, bit1 = c, bit2 = k
static float sp = 0.0f, pv = 0.0f, u_actual = 0.0f, u_manual = 0.0f;
static uint8_t log_on = 1;
static uint8_t errores_sensor = 0;
static uint32_t pos_ant, t_ant, t_inicio_log;
static uint32_t t_pid_prox, t_sensor_prox;

//------------AUXILIARES------------//
// true cuando venció el período; si el programa se atrasó, resincroniza
static uint8_t vencio(uint32_t *prox, uint16_t periodo)
{
    uint32_t ahora = tiempo_ms();
    if ((int32_t)(ahora - *prox) < 0)
        return 0;
    *prox += periodo;
    if ((int32_t)(ahora - *prox) >= 0)
        *prox = ahora + periodo;
    return 1;
}

static void esperar_ms(uint16_t ms)
{
    uint32_t t = tiempo_ms();
    while ((uint32_t)(tiempo_ms() - t) < ms)
        ;
}

static void imprimir_gan(const ganancias_t *g)
{
    UART_PRINT("Kp=");
    uart_put_f(g->kp, 5);
    UART_PRINT(" Ki=");
    uart_put_f(g->ki, 5);
    UART_PRINT(" Kd=");
    uart_put_f(g->kd, 5);
    UART_PRINT("\r\n");
}

static void entrar_modo(ctrl_modo_t m)
{
    modo = m;
    t_inicio_log = tiempo_ms();
    if (log_on && (m == CTRL_PID || m == CTRL_MANUAL || m == CTRL_ENSAYO))
        UART_PRINT("t_ms,sp,rpm,u_pct\r\n");
}

static void declarar_falla(const char *msg_P)
{
    if (modo == CTRL_FALLA)
        return;
    modo = CTRL_FALLA;
    pwm_set(0);
    u_actual = 0.0f;
    UART_PRINT("# FALLA: ");
    uart_puts_P(msg_P);
    UART_PRINT(" (x para rearmar)\r\n");
}

static uint8_t leer_sensor(void)
{
    if (as5600_actualizar() == 0)
    {
        errores_sensor = 0;
        return 1;
    }
    if (++errores_sensor >= MAX_ERRORES_SENSOR)
        declarar_falla(PSTR("sensor AS5600"));
    return 0;
}

static void log_linea(uint32_t ahora)
{
    uart_put_i((int32_t)(ahora - t_inicio_log));
    uart_putc(',');
    uart_put_f(modo == CTRL_PID ? sp : 0.0f, 1);
    uart_putc(',');
    uart_put_f(pv, 1);
    uart_putc(',');
    uart_put_f(u_actual, 1);
    UART_PRINT("\r\n");
}

//------------ENSAYO------------//
static void ensayo_reportar(void)
{
    uint8_t r = sint_identificar(ens.y, ENSAYO_N, TS_PID_S, ens.u1 - ens.u0, &ens.id);
    UART_PRINT("# --- ENSAYO AL ESCALON ---\r\n");
    if (r == SINT_ERR_DATOS || r == SINT_ERR_SIN_RESP)
    {
        UART_PRINT("# ERROR: el motor casi no respondio, subir el escalon\r\n");
        return;
    }
    if (r == SINT_ERR_SIGNO)
    {
        UART_PRINT("# ERROR: ganancia negativa, invertir SENTIDO_SENSOR\r\n");
        return;
    }
    if (r == SINT_AVISO_NO_ESTAC)
        UART_PRINT("# AVISO: no llego al estacionario, resultados dudosos\r\n");

    UART_PRINT("# K=");
    uart_put_f(ens.id.K, 3);
    UART_PRINT(" rpm/%  tau=");
    uart_put_f(ens.id.tau, 3);
    UART_PRINT(" s  theta=");
    uart_put_f(ens.id.theta, 3);
    UART_PRINT(" s\r\n");

    sint_zn_abierto(&ens.id, &sug_zn);
    sint_chr_0(&ens.id, &sug_chr);
    sug_ok |= 0x03;
    UART_PRINT("# ZN  : ");
    imprimir_gan(&sug_zn);
    UART_PRINT("# CHR : ");
    imprimir_gan(&sug_chr);
    UART_PRINT("# usar: a z | a c\r\n");
}

static float ensayo_tick(void)
{
    if (ens.fase == ENS_ESPERA)
    {
        if (++ens.cnt >= ENS_VENTANA_TICKS)
        {
            ens.cnt = 0;
            float tol = 0.01f * fabsf(pv);
            if (tol < 2.0f)
                tol = 2.0f;
            uint8_t estable = (ens.ventanas > 0) && (fabsf(pv - ens.pv_ref) <= tol);
            ens.pv_ref = pv;
            ens.ventanas++;
            if (estable || ens.ventanas >= ENS_VENTANAS_MAX)
            {
                ens.fase = ENS_ESCALON;
                ens.n = 0;
            }
        }
        return ens.u0;
    }

    // el primer y[0] es la velocidad justo antes de aplicar el escalón
    ens.y[ens.n++] = (int16_t)(pv < 0.0f ? 0.0f : pv);
    if (ens.n >= ENSAYO_N)
    {
        ensayo_reportar();
        entrar_modo(CTRL_PARADO);
        return 0.0f;
    }
    return ens.u1;
}

//--------------LAZO--------------//
static void tick_pid(void)
{
    uint32_t ahora = tiempo_ms();
    if (!leer_sensor())
        return;

    // velocidad = cuentas recorridas / tiempo real transcurrido
    uint32_t pos = as5600_posicion();
    uint32_t dt = ahora - t_ant;
    if (dt == 0)
        return;
    float rpm = (float)(int32_t)(pos - pos_ant) * (60000.0f / AS5600_CUENTAS) / (float)dt;
    pos_ant = pos;
    t_ant = ahora;
    pv += RPM_FILTRO_ALFA * (rpm - pv);

    if (fabsf(pv) > RPM_MAX_SEGURIDAD)
        declarar_falla(PSTR("sobrevelocidad"));

    float u = 0.0f;
    switch (modo)
    {
    case CTRL_MANUAL:
        u = u_manual;
        break;
    case CTRL_PID:
        u = pid_calcular(&pid, sp, pv);
        break;
    case CTRL_ENSAYO:
        u = ensayo_tick();
        break;
    default: // PARADO / FALLA
        u = 0.0f;
        break;
    }
    u_actual = u;
    pwm_set_porcentaje(u);

    if (log_on && (modo == CTRL_PID || modo == CTRL_MANUAL || modo == CTRL_ENSAYO))
        log_linea(ahora);
}

void control_tarea(void)
{
    if (vencio(&t_pid_prox, TS_PID_MS))
        tick_pid();
    if (vencio(&t_sensor_prox, TS_SENSOR_MS))
        leer_sensor();
}

//--------------API--------------//
void control_init(void)
{
    pid_init(&pid, KP_INICIAL, KI_INICIAL, KD_INICIAL, TS_PID_S, U_MIN, U_MAX);
    pid_set_filtro_d(&pid, PID_TF_S);

    UART_PRINT("# PID de velocidad - Nano + AS5600 + MOSFET D9\r\n");
    uint8_t e = 1;
    for (uint8_t i = 0; i < 5 && e; i++)
    {
        e = as5600_init();
        if (e)
            esperar_ms(10);
    }
    if (e)
        declarar_falla(PSTR("AS5600 no responde / sin iman"));
    else
    {
        UART_PRINT("# AS5600 OK\r\n");
        if (as5600_estado() & AS5600_ML)
            UART_PRINT("# AVISO: iman debil\r\n");
        if (as5600_estado() & AS5600_MH)
            UART_PRINT("# AVISO: iman muy fuerte\r\n");
    }
    pos_ant = as5600_posicion();
    t_ant = tiempo_ms();
    t_pid_prox = t_ant + TS_PID_MS;
    t_sensor_prox = t_ant + TS_SENSOR_MS;
    UART_PRINT("# ? = ayuda\r\n");
}

void control_parar(void)
{
    errores_sensor = 0;
    u_manual = 0.0f;
    u_actual = 0.0f;
    pwm_set(0);
    entrar_modo(CTRL_PARADO);
}

uint8_t control_iniciar_pid(void)
{
    if (modo == CTRL_FALLA)
        return 0;
    pid_reset(&pid, pv, u_actual); // arranca sin salto
    entrar_modo(CTRL_PID);
    return 1;
}

uint8_t control_set_sp(float rpm)
{
    if (rpm < 0.0f || rpm > RPM_SP_MAX)
        return 0;
    sp = rpm;
    return 1;
}

uint8_t control_manual(float pct)
{
    if (modo == CTRL_FALLA || pct < U_MIN || pct > U_MAX)
        return 0;
    u_manual = pct;
    if (modo != CTRL_MANUAL)
        entrar_modo(CTRL_MANUAL);
    return 1;
}

uint8_t control_ensayo(float u0, float u1)
{
    if (modo == CTRL_FALLA || u0 < U_MIN || u1 > U_MAX || u1 < U_MIN || u0 > U_MAX || u0 == u1)
        return 0;
    ens.fase = ENS_ESPERA;
    ens.u0 = u0;
    ens.u1 = u1;
    ens.cnt = 0;
    ens.ventanas = 0;
    ens.pv_ref = 0.0f;
    ens.n = 0;
    entrar_modo(CTRL_ENSAYO);
    return 1;
}

void control_set_ganancias(const ganancias_t *g)
{
    pid_set_ganancias(&pid, g->kp, g->ki, g->kd);
}

void control_get_ganancias(ganancias_t *g)
{
    g->kp = pid.kp;
    g->ki = pid.ki;
    g->kd = pid.kd;
}

void control_calcular_zn_cerrado(float ku, float tu)
{
    sint_zn_cerrado(ku, tu, &sug_zc);
    sug_ok |= 0x04;
    UART_PRINT("# ZN cerrado: ");
    imprimir_gan(&sug_zc);
    UART_PRINT("# usar: a k\r\n");
}

uint8_t control_aplicar_sugerencia(char tipo)
{
    const ganancias_t *g = 0;
    if (tipo == 'z' && (sug_ok & 0x01))
        g = &sug_zn;
    else if (tipo == 'c' && (sug_ok & 0x02))
        g = &sug_chr;
    else if (tipo == 'k' && (sug_ok & 0x04))
        g = &sug_zc;
    if (!g)
        return 0;
    control_set_ganancias(g);
    return 1;
}

uint8_t control_log_toggle(void)
{
    log_on = !log_on;
    return log_on;
}

void control_estado(void)
{
    UART_PRINT("# modo=");
    uart_put_i(modo); // 0 parado, 1 manual, 2 pid, 3 ensayo, 4 falla
    UART_PRINT(" sp=");
    uart_put_f(sp, 1);
    UART_PRINT(" rpm=");
    uart_put_f(pv, 1);
    UART_PRINT(" u=");
    uart_put_f(u_actual, 1);
    UART_PRINT("%\r\n# ");
    ganancias_t g;
    control_get_ganancias(&g);
    imprimir_gan(&g);
}
