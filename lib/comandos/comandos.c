#include "comandos.h"
#include "control.h"
#include "uart.h"
#include <stdlib.h>

#define LINEA_MAX 32

static void ok(void) { UART_PRINT("# OK\r\n"); }
static void err(void) { UART_PRINT("# ERROR\r\n"); }

static void ayuda(void)
{
    UART_PRINT("# p/i/d <v>   Kp, Ki, Kd\r\n");
    UART_PRINT("# s <rpm>     setpoint\r\n");
    UART_PRINT("# g            PID ON (lazo cerrado)\r\n");
    UART_PRINT("# x            parar / rearmar\r\n");
    UART_PRINT("# m <%>        salida manual (lazo abierto)\r\n");
    UART_PRINT("# t <u0> <u1>  ensayo al escalon u0->u1 [%]\r\n");
    UART_PRINT("# z <Ku> <Tu>  Ziegler-Nichols lazo cerrado\r\n");
    UART_PRINT("# a <z|c|k>    aplicar ganancias sugeridas\r\n");
    UART_PRINT("# l            log ON/OFF\r\n");
}

// lee un float del texto; avanza el puntero
static uint8_t leer_f(char **p, float *v)
{
    char *fin;
    float x = (float)strtod(*p, &fin);
    if (fin == *p)
        return 0;
    *v = x;
    *p = fin;
    return 1;
}

static void ejecutar(char *l)
{
    char c = l[0];
    if (c >= 'A' && c <= 'Z')
        c += 32;
    char *arg = l + 1;
    float a, b;
    ganancias_t g;

    switch (c)
    {
    case '?':
    case 'h':
        ayuda();
        control_estado();
        break;
    case 'p':
    case 'i':
    case 'd':
        if (!leer_f(&arg, &a) || a < 0.0f)
        {
            err();
            break;
        }
        control_get_ganancias(&g);
        if (c == 'p')
            g.kp = a;
        else if (c == 'i')
            g.ki = a;
        else
            g.kd = a;
        control_set_ganancias(&g);
        control_estado();
        break;
    case 's':
        (leer_f(&arg, &a) && control_set_sp(a)) ? ok() : err();
        break;
    case 'g':
        control_iniciar_pid() ? ok() : err();
        break;
    case 'x':
        control_parar();
        ok();
        break;
    case 'm':
        (leer_f(&arg, &a) && control_manual(a)) ? ok() : err();
        break;
    case 't':
        (leer_f(&arg, &a) && leer_f(&arg, &b) && control_ensayo(a, b)) ? ok() : err();
        break;
    case 'z':
        if (leer_f(&arg, &a) && leer_f(&arg, &b) && a > 0.0f && b > 0.0f)
            control_calcular_zn_cerrado(a, b);
        else
            err();
        break;
    case 'a':
        while (*arg == ' ')
            arg++;
        if (*arg >= 'A' && *arg <= 'Z')
            *arg += 32;
        if (control_aplicar_sugerencia(*arg))
            control_estado();
        else
            err();
        break;
    case 'l':
        control_log_toggle() ? UART_PRINT("# log ON\r\n") : UART_PRINT("# log OFF\r\n");
        break;
    default:
        err();
        break;
    }
}

void comandos_tarea(void)
{
    static char linea[LINEA_MAX];
    static uint8_t n = 0;
    int16_t c;

    while ((c = uart_getc()) >= 0)
    {
        if (c == '\r' || c == '\n')
        {
            if (n)
            {
                linea[n] = 0;
                ejecutar(linea);
                n = 0;
            }
        }
        else if (n < LINEA_MAX - 1)
            linea[n++] = (c == ',') ? '.' : (char)c; // acepta coma decimal
    }
}
