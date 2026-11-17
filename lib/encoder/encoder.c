#include "encoder.h"
#include "config.h"

// Basado en Practicas_con_encoder: INT0 en PD2 (A) e INT1 en PD3 (B).
//
// Cambio respecto de la práctica: allá cada flanco ascendente sumaba o restaba
// según el otro canal, y un rebote en A (con B quieto) contaba varias veces en el
// mismo sentido (de ahí el "angle / 2"). Acá las dos interrupciones se disparan
// con cualquier cambio y se decodifica con una tabla de estados de cuadratura:
// un rebote produce +1 -1 +1 -1... y se cancela solo. Los saltos inválidos
// (dos bits cambian a la vez) valen 0.

#define ENC_AB() ((uint8_t)((PIND >> PD2) & 0x03)) // bit0 = A (PD2), bit1 = B (PD3)

// índice = (estado_anterior << 2) | estado_nuevo
// mismo sentido que la práctica: A sube con B en alto -> -1 ; B sube con A en alto -> +1
static const int8_t tabla[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static volatile uint8_t ab_ant;
static volatile int8_t transiciones = 0; // dentro del paso actual
static volatile int8_t pasos = 0;        // pasos completos sin leer

static void decodificar(void)
{
    uint8_t ab = ENC_AB();
    transiciones += tabla[(uint8_t)((ab_ant << 2) | ab)];
    ab_ant = ab;

    // En la posición de reposo (detent) se decide si hubo un paso completo.
    // Con 4 transiciones por paso el reposo es A=B=1; con 2, también A=B=0.
#if ENC_TRANSICIONES_POR_PASO == 2
    if (ab == 0x03 || ab == 0x00)
#else
    if (ab == 0x03)
#endif
    {
        if (transiciones >= ENC_TRANSICIONES_POR_PASO / 2 && pasos < 127)
            pasos++;
        else if (transiciones <= -(ENC_TRANSICIONES_POR_PASO / 2) && pasos > -127)
            pasos--;
        transiciones = 0;
    }
}

ISR(INT0_vect) { decodificar(); } // cambió PD2 (A)
ISR(INT1_vect) { decodificar(); } // cambió PD3 (B)

void encoder_init(void)
{
    clear_bit(DDRD, PD2); // entradas
    clear_bit(DDRD, PD3);
    set_bit(PORTD, PD2); // pull-ups internos (el módulo KY-040 ya trae los suyos)
    set_bit(PORTD, PD3);

    ab_ant = ENC_AB();
    EICRA = (EICRA & ~((1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10))) |
            (1 << ISC00) | (1 << ISC10); // INT0 e INT1: cualquier cambio de nivel
    EIFR = (1 << INTF0) | (1 << INTF1);  // borra flags viejos
    EIMSK |= (1 << INT0) | (1 << INT1);
}

int8_t encoder_leer_pasos(void)
{
    uint8_t sreg = SREG;
    cli();
    int8_t p = pasos;
    pasos = 0;
    SREG = sreg;
    return (int8_t)(p * ENC_SENTIDO);
}
