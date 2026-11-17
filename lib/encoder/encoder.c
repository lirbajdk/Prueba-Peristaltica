/*********************************************************************************************************
 ****------INCLUDES-----
*********************************************************************************************************/
    #include <Arduino.h>
    #include "config.h"   // tiempo_ms(), RPM_SP_MAX y parámetros ENC_*
    #include "encoder.h"

/*********************************************************************************************************
 ------Interrupción por cambio en Puerto D (PCINT20-21)------
*********************************************************************************************************/
    // En lugar de una bandera (0 = antihorario, 1 = horario, 2 = nada) se acumulan
    // las transiciones con signo: así no se pierden pasos si se gira rápido, y un
    // rebote (horario, antihorario, horario...) se cancela solo.
    ////////////////////////////////////////////////////////
        static volatile int8_t encoder_transiciones = 0; // dentro del paso actual
        static volatile int8_t encoder_pasos = 0;        // pasos completos sin leer
        static volatile uint8_t encoder_prev_state = 0;
    ////////////////////////////////////////////////////////
    ISR(PCINT2_vect)
    {
        uint8_t current_state = read_encoder_pins();
        uint8_t state_change = (encoder_prev_state << 2) | current_state;

        // Detectar transición
        if (current_state != encoder_prev_state)
        {
            // Máquina de estados para decodificar el encoder
            switch (state_change)
            {
            case 0b0001: // 00 -> 01: giro horario
            case 0b0111: // 01 -> 11: giro horario
            case 0b1110: // 11 -> 10: giro horario
            case 0b1000: // 10 -> 00: giro horario
                encoder_transiciones++;
                break;

            case 0b0010: // 00 -> 10: giro antihorario
            case 0b1011: // 10 -> 11: giro antihorario
            case 0b1101: // 11 -> 01: giro antihorario
            case 0b0100: // 01 -> 00: giro antihorario
                encoder_transiciones--;
                break;

            default: // cambiaron los dos pines a la vez (se perdió un estado): se ignora
                break;
            }
            encoder_prev_state = current_state;

            // En la posición de reposo (el "clic") se decide si hubo un paso completo.
            // Con pull-ups el reposo es 11; los encoders de 2 transiciones por paso
            // también reposan en 00.
#if ENC_TRANSICIONES_POR_PASO == 2
            if (current_state == 0b11 || current_state == 0b00)
#else
            if (current_state == 0b11)
#endif
            {
                if (encoder_transiciones >= ENC_TRANSICIONES_POR_PASO / 2 && encoder_pasos < 127)
                    encoder_pasos++;
                else if (encoder_transiciones <= -(ENC_TRANSICIONES_POR_PASO / 2) && encoder_pasos > -127)
                    encoder_pasos--;
                encoder_transiciones = 0;
            }
        }
    }

/*********************************************************************************************************
 ------GETER PARA ENCODER----- (devuelve los pasos girados y los borra)
*********************************************************************************************************/
    int8_t get_ENCODER(void)
    {
        uint8_t sreg = SREG;
        cli();
        int8_t pasos = encoder_pasos;
        encoder_pasos = 0;
        SREG = sreg;
        return (int8_t)(pasos * ENC_SENTIDO);
    }

/*********************************************************************************************************
 ------Leer estado actual de los pines del encoder-----  (bit0 = A, bit1 = B)
*********************************************************************************************************/
    uint8_t read_encoder_pins(void)
    {
        return (PIND & (ENCODER_A_MASK | ENCODER_B_MASK)) >> ENCODER_A;
    }

/*********************************************************************************************************
 ------Inicializar pines del encoder-----
*********************************************************************************************************/
    void encoder_init(void)
    {
        // Configurar pines como entrada con pull-up
        DDRD &= ~(ENCODER_A_MASK | ENCODER_B_MASK);
        PORTD |= (ENCODER_A_MASK | ENCODER_B_MASK);
    }

/*********************************************************************************************************
 ------Inicializar interrupciones PCINT para Puerto D-----
*********************************************************************************************************/
    void pcint_init(void)
    {
        encoder_prev_state = read_encoder_pins(); // arranca desde la posición real

        // Habilitar interrupciones para Puerto D (PCINT16–23)
        PCICR |= (1 << PCIE2);

        // Configurar máscara para los pines específicos
        PCMSK2 |= (1 << PCINT20); // PD4 (ENCODER_A)
        PCMSK2 |= (1 << PCINT21); // PD5 (ENCODER_B)
        // PCMSK2 |= (1 << PCINT22); // PD6 (botón - opcional)

        PCIFR = (1 << PCIF2); // borra un pedido viejo
        // sei() lo hace main() una vez que está todo configurado
    }

/*********************************************************************************************************
 ****------FUNCIONES-----
*********************************************************************************************************/
    /***************************************************************
    *! ------GRUPO_SP-----GRUPO_SP-----GRUPO_SP-----GRUPO_SP-----
    ***************************************************************/
        /*******************************************************
         ------LECTOR DE SENTIDO DE GIRO Y AUMENTADOR-----
        *******************************************************/
            /////////////////////////////
                static uint16_t valor_sp = 0; // [rpm]
            /////////////////////////////
            void sistema_sube_baja_sp(int8_t snt_g, uint16_t paso) // snt_g = sentido de giro
            {
                if (snt_g > 0) // giro horario
                {
                    if (valor_sp < LIMITE_SUPERIOR_SP - paso) // limite max
                        valor_sp += paso;
                    else
                        valor_sp = LIMITE_SUPERIOR_SP;
                }
                else if (snt_g < 0) // giro antihorario
                {
                    if (valor_sp > LIMITE_INFERIOR_SP + paso) // limite min
                        valor_sp -= paso;
                    else
                        valor_sp = LIMITE_INFERIOR_SP;
                }
            }

        /*******************************************************
         ------FUCNION MANDADA PARA ROTACIONES------
        *******************************************************/
            // Paso chico girando despacio, paso grande si los clics vienen seguidos
            uint16_t enco_get_sp(void)
            {
                int8_t giro = get_ENCODER();
                static uint32_t tiempo = 0; // momento del último paso

                if (giro != 0)
                {
                    uint32_t ahora = tiempo_ms();
                    uint16_t paso = ((uint32_t)(ahora - tiempo) < ENC_T_RAPIDO_MS) ? ENC_PASO_RAPIDO_RPM : ENC_PASO_RPM;
                    tiempo = ahora;

                    for (; giro > 0; giro--)
                        sistema_sube_baja_sp(1, paso); // giro horario
                    for (; giro < 0; giro++)
                        sistema_sube_baja_sp(-1, paso); // giro antihorario
                }
                return valor_sp;
            }

        /*******************************************************
         ------FUCNION PA SETEAR/REINICIAR------
        *******************************************************/
            void enco_set_sp(uint16_t set)
            {
                if (set > LIMITE_SUPERIOR_SP)
                    set = LIMITE_SUPERIOR_SP;
                valor_sp = set;
            }
