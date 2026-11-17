#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************************************************
 ****------INCLUDES-----
*********************************************************************************************************/

    #include <Arduino.h>

/*********************************************************************************************************
 ****------DEFINES-----
*********************************************************************************************************/
    // Definición de pines del encoder (común del encoder a GND)
    #define ENCODER_A PD4   // D4 (PCINT20)
    #define ENCODER_B PD5   // D5 (PCINT21)  -> tiene que ser el pin siguiente a ENCODER_A
    //#define ENCODER_BTN PD6 // D6 (PCINT22) - opcional

    // Máscaras para los pines del encoder
    #define ENCODER_A_MASK (1 << ENCODER_A)
    #define ENCODER_B_MASK (1 << ENCODER_B)
    //#define ENCODER_BTN_MASK (1 << ENCODER_BTN)

    // Límites del setpoint [rpm]
    #define LIMITE_SUPERIOR_SP ((uint16_t)RPM_SP_MAX)
    #define LIMITE_INFERIOR_SP 0

/*********************************************************************************************************
 ****------PROTOTIPOS DE FUNCIONES-----
*********************************************************************************************************/
    /*****************************************************
     ****------FUCNIONES Q CONFIGURAN-----
    *****************************************************/
        void encoder_init(void);
        void pcint_init(void);
        uint8_t read_encoder_pins(void);
        int8_t get_ENCODER(void); // pasos girados desde la última lectura (+ horario, - antihorario)

    /*****************************************************
     ****------FUCIONES RELACIONADAS AL GIRO-----
    *****************************************************/
        /*************************************************
         ****------GRUPO SETPOINT-----
        *************************************************/
            void sistema_sube_baja_sp(int8_t snt_g, uint16_t paso);
            uint16_t enco_get_sp(void);   // procesa el giro y devuelve el setpoint [rpm]
            void enco_set_sp(uint16_t sp); // para sincronizar con el comando 's'

#ifdef __cplusplus
}
#endif

#endif
