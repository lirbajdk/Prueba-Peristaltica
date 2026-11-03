#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define I2C_FREC 400000UL
#define I2C_OK 0
#define I2C_ERR_TIMEOUT 1
#define I2C_ERR_START 2
#define I2C_ERR_NACK 3

void i2c_init(void);
// lee n bytes desde el registro reg del dispositivo dir (7 bits)
uint8_t i2c_leer_reg(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif
