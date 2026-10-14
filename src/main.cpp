#include <Wire.h> // Única librería estándar obligatoria para el hardware I2C
#include <Arduino.h>


// Definiciones de hardware según el Infineon AS5600 Datasheet
const uint8_t AS5600_ADDRESS = 0x36;  // Dirección I2C fija del sensor
const uint8_t REG_RAW_ANGLE_MSB = 0x0C; // Registro del byte alto del ángulo

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Inicializa el bus I2C en los pines por defecto (A4/A5 en Uno)
    
    Serial.println("--- Probando AS5600 sin librerías externas ---");
}

void loop() {
    // 1. Apuntar al registro que queremos leer (0x0C)
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(REG_RAW_ANGLE_MSB);
    if (Wire.endTransmission() != 0) {
        Serial.println("Error: No se detecta el AS5600. Revisa conexiones.");
        delay(1000);
        return;
    }

    // 2. Solicitar los 2 bytes consecutivos (0x0C y 0x0D)
    Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);
    
    if (Wire.available() >= 2) {
        // Leer el byte alto y el byte bajo
        uint8_t c_alta = Wire.read();
        uint8_t c_baja = Wire.read();

        // 3. C++ Puro: Unir los bytes usando desplazamiento de bits (Bitwise operators)
        // Desplazamos el byte alto 8 posiciones y sumamos el byte bajo.
        // Aplicamos una máscara de 12 bits (0x0FFF) por seguridad.
        uint16_t posicion_cruda = ((uint16_t)c_alta << 8) | c_baja;
        posicion_cruda &= 0x0FFF; 

        // 4. Conversión matemática a Grados (0 a 360°)
        float grados = (posicion_cruda * 360.0) / 4096.0;

        // Imprimir resultados en consola
        Serial.print("Valor Crudo (0-4095): ");
        Serial.print(posicion_cruda);
        Serial.print("  |  Ángulo: ");
        Serial.print(grados, 2);
        Serial.println("°");
    }

    delay(200); // Muestreo cada 200 milisegundos
}
