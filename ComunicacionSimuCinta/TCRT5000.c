#include "TCRT5000.h"

/*
 * Inicializa los pines según el modo elegido en la estructura.
 */
void TCRT5000_Init(TCRT5000_t *sensor) {
    if (sensor->mode == TCRT_DIGITAL) {
        // Configuramos el pin digital como ENTRADA
        // Para saber qué DDR es, hacemos aritmética de punteros (DDR = PIN + 1 en AVR)
        volatile uint8_t *ddr_reg = sensor->pin_reg + 1;
        *ddr_reg &= ~(1 << sensor->pin_num);
        
        // Opcional: Activar resistencia Pull-Up interna
        // volatile uint8_t *port_reg = sensor->pin_reg + 2;
        // *port_reg |= (1 << sensor->pin_num);
    } 
    else if (sensor->mode == TCRT_ANALOG) {
        // Los pines analógicos (PC0-PC5) como entrada
        DDRC &= ~(1 << sensor->adc_channel);
        
        // Deshabilitar el buffer digital para ahorrar energía en ese pin
        DIDR0 |= (1 << sensor->adc_channel);
    }
}

/*
 * Retorna el estado digital del módulo PCB (0 o 1).
 * Nota: Los módulos con LM393 suelen dar 0 (LOW) cuando detectan un objeto (reflejo).
 */
uint8_t TCRT5000_ReadDigital(TCRT5000_t *sensor) {
    if (sensor->mode != TCRT_DIGITAL) return 0;
    
    if (*(sensor->pin_reg) & (1 << sensor->pin_num)) {
        return 1; // Pin en ALTO
    } else {
        return 0; // Pin en BAJO
    }
}

/*
 * Realiza una conversión analógica a digital (ADC).
 * Ideal para el sensor suelto, devuelve un valor de 0 a 1023.
 * 0 = Blanco/Objeto muy cerca (mucha reflexión IR)
 * 1023 = Negro/Nada enfrente (nula reflexión IR)
 */
uint16_t TCRT5000_ReadAnalog(TCRT5000_t *sensor) {
    if (sensor->mode != TCRT_ANALOG) return 0;

    // Seleccionar el canal ADC (asegurando limpiar los bits previos)
    ADMUX = (ADMUX & 0xF0) | (sensor->adc_channel & 0x0F);

    // Iniciar la conversión
    ADCSRA |= (1 << ADSC);

    // Esperar a que la conversión termine (el bit ADSC vuelve a 0)
    while (ADCSRA & (1 << ADSC));

    // Retornar el valor de 10 bits
    return ADC;
}

/*
 * Función de conveniencia para arrancar el módulo ADC del ATmega328P
 * Se llama UNA SOLA VEZ en el main() si vas a usar sensores analógicos.
 */
void TCRT5000_InitADC(void) {
    // Voltaje de referencia = AVCC (5V)
    ADMUX = (1 << REFS0);
    
    // Habilitar ADC y establecer prescaler a 128 (16MHz / 128 = 125kHz, óptimo para el ADC)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}