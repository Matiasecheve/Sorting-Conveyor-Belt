#ifndef TCRT5000_H_
#define TCRT5000_H_

#include <stdbool.h>
#include <avr/io.h>
#include <stdint.h>

/* Modos de operación del sensor */
typedef enum {
	TCRT_DIGITAL, // Para leer el pin D0 del módulo PCB
	TCRT_ANALOG   // Para leer el sensor suelto o el pin A0 usando el ADC
} TCRT_Mode_t;

/* Estructura del objeto TCRT5000 */
typedef struct {
	TCRT_Mode_t mode;
	
	/* Configuración Digital (Solo se usa si mode == TCRT_DIGITAL) */
	volatile uint8_t *pin_reg; // Ej: &PIND
	uint8_t pin_num;           // Ej: PD2
	
	/* Configuración Analógica (Solo se usa si mode == TCRT_ANALOG) */
	uint8_t adc_channel;       // Canal del ADC (0 a 7, correspondiente a PC0-PC7)
	
} TCRT5000_t;

/* Prototipos de funciones */
void TCRT5000_Init(TCRT5000_t *sensor);
uint8_t TCRT5000_ReadDigital(TCRT5000_t *sensor);
uint16_t TCRT5000_ReadAnalog(TCRT5000_t *sensor);

/* Prototipo opcional: Inicializa el hardware del ADC del ATmega328P */
void TCRT5000_InitADC(void);

#endif /* TCRT5000_H_ */