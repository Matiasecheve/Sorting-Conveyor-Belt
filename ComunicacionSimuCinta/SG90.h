#ifndef SG90_H_
#define SG90_H_

#include <stdint.h>

typedef void (*toggle_ptr_t)(volatile uint8_t*, uint8_t);

typedef enum {
	SG90_IDLE,
	SG90_PULSE_HIGH,
	SG90_PULSE_LOW
} sg90_state_t;

typedef struct {
	volatile uint8_t *port;       // Dirección del puerto (ej: &PORTC)
	uint8_t pin;                  // Pin físico
	toggle_ptr_t toggle_func;     // Función de Toggle
	volatile uint16_t *tcnt_reg;  // Puntero directo al registro del hardware (TCNT1)
	uint16_t target_pulse_ticks;  // Ancho de pulso en ticks
	uint16_t last_tick;           // Marca temporal (16 bits)
	sg90_state_t state;
} SG90_t;

void SG90_Init(SG90_t *servo, volatile uint8_t *port, uint8_t pin, toggle_ptr_t t_func, volatile uint16_t *timer_reg);
void SG90_SetAngle(SG90_t *servo, uint8_t angle);
void SG90_Process(SG90_t *servo);

#endif