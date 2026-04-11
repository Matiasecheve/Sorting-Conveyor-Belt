#ifndef SG90_H_
#define SG90_H_

#include <stdint.h>
#include <avr/io.h>

typedef struct {
	volatile uint8_t *port;
	uint8_t pin;
	uint16_t target_pulse_ticks;
} SG90_t;

void SG90_Init(SG90_t *servo, volatile uint8_t *port, uint8_t pin);
void SG90_SetAngle(SG90_t *servo, uint8_t angle);

#endif