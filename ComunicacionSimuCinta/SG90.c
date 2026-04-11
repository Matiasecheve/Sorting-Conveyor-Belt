#include "SG90.h"
#include <avr/interrupt.h>

#define MAX_SERVOS 3
#define CYCLE_TICKS 40000 // 20ms con prescaler 8 @ 16MHz

static SG90_t* servo_array[MAX_SERVOS];
static uint8_t servo_count = 0;

void SG90_Init(SG90_t *servo, volatile uint8_t *port, uint8_t pin) {
	servo->port = port;
	servo->pin = pin;
	servo->target_pulse_ticks = 1088; // Reposo

	*(servo->port) &= ~(1 << servo->pin); // Forzar LOW

	if (servo_count < MAX_SERVOS) {
		servo_array[servo_count] = servo;
		servo_count++;
	}

	// Si es el primer servo inicializado, encendemos el motor de interrupciones
	if (servo_count == 1) {
		TIMSK1 |= (1 << OCIE1A); // Habilitar ISR de Comparación A en Timer1
		OCR1A = TCNT1 + 100;     // Disparar la primera interrupción en 50µs
	}
}

void SG90_SetAngle(SG90_t *servo, uint8_t angle) {
	if (angle > 180) angle = 180;
	servo->target_pulse_ticks = 1088 + (((uint32_t)angle * 3712UL) / 180UL);
}

/* --- MOTOR DE PWM MULTIPLEXADO --- */
ISR(TIMER1_COMPA_vect) {
	static uint8_t isr_state = 0;
	static uint16_t cycle_ticks = 0;

	if (servo_count == 0) return;

	uint8_t servo_idx = isr_state / 2;

	if (isr_state < (servo_count * 2)) {
		if ((isr_state & 0x01) == 0) {
			// ESTADO PAR: Encender Pin
			*(servo_array[servo_idx]->port) |= (1 << servo_array[servo_idx]->pin);
			uint16_t ticks = servo_array[servo_idx]->target_pulse_ticks;
			OCR1A += ticks;
			cycle_ticks += ticks;
			} else {
			// ESTADO IMPAR: Apagar Pin
			*(servo_array[servo_idx]->port) &= ~(1 << servo_array[servo_idx]->pin);
			OCR1A += 100; // Tiempo muerto de 50µs entre servos para aislar ruido
			cycle_ticks += 100;
		}
		isr_state++;
		} else {
		// Terminó la secuencia de los 3 servos. Rellenar el resto de los 20ms.
		if (cycle_ticks < CYCLE_TICKS) {
			OCR1A += (CYCLE_TICKS - cycle_ticks);
			} else {
			OCR1A += 100; // Failsafe matemático
		}
		isr_state = 0;
		cycle_ticks = 0;
	}
}