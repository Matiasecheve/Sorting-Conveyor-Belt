#include "SG90.h"

void SG90_Init(SG90_t *servo, volatile uint8_t *port, uint8_t pin, toggle_ptr_t t_func, volatile uint16_t *timer_reg) {
	servo->port = port;
	servo->pin = pin;
	servo->toggle_func = t_func;
	servo->tcnt_reg = timer_reg; // Conectamos directo a &TCNT1

	servo->target_pulse_ticks = 1088;
	
	servo->state = SG90_IDLE;
	servo->last_tick = 0;
	
	*(servo->port) &= ~(1 << servo->pin); // Forzamos LOW para la FSM
}

void SG90_SetAngle(SG90_t *servo, uint8_t angle) {
	if (angle > 180) angle = 180;
	
	// Ecuación entera: 1088 ticks + (Angulo * Delta / 180)
	servo->target_pulse_ticks = 1088 + (((uint32_t)angle * 3712UL) / 180UL);
}

void SG90_Process(SG90_t *servo) {
	// Leemos el hardware directamente
	uint16_t now = *(servo->tcnt_reg);
	
	// Al ser enteros de 16 bits sin signo, el desbordamiento (rollover)
	// matemático se maneja solo y da el delta exacto de tiempo.
	uint16_t delta = now - servo->last_tick;

	switch (servo->state) {
		case SG90_IDLE:
		servo->toggle_func(servo->port, servo->pin); // Pasa a HIGH
		servo->last_tick = now;
		servo->state = SG90_PULSE_HIGH;
		break;

		case SG90_PULSE_HIGH:
		if (delta >= servo->target_pulse_ticks) {
			servo->toggle_func(servo->port, servo->pin); // Pasa a LOW
			servo->state = SG90_PULSE_LOW;
		}
		break;

		case SG90_PULSE_LOW:
		// 20ms = 40.000 ticks a 2MHz
		if (delta >= 40000) {
			servo->state = SG90_IDLE;
		}
		break;
	}
}