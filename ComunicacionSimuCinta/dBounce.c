/*
 * dBounce.c
 *
 * Created: 8/4/2026 21:25:51
 *  Author: matia
 */ 

#include "dbounce.h"
#include <stddef.h>

void Debounce(debounce_t *btn, volatile uint8_t *PIN_REG, uint8_t PIN_MASK) {
	// Lectura del registro de 8 bits
	uint8_t current_state = (*PIN_REG & PIN_MASK) ? 1 : 0;

	if (current_state == btn->prev_state) {
		if (btn->pressed_count < 9) {
			btn->pressed_count++;
		}
		else if (btn->pressed_count == 9) {
			// Estado estable alcanzado
			if (current_state == 0) { // Botón presionado (Pull-up)
				btn->confirmed_press = true;
				if (btn->onPress != NULL) btn->onPress();
			}
			else { // Botón liberado
				btn->confirmed_press = false;
				if (btn->onRelease != NULL) btn->onRelease();
			}
			btn->pressed_count++; // Evita re-disparar
		}
	}
	else {
		btn->pressed_count = 0;
		btn->prev_state = current_state;
	}
}