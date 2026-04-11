/*
 * dBounce.h
 *
 * Created: 8/4/2026 21:26:52
 *  Author: matia
 */ 

#ifndef DBONCE_H_
#define DBONCE_H_

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t pressed_count;
    uint8_t prev_state;
    bool confirmed_press;
    void (*onPress)(void);
    void (*onRelease)(void);
} debounce_t;

/**
 * @param btn Puntero a la estructura.
 * @param PIN_REG Dirección del registro PINx (ej: &PINB).
 * @param PIN_MASK Máscara del pin (ej: (1 << PB0)).
 */
void Debounce(debounce_t *btn, volatile uint8_t *PIN_REG, uint8_t PIN_MASK);

#endif