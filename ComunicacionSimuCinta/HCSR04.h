#ifndef HCSR04_H_
#define HCSR04_H_

#include <stdint.h>

/* Estados de la FSM del sensor */
typedef enum {
    HCSR_IDLE,
    HCSR_TRIG_START,
    HCSR_TRIG_WAIT,
    HCSR_ECHO_WAIT_HIGH,
    HCSR_ECHO_WAIT_LOW,
    HCSR_DATA_READY
} hcsr_state_t;

typedef void (*hcsr_write_ptr)(uint8_t state);
typedef uint8_t (*hcsr_read_ptr)(void);
typedef uint32_t (*hcsr_get_time_ptr)(void);

typedef struct {
    hcsr_write_ptr    trigger_write;
    hcsr_read_ptr     echo_read;
    hcsr_get_time_ptr get_us;
    /* Variables internas de la FSM */
    hcsr_state_t      state;
    uint32_t          t_ref;
    uint16_t          distancia;
} HCSR04_Config_t;

void HCSR04_Process(HCSR04_Config_t *config);

#endif