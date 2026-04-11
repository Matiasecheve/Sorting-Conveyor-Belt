#include "HCSR04.h"

void HCSR04_Process(HCSR04_Config_t *config) {
	switch (config->state) {
		case HCSR_IDLE:
		break;

		case HCSR_TRIG_START:
			config->trigger_write(1);
			config->t_ref = config->get_us();
			config->state = HCSR_TRIG_WAIT;
		break;

		case HCSR_TRIG_WAIT:
			if ((config->get_us() - config->t_ref) >= 10) { // Pasaron los 10us
				config->trigger_write(0);
				config->state = HCSR_ECHO_WAIT_HIGH;
			}
		break;

		case HCSR_ECHO_WAIT_HIGH:
			if (config->echo_read() == 1) {
				config->t_ref = config->get_us();
				config->state = HCSR_ECHO_WAIT_LOW;
			}
		break;

		case HCSR_ECHO_WAIT_LOW:
			if (config->echo_read() == 0){
				uint32_t t_vuelo = config->get_us() - config->t_ref;
				config->distancia = (uint16_t)(t_vuelo / 58);
				config->state = HCSR_DATA_READY;
			}
		break;

		case HCSR_DATA_READY:
		// Espera a que el main lea el dato y resetee a IDLE
		break;
	}
}