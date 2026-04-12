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
ISR(TIMER1_COMPA_vect){
	// Las variables 'static' mantienen su valor en memoria RAM entre cada llamada a la interrupción.
	// isr_state: Contador de la Máquina de Estados (0=Servo0-ON, 1=Servo0-OFF, 2=Servo1-ON, etc.)
	static uint8_t isr_state = 0;

	// cycle_ticks: Acumulador de tiempo. Registra cuántos "ticks" (pasos de 0.5us)
	// se van consumiendo para poder calcular el tiempo de espera final y lograr los 20ms exactos.
	static uint16_t cycle_ticks = 0;

	// Verificación de seguridad. Si el puntero o el arreglo están vacíos, aborta para evitar un Hard Fault.
	if (servo_count == 0) return;

	// Mapeo de estado a índice de motor. Como cada servo usa 2 estados (uno para ON y otro para OFF),
	// la división entera determina el índice. (Ej: estados 0 y 1 / 2 = Servo 0).
	uint8_t servo_idx = isr_state / 2;

	// Condición principal: ¿Aún quedan servos por pulsar?
	// Para 3 servos, servo_count * 2 = 6. Procesará los estados del 0 al 5 aquí adentro.
	if (isr_state < (servo_count * 2)) {
		
		// Operación bit a bit (AND 0x01). Evalúa el Bit 0 del número. Si es 0, el estado es PAR.
		// Los estados PARES (0, 2, 4) inician el PWM de cada motor.
		if ((isr_state & 0x01) == 0) {
			
			// ESTADO PAR: Encender Pin
			// Operación lógica OR absoluta a nivel de bits. Fija el pin correspondiente en ALTO (1).
			*(servo_array[servo_idx]->port) |= (1 << servo_array[servo_idx]->pin);
			
			// Lee el ancho de pulso requerido (ej: 3000 ticks para 1.5ms) según el ángulo configurado.
			uint16_t ticks = servo_array[servo_idx]->target_pulse_ticks;
			
			// Programación de hardware (Timer 1 Modo Normal): Suma el ancho de pulso al registro comparador.
			// Esto le indica al silicio que vuelva a disparar esta ISR *exactamente* cuando el pulso deba terminar.
			OCR1A += ticks;
			
			// Contabiliza el tiempo gastado en el acumulador del ciclo.
			cycle_ticks += ticks;
			
			} else {
			// ESTADO IMPAR: Apagar Pin (estados 1, 3, 5)
			// Operación lógica AND-NOT absoluta a nivel de bits. Fija el pin en BAJO (0), terminando el pulso.
			*(servo_array[servo_idx]->port) &= ~(1 << servo_array[servo_idx]->pin);
			
			// Dead Time (Tiempo Muerto). Suma 100 ticks (50us) al comparador.
			// Retrasa el encendido del siguiente servo para evitar solapamientos y picos de corriente inductiva en la fuente.
			OCR1A += 100;
			
			// Contabiliza el tiempo muerto.
			cycle_ticks += 100;
		}
		// Avanza la FSM al siguiente paso lógico.
		isr_state++;
		
		} else {
		// ESTADO DE ESPERA (Sincronización del Período de 50Hz)
		// Ya se enviaron los pulsos a todos los servos. Ahora el microcontrolador debe esperar
		// sin hacer nada hasta completar la ventana obligatoria de 20ms (CYCLE_TICKS = 40000 ticks).
		
		if (cycle_ticks < CYCLE_TICKS) {
			// Calcula el tiempo remanente (20ms - tiempo gastado en pulsos) y programa el salto final del Timer.
			// Esto garantiza un periodo de repetición matemáticamente perfecto de 50Hz.
			OCR1A += (CYCLE_TICKS - cycle_ticks);
			} else {
			// Failsafe: Si por una anomalía aritmética el tiempo acumulado superó los 20ms,
			// fuerza una interrupción inmediata corta (50us) para no desbordar y bloquear el sistema.
			OCR1A += 100;
		}
		
		// Resetea la FSM y el acumulador para iniciar un nuevo marco de 20ms comenzando desde el Servo 0.
		isr_state = 0;
		cycle_ticks = 0;
	}
}