/*
 * =============================================================================
 * SISTEMA DE CLASIFICACIÓN AUTOMATIZADA DE CAJAS - ACTIVIDAD Nº3
 * Microcontroladores 2026 - Ingeniería en Mecatrónica - UNER
 * =============================================================================
 *
 * 1. PROTOCOLO DE COMUNICACIÓN (UNER)
 * -----------------------------------------------------------------------------
 * Formato: [U][N][E][R][LEN][:][CMD][PAYLOAD...][CKS]
 * - LEN: 1 byte (CMD + Payload).
 * - CKS: XOR acumulado desde 'U' hasta el último byte del payload.
 * - Endianness: Little Endian para valores de más de 1 byte.
 *
 * 2. ESTADOS DEL SISTEMA Y DEBUG (LEDs en Puerto B / Etiquetas A0-A1)
 * -----------------------------------------------------------------------------
 * LED PB5 - Estado General:
 * - ST_IDLE    (0): OFF - Esperando Handshake (0xF0).
 * - ST_READY   (1): ON Fijo - Conectado, esperando Inicio (0x50).
 * - ST_RUNNING (2): Toggle Rápido (100ms) - Clasificación en proceso.
 * - ST_ERROR   (3): Toggle Lento (500ms) - Fallo de Reset o Comunicación.
 *
 * 3. TRAMAS DE SIMULACIÓN (PC -> MICRO vía Hércules en HEX)
 * -----------------------------------------------------------------------------
 * > CONEXIÓN E INICIO:
 * - ACK Alive (F0): $55$4E$45$52$02$3A$F0$0D$C9
 * - Config (50):    $55$4E$45$52$05$3A$50$0A$06$08$0A$6D (V:1.0, S0:6, S1:8, S2:10)
 *
 * > EVENTOS DE CAJAS (CMD 5F):
 * - Caja 6cm:       $55$4E$45$52$02$3A$5F$06$6D
 * - Caja 8cm:       $55$4E$45$52$02$3A$5F$08$63
 * - Caja 10cm:      $55$4E$45$52$02$3A$5F$0A$61
 *
 * > SENSORES IR (CMD 5E - Entrada IRState=1 / Salida IRState=0):
 * - S0 (6cm):       Ent: $55$4E$45$52$03$3A$5E$00$01$6A | Sal: $55$4E$45$52$03$3A$5E$00$00$6B
 * - S1 (8cm):       Ent: $55$4E$45$52$03$3A$5E$01$01$6B | Sal: $55$4E$45$52$03$3A$5E$01$00$6A
 * - S2 (10cm):      Ent: $55$4E$45$52$03$3A$5E$02$01$68 | Sal: $55$4E$45$52$03$3A$5E$02$00$69
 *
 * 4. TEMPORIZACIÓN Y LÓGICA INTERNA
 * -----------------------------------------------------------------------------
 * - TIMER0: Base de tiempo de 2ms (Modo CTC). Gestiona tick_ms global.
 * - TIMER1: Heartbeat en PB5 (~1 Hz).
 * - COLA (FIFO): Almacena tipos de caja (6, 8, 10) según llegan (0x5F).
 * - ACTUADORES: Transición No Bloqueante.
 * 1. Recibe orden -> Pasa a ACT_EXTENDING (Envía 0x52, armPosition=1).
 * 2. Espera 150ms -> Pasa a ACT_RETRACTING (Envía 0x52, armPosition=0).
 * 3. Espera 150ms -> Pasa a ACT_IDLE (Disponible).
 * =============================================================================
 */

/* ============================================================
 * INCLUDES
 * ============================================================ */
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/*
 * Protocol_UNER.h ya incluye:
 *   - #define F_CPU 16000000UL
 *   - <avr/io.h>, <stdint.h>, <stdbool.h>, <stddef.h>
 *   - Definiciones de BUFSIZE, MASK, MAX_PAYLOAD
 *   - Enums: rx_state_t  (WAIT_U … GET_CKS)
 *   - Structs: ring_buf_t, protocol_rx_t, protocol_tx_t
 *   - Tipos: cmd_handler_t, protocol_command_t
 *   - Extern: Rx (protocol_rx_t), Tx (protocol_tx_t)
 *   - Prototipos de Protocol_Init, Protocol_HandleUART,
 *     Protocol_TxAddChar, Protocol_TxSendString,
 *     Protocol_SendSimuCMD, Protocol_DecodeCMD
 */
#include "Protocol_UNER.h"
#include "dBounce.h"
#include "SG90.h"
#include "HCSR04.h"
#include "TCRT5000.h"
/* ============================================================
 * CONSTANTES GLOBALES
 * ============================================================
 * BUFSIZE, MASK y MAX_PAYLOAD ya vienen de Protocol_UNER.h    */
#define DEBUG_FAST_MS   100     /* Periodo toggle PB5 en ST_RUNNING       */
#define MaxQueue        10      /* Maximo número de cajas en cinta        */
#define DIST_SENSOR_TO_SERVO_MM  200  // Ajustá esto a la distancia real de tu cinta
#define ANCHO_CAJA_CM      8UL
#define D0_CM              45UL
#define D1_CM              88UL
#define D2_CM              136UL
#define KICK_FIFO_SIZE     5
/* ============================================================
 * ENUMS
 * ============================================================ */

typedef enum {
    ST_IDLE,        /* Esperando conexión (enviamos 0xF0 periódicamente) */
    ST_READY,       /* Conectado, esperando START (0x50)                  */
    ST_RUNNING,     /* Clasificando cajas                                 */
    ST_ERROR        /* Fallo o Reset                                      */
} system_state_t;

/*
 * rx_state_t (WAIT_U … GET_CKS) ya está definido en Protocol_UNER.h.
 * No se redefine aquí para evitar conflictos de compilación.
 */

/* Estados del actuador */
typedef enum {
	ACT_IDLE,       /* Retraído, disponible                  */
	ACT_WAITING,    /* Esperando que la caja llegue al centro */
	ACT_EXTENDING,  /* Extendido, esperando para retraer     */
	ACT_RETRACTING  /* Retrayendo, esperando confirmación    */
} actuator_state_t;
/* ============================================================
 * ESTRUCTURAS
 * ============================================================
 * ring_buf_t, protocol_rx_t y protocol_tx_t vienen de Protocol_UNER.h.
 * Solo se definen aquí las estructuras propias de la aplicación.  */

/* Estado de cada actuador */
typedef struct {
    actuator_state_t state;
    uint32_t         timestamp_ms;   /* Momento en que comenzó el estado actual */
} _sActuator;

/*
 * Tabla de comandos — usa los tipos de Protocol_UNER.h:
 *   cmd_handler_t     ? void (*)(void)
 *   protocol_command_t ? { uint8_t cmd_id; cmd_handler_t execute; }
 * Se crea un alias local _sCommand para no cambiar ninguna referencia
 * existente en el código.
 */
typedef protocol_command_t _sCommand;

/* ============================================================
 * BANDERAS DE EVENTOS (FLAGS)
 * ============================================================ */
typedef struct {
    /* =========================================================
     * 1. MODOS DE OPERACIÓN Y CONFIGURACIÓN DEL SISTEMA
     * ========================================================= */
    volatile bool isMode;                 // 0 = Lazo Abierto (Tiempo) | 1 = Lazo Cerrado (IR)
    volatile bool manual_timeout_enabled; // 0 = Timeout Automático    | 1 = Timeout Manual
    volatile bool hw_sensors_enabled;     // 0 = Control por Qt        | 1 = Sensores físicos

    /* =========================================================
     * 2. EVENTOS DE SENSORES Y ENTRADA DE CAJAS
     * ========================================================= */
    volatile bool box_entry_active;       // Nueva caja detectada y clasificada en la entrada
    volatile bool ir0_active;             // Sensor físico Zona 0 activado
    volatile bool ir1_active;             // Sensor físico Zona 1 activado
    volatile bool ir2_active;             // Sensor físico Zona 2 activado

    /* =========================================================
     * 3. CONTROL DE TAREAS Y DESPLAZAMIENTO DE COLAS (QUEUES)
     * ========================================================= */
    volatile bool movQ0;                  // Semáforo de movimiento en Queue 0
    volatile bool movQ1;                  // Semáforo de movimiento en Queue 1
    volatile bool movQ2;                  // Semáforo de movimiento en Queue 2
    volatile bool box_entry_Q1;           // Bandera de traspaso (Sala de espera): caja pasa de Q0 a Q1
    volatile bool box_entry_Q2;           // Bandera de traspaso (Sala de espera): caja pasa de Q1 a Q2

    /* =========================================================
     * 4. RESPUESTAS TX PENDIENTES (COLA UART HACIA Qt)
     * =========================================================
     * Los callbacks RX solo levantan un flag. HandlePendingReplies
     * despacha la respuesta en el loop principal cuando RX está libre.
     */
    volatile bool reply_send_alive;       // Enviar ping/alive de estado (0xF0)
    volatile bool reply_ack_alive;        // Responder ACK al ping proveniente de Qt
    volatile bool reply_send_start;       // Confirmar arranque de la cinta (0x50)
    volatile bool reply_send_stop;        // Confirmar detención de la cinta (0x51)
    volatile bool reply_send_reset;       // Confirmar reinicio de memoria y hardware (0x53)
    volatile bool reply_send_config;      // Confirmar nueva configuración de salidas (0x40)
    volatile bool reply_send_speed_ack;   // Confirmar cambio de velocidad manual (0x54)
    volatile bool reply_send_timeout_ack; // Confirmar cambio de tiempo de servo (0x55)
    volatile bool reply_send_BeltVel;     // Enviar a Qt la velocidad calculada de la cinta
    volatile bool reply_error_start;      // Error 0x5A: Intento de arranque sin configuración
    volatile bool reply_error;            // Transitar a ST_ERROR de forma asíncrona
	volatile bool reply_send_mode_ack;    // Confirmar cambio de Modo (0x59)
	volatile bool reply_send_calib_ack;   // Confirmar calibración Ultrasónico (0x60)
	volatile bool reply_send_manual_timeout_ack;  // Confirmar cambio Manual/Auto (0x56)
	volatile bool reply_send_wait_center_ack; // Confirmar WaitCenter (0x57)
	volatile bool reply_send_act_times_ack; // Confirmar tiempos de servo (0x62)
} _sEventFlags;

/* ============================================================
 * PROTOTIPOS
 * ============================================================ */
void InitUART0(void);
void InitTimer0(void);
void InitTimer1(void);
void InitPort(void);

/*
 * TX — se mantienen los nombres internos como wrappers inline
 * que delegan en las funciones de la librería.
 */
void HandleTX(void);
void Inject_RX_Command(const uint8_t *trama, uint8_t len);
void SendText(const char* texto);
	
/* RX / Protocolo */
void DecodeCMD(uint8_t cmd);

/* Lógica de clasificación */
void FireActuator(uint8_t outNum, uint8_t extend);
void HandleActuators(void);

/* Cola */
void HandleQueue();

/* Respuestas TX diferidas */
void HandlePendingReplies(void);

/* Debug */
void UpdateDebugLEDs(void);
void DebugQueues(void);
void Simulador_Cinta(void);


/* Misc */
void TogglePin(volatile uint8_t *port, uint8_t pin);

/* Hardware */
void HCSR04();
void HandlePhysicalIRs(void);
void SetTimeOutServo(void);

// TCRT5000_t IrEntry = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD5 };
// TCRT5000_t IrQ0    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD2 };
// TCRT5000_t IrQ1    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD3 };
// TCRT5000_t IrQ2    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD4 };

TCRT5000_t IrEntry = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD2 };
TCRT5000_t IrQ0    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD3 };
TCRT5000_t IrQ1    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD4 };
TCRT5000_t IrQ2    = { .mode = TCRT_DIGITAL, .pin_reg = &PIND, .pin_num = PD5 };

void DoStartBotton();
void DoStopBotton();
void DoResetBotton();


void Sensor_Trig(uint8_t state);
uint8_t Sensor_Echo(void);
uint32_t Sensor_GetUs(void);

HCSR04_Config_t SensorCajas;
uint32_t last_sensor_trigger; 

/* ============================================================
 * MACROS DE COMPATIBILIDAD
 * -----------------------------------------------------------------------------
 * El código de aplicación usaba TxAddChar / TxSendString / SendSimuCMD
 * directamente. Estos macros redirigen de forma transparente a la librería
 * sin tener que renombrar cada llamada en todo el archivo.
 * ============================================================ */

/* ============================================================
 * IMPLEMENTACIÓN — TX
 * -----------------------------------------------------------------------------
 * Las funciones TxAddChar, TxSendString y SendSimuCMD han sido
 * reemplazadas por los macros de compatibilidad definidos arriba,
 * que delegan directamente en Protocol_TxAddChar,
 * Protocol_TxSendString y Protocol_SendSimuCMD respectivamente.
 *
 * HandleTX: en la versión original gestionaba el envío char a char
 * del pMsg pendiente. La librería gestiona el vaciado del ring buffer
 * íntegramente mediante la ISR USART_UDRE_vect, por lo que HandleTX
 * queda como stub vacío para no romper la llamada en el loop principal.
 * ============================================================ */

#define TxAddChar(d)            Protocol_TxAddChar(d)
#define TxSendString(s)         Protocol_TxSendString(s)
#define SendSimuCMD(cmd, pl, l) Protocol_SendSimuCMD(cmd, pl, l)

/* ============================================================
 * VARIABLES GLOBALES
 * ============================================================
 * Rx y Tx son extern protocol_rx_t / protocol_tx_t declarados
 * en Protocol_UNER.h y definidos en Protocol_UNER.c            */


/* Variable dinámica para el tiempo de extensión de los actuadores */
volatile uint16_t WaitTime = 160;
_sActuator   Actuator[3];

volatile system_state_t sys_state   = ST_IDLE;
volatile uint32_t       tick_ms     = 0;    /* Incrementado cada 2ms por ISR Timer0 */

/* ovf_counter_hb para heartbeat Timer1 */
uint8_t ovf_counter_hb = 0;

/*
 * config_salidas[n] = tipo de caja asignado a la salida n (6, 8 o 10)
 * Recibido en el payload del comando 0x50:
 *   payload[0] = v*10  (velocidad)
 *   payload[1] = boxType0
 *   payload[2] = boxType1
 *   payload[3] = boxType2
 */
uint8_t config_salidas[3];

/* Timestamp para debug LED */
uint32_t debug_led_ts = 0;

/* Instancia Global de eventos */
_sEventFlags Ev;

/* Manejo de Queue */
uint8_t Queue0[MaxQueue];
uint8_t Queue1[MaxQueue];
uint8_t Queue2[MaxQueue];

uint8_t Qelements0 = 0;
uint8_t Qelements1 = 0;
uint8_t Qelements2 = 0;

uint8_t lastboxtype = 0;

// Variables de resguardo para el traspaso entre zonas
volatile uint8_t last_box_Q1 = 0, last_box_Q2 = 0;

/* Declaracion de actuadores*/
SG90_t Servo[3];
uint8_t BeltVel;

/* Tiempos de Actuadores Dinámicos */
volatile uint16_t ActExtendMs = 300;
volatile uint16_t ActDelayMs = 100;

/* Límites dinámicos del sensor ultrasónico (Configurables vía CMD 0x60) */
volatile uint8_t min_6cm  = 14;
volatile uint8_t max_6cm  = 15;
volatile uint8_t min_8cm  = 11;  // Nota: Antes tu código usaba >6 (o sea 7). Ahora es ajustable.
volatile uint8_t max_8cm  = 13;
volatile uint8_t min_10cm = 9;
volatile uint8_t max_10cm = 11;

volatile uint32_t ir_entry_fall_ts = 0;
volatile uint8_t ir_entry_last = 1;


typedef struct {
	uint32_t timestamp[KICK_FIFO_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} kick_fifo_t;

kick_fifo_t KickFIFO[3];


/* Manejo del debounce de los botones */

debounce_t StartBotton = {
    .pressed_count = 0,
    .prev_state = 1,
    .onPress = NULL,
    .onRelease = NULL
};

debounce_t StopBotton = {
    .pressed_count = 0,
    .prev_state = 1,
    .onPress = NULL,
    .onRelease = NULL
};

debounce_t ResetBotton = {
    .pressed_count = 0,
    .prev_state = 1,
    .onPress = NULL,
    .onRelease = NULL
};


static void KickFIFO_Push(uint8_t servo, uint32_t ts){
	if (servo > 2) return;
	if (KickFIFO[servo].count >= KICK_FIFO_SIZE) return;
	KickFIFO[servo].timestamp[KickFIFO[servo].head] = ts;
	KickFIFO[servo].head++;
	if (KickFIFO[servo].head >= KICK_FIFO_SIZE){
		KickFIFO[servo].head = 0;
	}
	KickFIFO[servo].count++;
}

static uint8_t KickFIFO_Peek(uint8_t servo, uint32_t *ts){
	if (KickFIFO[servo].count == 0) return 0;
	*ts = KickFIFO[servo].timestamp[KickFIFO[servo].tail];
	return 1;
}

static void KickFIFO_Pop(uint8_t servo){
	if (KickFIFO[servo].count == 0) return;
	KickFIFO[servo].tail++;
	if (KickFIFO[servo].tail >= KICK_FIFO_SIZE){
		KickFIFO[servo].tail = 0;
	}
	KickFIFO[servo].count--;
}

void HandleTimeMode(void){

	if (Ev.isMode != 0) return;
	if (sys_state != ST_RUNNING) return;

	uint8_t curr_ir = TCRT5000_ReadDigital(&IrEntry);

	uint32_t now;

	cli();
	now = tick_ms;
	sei();

	/* =====================================================
	 * FLANCO DE BAJADA (La caja empieza a tapar el sensor)
	 * ===================================================== */
	if ((curr_ir == 0) && (ir_entry_last == 1)) {
		cli();
		ir_entry_fall_ts = tick_ms; // Registramos el momento exacto en que entra la caja
		sei();
	}

	/* =====================================================
	 * FLANCO DE SUBIDA (La caja termina de pasar)
	 * ===================================================== */
	if ((curr_ir == 1) && (ir_entry_last == 0)) {
		uint32_t delta_t;
		cli();
		delta_t = tick_ms - ir_entry_fall_ts;
		sei();
        
		/* FILTRO ANTI-RUIDO */
		if (delta_t >= 20) {
			/* =========================================
			 * VELOCIDAD (Solo para actualizar Qt)
			 * ========================================= */
			BeltVel = (uint8_t)(((uint32_t)ANCHO_CAJA_CM * 1000UL) / delta_t);
			Ev.reply_send_BeltVel = 1; // Levantamos bandera para mandar a Qt

			/* =========================================
			 * BUSCAR SERVO DESTINO
			 * ========================================= */
			uint8_t target = 255;

			if (lastboxtype == config_salidas[0]) {
				target = 0;
			}
			else if (lastboxtype == config_salidas[1]) {
				target = 1;
			}
			else if (lastboxtype == config_salidas[2]) {
				target = 2;
			}

			/* =========================================
			 * AGENDAR PATADA (CINEMÁTICA EXACTA)
			 * ========================================= */
			if (target != 255) {
				uint32_t dist_cm = 0;

				switch(target){
					case 0: dist_cm = D0_CM; break;
					case 1: dist_cm = D1_CM; break;
					case 2: dist_cm = D2_CM; break;
				}

				// Regla de tres simple para no perder precisión por truncamiento de enteros:
				// Si en recorrer ANCHO_CAJA_CM tardó 'delta_t', ¿cuánto tarda en llegar al centro del servo?
				uint32_t distancia_al_centro = dist_cm + (ANCHO_CAJA_CM / 2);
				uint32_t t_vuelo_exacto = (distancia_al_centro * delta_t) / ANCHO_CAJA_CM;

				// El cronómetro arranca desde el FLANCO DE BAJADA (ir_entry_fall_ts)
				uint32_t ts_patada = ir_entry_fall_ts + t_vuelo_exacto;

				KickFIFO_Push(target, ts_patada);
			}

			/* =========================================
			 * AVISO VISUAL A QT
			 * ========================================= */
			uint8_t payload = lastboxtype;
			SendSimuCMD(0x5F, &payload, 1);
		}
	}

	ir_entry_last = curr_ir;
}
/* ============================================================
 *                          QUEUE
 * ============================================================ */

void HandleQueue() {
	
	if (Ev.isMode == 0) return;

	// --- BLOQUEO DE SEGURIDAD: Si la cinta está detenida, se congela la matriz ---
	if (sys_state != ST_RUNNING) return;

    // --- TRAMO 0 (Entrada desde el Medidor 0x5F) ---
    if (Ev.box_entry_active && !Ev.movQ0){
        if (Qelements0 < MaxQueue) {
            Queue0[MaxQueue - 1 - Qelements0] = lastboxtype;
            Qelements0++;
            Ev.box_entry_active = 0;
            DebugQueues();
        }
    }

    if (Ev.ir0_active && !Ev.movQ0) {
        Ev.ir0_active = 0;
        if (Qelements0 > 0) {
            if (Queue0[MaxQueue - 1] == config_salidas[0]) {
                Actuator[0].state = ACT_WAITING;
                Actuator[0].timestamp_ms = tick_ms;
            } else {
                // PASO DE ESTAFETA: De zona 0 a zona 1
                last_box_Q1 = Queue0[MaxQueue - 1];
                Ev.box_entry_Q1 = 1;
            }
            Queue0[MaxQueue - 1] = 0;
            Qelements0--;
            Ev.movQ0 = 1; // Inicia tarea de desplazamiento en Q0
        }
    }

    // --- TRAMO 1 (Entrada desde Zona 0) ---
    if (Ev.box_entry_Q1 && !Ev.movQ1) {
        if (Qelements1 < MaxQueue) {
            Queue1[MaxQueue - 1 - Qelements1] = last_box_Q1;
            Qelements1++;
            Ev.box_entry_Q1 = 0;
        }
    }

    if (Ev.ir1_active && !Ev.movQ1) {
        Ev.ir1_active = 0;
        if (Qelements1 > 0) {
            if (Queue1[MaxQueue - 1] == config_salidas[1]) {
                Actuator[1].state = ACT_WAITING;
                Actuator[1].timestamp_ms = tick_ms;
            } else {
                // PASO DE ESTAFETA: De zona 1 a zona 2
                last_box_Q2 = Queue1[MaxQueue - 1];
                Ev.box_entry_Q2 = 1;
            }
            Queue1[MaxQueue - 1] = 0;
            Qelements1--;
            Ev.movQ1 = 1; // Inicia tarea de desplazamiento en Q1
        }
    }

    // --- TRAMO 2 (Entrada desde Zona 1 y Descarte Final) ---
    if (Ev.box_entry_Q2 && !Ev.movQ2) {
        if (Qelements2 < MaxQueue) {
            Queue2[MaxQueue - 1 - Qelements2] = last_box_Q2;
            Qelements2++;
            Ev.box_entry_Q2 = 0;
        }
    }

    if (Ev.ir2_active && !Ev.movQ2) {
        Ev.ir2_active = 0;
        if (Qelements2 > 0) {
            if (Queue2[MaxQueue - 1] == config_salidas[2]) {
                Actuator[2].state = ACT_WAITING;
                Actuator[2].timestamp_ms = tick_ms;
                
            }
            // Para Queue2, si no se patea, simplemente se elimina (descarte)
            // No hay Queue3, así que el lugar queda vacío.
            Queue2[MaxQueue - 1] = 0;
            Qelements2--;
            Ev.movQ2 = 1; // Inicia tarea de desplazamiento en Q2
        }
    }

    // --- TAREAS DE DESPLAZAMIENTO (SHIFTING SERIALIZADO) ---

    if (Ev.movQ0) {
        static uint8_t i0 = 1;
        Queue0[MaxQueue - i0] = Queue0[MaxQueue - i0 - 1];
        if (++i0 == MaxQueue) { Queue0[0] = 0; Ev.movQ0 = 0; i0 = 1; DebugQueues(); }
    }

    if (Ev.movQ1) {
        static uint8_t i1 = 1;
        Queue1[MaxQueue - i1] = Queue1[MaxQueue - i1 - 1];
        if (++i1 == MaxQueue) { Queue1[0] = 0; Ev.movQ1 = 0; i1 = 1; DebugQueues(); }
    }

    if (Ev.movQ2){ 
        static uint8_t i2 = 1;
        Queue2[MaxQueue - i2] = Queue2[MaxQueue - i2 - 1];
        if (++i2 == MaxQueue) { Queue2[0] = 0; Ev.movQ2 = 0; i2 = 1; DebugQueues(); }
    }
}

/* ============================================================
 * IMPLEMENTACIÓN — ACTUADORES
 * ============================================================ */

/*
 * FireActuator — Envía el comando 0x52 al simulador para extender o retraer
 *                el brazo de la salida `outNum`.
 *
 * CMD 0x52 payload: [boxType_byte][armPosition_byte]
 *   - boxType_byte   : bitmask de salidas a las que aplica el cambio de estado
 *                      bit n = 1 ? cambiar estado del brazo de la salida n
 *   - armPosition_byte: bitmask de posición deseada
 *                      bit n = 1 ? extender, bit n = 0 ? retraer
 *
 * Ejemplo extender salida 1: boxType=0b010, armPos=0b010
 * Ejemplo retraer  salida 1: boxType=0b010, armPos=0b000
 */
void FireActuator(uint8_t outNum, uint8_t extend) {
    if (outNum > 2) return;
	
	//Como el servo esta al revez patea en 0 y retrae a 120 pero la lógica es la misma. 
	
	if (extend) {
		SG90_SetAngle(&Servo[outNum], 0); // Patear
		} else {
		SG90_SetAngle(&Servo[outNum], 130);  // Retraer
	}
	
    uint8_t mask = (1 << outNum);
    uint8_t pl[2];
    pl[0] = mask;
    pl[1] = extend ? mask : 0x00;

    SendSimuCMD(0x52, pl, 2);

    /* --- MENSAJE DE DEBUG PARA HÉRCULES --- */
    /*if (extend) {
        TxSendString("\r\n>>> PATEANDO BRAZO: ");
        TxAddChar(outNum + '0'); // Muestra 0, 1 o 2
        TxSendString("\r\n");
    }*/

    //PORTB |= (1 << PB1);
}	

/*
 * HandleActuators — Máquina de estados unificada para los 3 actuadores.
 * Se llama desde el loop principal (NO bloqueante).
 * Usa tick_ms para medir los tiempos de transición.
 */
/*
 * HandleActuators — Máquina de estados unificada para los 3 actuadores.
 * Se llama desde el loop principal (NO bloqueante).
 * Usa tick_ms para medir los tiempos de transición.
 */
void HandleActuators(void) {

	uint32_t now;

	/* =========================================================
	 * LECTURA ATÓMICA DEL TICK GLOBAL
	 * ========================================================= */
	cli();
	now = tick_ms;
	sei();

	/* Un solo bucle para los 3 actuadores en CUALQUIER modo */
	for (uint8_t i = 0; i < 3; i++) {
		
		uint32_t kick_ts;

		switch (Actuator[i].state) {
			
			/* =================================================
			 * ACT_IDLE (Reposo)
			 * ================================================= */
			case ACT_IDLE:
				// En MODO TIEMPO (0), el IDLE se encarga de revisar la agenda (FIFO)
				if (Ev.isMode == 0) {
					if (KickFIFO_Peek(i, &kick_ts)) {
						/* Comparación segura para overflow */
						if ((int32_t)(now - kick_ts) >= 0) {
							KickFIFO_Pop(i);          // Sacamos la caja de la lista
							FireActuator(i, 1);       // Ordenamos patear
							Actuator[i].state = ACT_EXTENDING;
							Actuator[i].timestamp_ms = now;
							
							Ev.reply_send_BeltVel = 1; // Avisamos a Qt para actualizar la Q visual
						}
					}
				}
				// En MODO IR (1), el IDLE es pasivo. Solo espera que HandleQueue() 
				// lo pase al estado ACT_WAITING.
			break;

			/* =================================================
			 * ACT_WAITING (Sala de espera antes del golpe)
			 * ================================================= */
			case ACT_WAITING:
				// En MODO IR (1), esperamos el tiempo de centrado antes de patear
				if (Ev.isMode == 1) {
					if ((now - Actuator[i].timestamp_ms) >= WaitTime) {
						FireActuator(i, 1);       // Ordenamos patear
						Actuator[i].state = ACT_EXTENDING;
						Actuator[i].timestamp_ms = now;
					}
				} else {
					// MODO TIEMPO no usa este estado. Si cae acá por error, lo reparamos.
					Actuator[i].state = ACT_IDLE;
				}
			break;

			/* =================================================
			 * ACT_EXTENDING (Saliendo...) - COMPARTIDO POR AMBOS MODOS
			 * ================================================= */
			case ACT_EXTENDING:
				// AHORA USA LA VARIABLE DINÁMICA DE QT
				if ((now - Actuator[i].timestamp_ms) >= ActExtendMs) {
					FireActuator(i, 0);       // Ordenamos retraer
					Actuator[i].state = ACT_RETRACTING;
					Actuator[i].timestamp_ms = now;
				}
			break;

			/* =================================================
			 * ACT_RETRACTING (Volviendo...) - COMPARTIDO POR AMBOS MODOS
			 * ================================================= */
			case ACT_RETRACTING:
				// AHORA USA LA VARIABLE DINÁMICA DE QT
				if ((now - Actuator[i].timestamp_ms) >= ActDelayMs) {
					Actuator[i].state = ACT_IDLE; // Liberado y listo para otra caja
				}
			break;
		}
	}
}
/* ============================================================
 * INFRARROJOS
 * ============================================================ */

void HandlePhysicalIRs(void){
	
	// --- BLOQUEO DE SEGURIDAD: Si la cinta está detenida, se congela la matriz ---
	if (sys_state != ST_RUNNING) return;
	
	if (Ev.hw_sensors_enabled == 0) return; // Si Qt manda, ignoramos el hardware físico
	
	static uint8_t last_ir0 = 1, last_ir1 = 1, last_ir2 = 1;
	
	uint8_t curr_ir0 = TCRT5000_ReadDigital(&IrQ0);
	if (curr_ir0 == 0 && last_ir0 == 1) {
		Ev.ir0_active = 1;
		uint8_t pl[2] = {0x00, 0x01}; // Sensor 0, Estado 1 (Entrando)
		SendSimuCMD(0x5E, pl, 2);     // Avisamos a Qt
	}
	last_ir0 = curr_ir0;
	
	uint8_t curr_ir1 = TCRT5000_ReadDigital(&IrQ1);
	if (curr_ir1 == 0 && last_ir1 == 1) {
		Ev.ir1_active = 1;
		uint8_t pl[2] = {0x01, 0x01}; // Sensor 1, Estado 1
		SendSimuCMD(0x5E, pl, 2);
	}
	last_ir1 = curr_ir1;
	
	uint8_t curr_ir2 = TCRT5000_ReadDigital(&IrQ2);
	if (curr_ir2 == 0 && last_ir2 == 1) {
		Ev.ir2_active = 1;
		uint8_t pl[2] = {0x02, 0x01}; // Sensor 2, Estado 1
		SendSimuCMD(0x5E, pl, 2);
	}
	last_ir2 = curr_ir2;
}
/* ============================================================
 * ULTRASONICO
 * ============================================================ */
void HCSR04(void) {
	static bool last_entry_ir = 1;
	static uint32_t last_measurement_tick = 0;
	static hcsr_state_t last_fsm_state = HCSR_IDLE; // Tracker para flancos lógicos de la FSM

	/* 1. DETECCIÓN DEL FLANCO DE BAJADA (Objeto detectado por IR) */
	bool curr_entry_ir = TCRT5000_ReadDigital(&IrEntry);
	
	if (/*(sys_state == ST_RUNNING) &&*/(curr_entry_ir == 0) && (last_entry_ir == 1) &&
	(SensorCajas.state == HCSR_IDLE) &&
	((tick_ms - last_measurement_tick) > 200)){
		
		SensorCajas.t_ref = SensorCajas.get_us(); // Sincronización en microsegundos
		SensorCajas.state = HCSR_WAIT_CENTER;
		last_measurement_tick = tick_ms;
		// Inicializamos el trigger acá por prolijidad, aunque la Opción B lo pise luego.
		last_sensor_trigger = tick_ms;
	}
	last_entry_ir = curr_entry_ir;
	
	/* 2. PROCESAMIENTO DE LA MÁQUINA DE ESTADOS DEL SENSOR */
	// Acá adentro la FSM decide si cambia de WAIT_CENTER a TRIG_START
	HCSR04_Process(&SensorCajas);
	
	/* 3. ACTUALIZACIÓN DINÁMICA DEL WATCHDOG (Opción B) */
	// Evaluamos el estado INMEDIATAMENTE después de procesarlo.
	// Detectamos la transición exacta: Terminó de esperar -> Va a disparar el pulso
	if ((last_fsm_state == HCSR_WAIT_CENTER) && (SensorCajas.state == HCSR_TRIG_START)) {
		last_sensor_trigger = tick_ms; // Reiniciamos el cronómetro excluyendo el wait_time_center
	}
	last_fsm_state = SensorCajas.state;

	/* 4. TIMEOUT DE SEGURIDAD (Watchdog lógico) */
	// Solo auditamos el cuelgue si el sensor está activamente emitiendo/escuchando
	if ((SensorCajas.state != HCSR_IDLE) && (SensorCajas.state != HCSR_WAIT_CENTER)) {
		if ((tick_ms - last_sensor_trigger) > 100) {
			SensorCajas.state = HCSR_IDLE; // Abortar medición trabada
		}
	}
	
	/* 5. LECTURA Y CLASIFICACIÓN DE RESULTADOS */
	if (SensorCajas.state == HCSR_DATA_READY) {
		uint16_t cm = SensorCajas.distancia;
		uint8_t measure = 0;

		/* Clasificación paramétrica dinámica */
		if (cm >= min_6cm && cm <= max_6cm) { measure = 6; }
		else if (cm >= min_8cm && cm <= max_8cm) { measure = 8; }
		else if (cm >= min_10cm && cm <= max_10cm)  { measure = 10; }

		if (measure != 0) {
			lastboxtype = measure;
			Ev.box_entry_active = 1;
			
			/* Inyección al protocolo de comunicaciones */
			uint8_t payload = lastboxtype;
			SendSimuCMD(0x5F, &payload, 1);
		}

		/* Liberación incondicional de los recursos */
		SensorCajas.state = HCSR_IDLE;
	}
}
/* ============================================================
 * CALLBACKS DE COMANDOS (RX desde el simulador)
 * ============================================================ */

/*
 * Cmd_SetActuatorTimes — (0x62 SIMU->MICRO).
 * Payload: [ExtLow][ExtHigh][DelLow][DelHigh]
 * Acción: Actualiza los tiempos de extensión y retracción de los servos.
 */
void Cmd_SetActuatorTimes(void) {
	if (Rx.payloadLen >= 4) {
		ActExtendMs = (uint16_t)Rx.payload[0] | ((uint16_t)Rx.payload[1] << 8);
		ActDelayMs  = (uint16_t)Rx.payload[2] | ((uint16_t)Rx.payload[3] << 8);
		
		Ev.reply_send_act_times_ack = 1; // Bandera para enviar ACK
	}
}

/* Modificación Dinámica del Tiempo de Espera al centro del HCSR04 */
void Cmd_SetWaitCenter(void) {
	if (Rx.payloadLen >= 2) {
		SensorCajas.wait_time_center = (uint16_t)Rx.payload[0] | ((uint16_t)Rx.payload[1] << 8);
		Ev.reply_send_wait_center_ack = 1; // Levantamos bandera para el ACK
	}
}


/*
 * Cmd_SetMode — (0x59 SIMU->MICRO)
 * Payload: [0x00] Modo Tiempo | [0x01] Modo IR
 */
void Cmd_SetMode(void) {
    if (Rx.payloadLen >= 1) {
        // Asignamos el nuevo modo (0 o 1)
        Ev.isMode = Rx.payload[0]; 

        // Levantamos la bandera para responder el ACK a Qt.
        // NOTA: Se mantienen las colas y FIFOs intactas porque 
        // las cajas que ya entraron siguen físicamente en la cinta.
        Ev.reply_send_mode_ack = true;
		
    }
}
/*
 * Cmd_Calibracion — (0x60 SIMU->MICRO).
 * Payload: [Min6][Max6][Min8][Max8][Min10][Max10]
 * Acción: Actualiza los límites del sensor ultrasónico.
 */
void Cmd_Calibracion(void) {
	if (Rx.payloadLen >= 6) {
		min_6cm  = Rx.payload[0];
		max_6cm  = Rx.payload[1];
		min_8cm  = Rx.payload[2];
		max_8cm  = Rx.payload[3];
		min_10cm = Rx.payload[4];
		max_10cm = Rx.payload[5];
		
		// Levantamos la bandera para enviar el ACK a Qt
		Ev.reply_send_calib_ack = 1;
	}
}
/*
 * Cmd_SetManualTimeout — (0x56 SIMU->MICRO)
 * Payload: [0x01] Manual / [0x00] Automático
 */
void Cmd_SetManualTimeout(void) {
	if (Rx.payloadLen >= 1){
		Ev.manual_timeout_enabled = Rx.payload[0];	
	}
}

/*
 * Cmd_AckAlive — Respuesta del simulador al ALIVE (0xF0 SIMU?MICRO).
 * El simulador envía payload = 0x0D.
 * Acción: transitar a ST_READY y responder con 0x50 (START) para
 * solicitar el encendido de la cinta.
 *
 * NOTA: Según el protocolo:
 *   - MICRO?SIMU 0xF0 (sin payload): verifica conexión
 *   - SIMU?MICRO 0xF0 payload=0x0D: ACK conexión
 * En este callback ya estamos recibiendo el ACK del simulador,
 * por lo que respondemos encendiendo la cinta con 0x50.
 */

void Cmd_AckAlive(void) {
	// Si recibimos el 0x0D, es el ACK del simulador.
	if (Rx.payloadLen > 0 && Rx.payload[0] == 0x0D) {
		sys_state = ST_READY;
		// Fin de la transacción. NO levantamos la bandera de reply.
	}
}

/*
 * Cmd_ConfigCinta — Respuesta del simulador al START (0x50 SIMU?MICRO).
 * Payload: [v*10][boxType0][boxType1][boxType2]
 * Acción: guardar configuración y transitar a ST_RUNNING.
 */
void Cmd_ConfigCinta(void) {
	// Verificamos que lleguen los 3 bytes desde Qt
	if (Rx.payloadLen >= 3) {
		config_salidas[0] = Rx.payload[0];
		config_salidas[1] = Rx.payload[1];
		config_salidas[2] = Rx.payload[2];
		
		// Levantamos la bandera para enviar el ACK
		Ev.reply_send_config = 1;
	}
}
/*
 * Cmd_AckStop — ACK de detención (0x51 SIMU?MICRO, payload=0x0D).
 */
void Cmd_AckStop(void) {
	sys_state = ST_READY;
	
	// NUEVO: Detener motor poniendo el pin en 1
	PORTC |= (1 << PC0);

	if (Rx.payloadLen > 0 && Rx.payload[0] == 0x0D) return;
	
	Ev.reply_send_stop = 1;
}

/*
 * Cmd_AckReset — ACK de reset (0x53 SIMU?MICRO).
 * payload[0] = 0x0D ? reset OK
 * payload[0] = 0x0A ? no pudo resetearse
 */
void Cmd_AckReset(void) {
	
	PORTC |= (1 << PD0);          // INICIO: Lo ponemos en 1 para que arranque FRENADO
	
	// 1. Limpieza de colas del Modo IR
	memset(Queue0, 0, sizeof(Queue0));
	memset(Queue1, 0, sizeof(Queue1));
	memset(Queue2, 0, sizeof(Queue2));
	Qelements0 = 0;
	Qelements1 = 0;
	Qelements2 = 0;
	lastboxtype = 0;
	last_box_Q1 = 0;
	last_box_Q2 = 0;
	
	// 2. Limpieza de banderas de eventos
	Ev.box_entry_active = 0;
	Ev.ir0_active = 0;
	Ev.ir1_active = 0;
	Ev.ir2_active = 0;
	Ev.movQ0 = 0;
	Ev.movQ1 = 0;
	Ev.movQ2 = 0;
	Ev.box_entry_Q1 = 0;
	Ev.box_entry_Q2 = 0;

	// 3. NUEVO: Limpieza absoluta del Modo Tiempo
	memset(KickFIFO, 0, sizeof(KickFIFO)); // Pone a 0 todos los contadores, cabezas, colas y timestamps
	BeltVel = 0;                           // Borra la velocidad calculada
	ir_entry_fall_ts = 0;                  // Borra el cronómetro del sensor
	ir_entry_last = 1;                     // Devuelve el sensor a su estado natural "destapado" (Pull-up alto)
	Ev.reply_send_BeltVel = 0;             // Baja bandera de TX por si quedó colgada

	// 4. Limpieza de Actuadores y Estado General
	for (uint8_t i = 0; i < 3; i++) {
		Actuator[i].state = ACT_IDLE;
		Actuator[i].timestamp_ms = 0;
	}
	Ev.hw_sensors_enabled = 1;

	sys_state = ST_READY; // CAMBIO: Así el LED PB5 se queda encendido fijo indicando que sigue conectado.

	// 5. Filtro Corta-Lazos (ping-pong entre micro y Qt)
	if (Rx.payloadLen > 0 && (Rx.payload[0] == 0x0D || Rx.payload[0] == 0x0A)) return;

	Ev.reply_send_reset = 1;
}

/*
 * Cmd_Start — (0x50 SIMU->MICRO).
 * Acción: Arranca la cinta (pasa a ST_RUNNING).
 */
/* ---------------------------------------------------------
 * START (0x50) - EXTRAE CONFIG DEL PROFE O USA LA DE QT
 * --------------------------------------------------------- */
void Cmd_Start(void) {
	// 1. MODO CÁTEDRA (Profesor)
	if (Rx.payloadLen >= 4) {
		BeltVel = Rx.payload[0];
		config_salidas[0] = Rx.payload[1];
		config_salidas[1] = Rx.payload[2];
		config_salidas[2] = Rx.payload[3];
		SetTimeOutServo();
		
		
		// CONTROL DEL MOTOR DE LA CINTA (PD0)
		PORTC &= ~(1 << PC0);  // PC0 en 0 (MARCHA)
		
		sys_state = ST_RUNNING;
	}
	// 2. MODO QT (Tu interfaz)
	else if (Rx.payloadLen == 0) {
		if ((config_salidas[0] != 0) && (config_salidas[1] != 0) && (config_salidas[2] != 0)) {
			
			// NUEVO: Arrancar motor poniendo el pin en 0
			PORTC &= ~(1 << PC0);  // PC0 en 0 (MARCHA)
		

			sys_state = ST_RUNNING;
			Ev.reply_send_start = 1; 
		} else {
			Ev.reply_error_start = 1;
		}
	}
}
/*
 * Cmd_AckActuador — ACK del simulador al CMD 0x52.
 * payload[0] = 0xFF ? OK
 * No se necesita acción adicional; el timing lo gestiona HandleActuators.
 */
void Cmd_AckActuador(void) {
	// FILTRO CORTA-LAZOS: Si el simulador del profesor responde 0xFF, es un ACK. No hacemos nada.
	if (Rx.payloadLen == 1 && Rx.payload[0] == 0xFF) {
		return;
	}

	// Si tiene al menos 2 bytes, es una orden manual de tu interfaz Qt
	if (Rx.payloadLen >= 2) {
		uint8_t mask = Rx.payload[0];
		uint8_t pos  = Rx.payload[1];

		for (uint8_t i = 0; i < 3; i++) {
			if (mask & (1 << i)) { // Si el bit correspondiente a este brazo está en 1
				if (pos & (1 << i)) {
					// Qt ordenó EXTENDER
					Actuator[i].state = ACT_WAITING;
					Actuator[i].timestamp_ms = tick_ms;
					} else {
					// Qt ordenó RETRAER
					Actuator[i].state = ACT_RETRACTING;
					Actuator[i].timestamp_ms = tick_ms;
				}
			}
		}
	}
}
/*
 * Cmd_AckVelocidad — ACK del simulador al CMD 0x54.
 * payload[0] = v*10, payload[1] = 0x0D
 */
void Cmd_AckVelocidad(void) {
	if (Rx.payloadLen >= 1) {
		BeltVel = Rx.payload[0];
		SetTimeOutServo(); // Si está en Auto, esto actualiza el WaitTime y manda ACK
		
		if (Rx.payloadLen > 1 && Rx.payload[1] == 0x0D) return;
		Ev.reply_send_speed_ack = 1;
	}
}

/*
 * Cmd_SensorEvent — Procesamiento del comando 0x5E (SIMU?MICRO).
 *
 * El simulador envía pares [outNum][IRState] en el payload.
 * Si múltiples sensores cambian al mismo tiempo, el payload contiene
 * múltiples pares consecutivos.
 *
 * Ejemplo: payload = {0x00, 0x01, 0x02, 0x00} significa:
 *   - Sensor salida 0: IRState=1 (caja entra)
 *   - Sensor salida 2: IRState=0 (caja sale)
 *
 * Solo nos interesa IRState=1 (entrada de caja al sensor) para disparar
 * el actuador a tiempo.
 */
void Cmd_SensorEvent(void) {
	
	Ev.hw_sensors_enabled = 0; // Qt mandó un comando, le pasamos el control.

    uint8_t i = 0;

    // Recorremos TODO el payload de a pares [outNum, irState]
    while (i + 1 < Rx.payloadLen) {
        uint8_t outNum  = Rx.payload[i];
        uint8_t irState = Rx.payload[i + 1];
        i += 2; // Saltamos al siguiente par

        if (irState == 1) { // Solo nos interesa cuando la caja ENTRA
            switch(outNum) {
                case 0: Ev.ir0_active = 1; break;
                case 1: Ev.ir1_active = 1; break;
                case 2: Ev.ir2_active = 1; break;
            }
        }
    }
}

/*
 * Cmd_NuevaCaja — Indica que una nueva caja fue medida en la cinta (0x5F).
 * payload[0] = boxType (6, 8 o 10).
 * Acción: encolar el tipo de caja para que ClassifyBox pueda procesarla
 * cuando llegue al sensor correspondiente.
 */
void Cmd_NuevaCaja(void) {
	
	Ev.hw_sensors_enabled = 0; // Qt mando un comando, le pasamos el control. 
	
    // 1. Guardamos el tipo de caja en la variable de resguardo
    // Rx.payload[0] tiene el 6, 8 o 10
    lastboxtype = Rx.payload[0];

    // 2. Levantamos el flag para que el while(1) sepa que hay una caja nueva
    Ev.box_entry_active = 1;
	
}

/* Modificación Dinámica del Timeout del Servo */
void Cmd_ConfigTimeout(void) {
	if (Rx.payloadLen >= 2) {
		// Solo actualizamos WaitTime si el modo manual está activado
		if (Ev.manual_timeout_enabled) {
			WaitTime = (uint16_t)Rx.payload[0] | ((uint16_t)Rx.payload[1] << 8);
		}
		SetTimeOutServo(); // Esto confirma el valor (o lo corrige si no estaba en manual)
	}
}
/* ============================================================
 * DESPACHO DE RESPUESTAS TX DIFERIDAS
 * -----------------------------------------------------------------------------
 * Se llama desde el loop principal DESPUÉS de Protocol_HandleUART.
 * Garantiza que cualquier SendSimuCMD se ejecute cuando la máquina
 * de estados del protocolo ya terminó de procesar la trama entrante,
 * evitando que la respuesta inmediata del simulador Qt corrompa el
 * estado del parser RX.
 * ============================================================ */
/* ============================================================
 * DESPACHO DE RESPUESTAS TX DIFERIDAS
 * -----------------------------------------------------------------------------
 * Se llama desde el loop principal DESPUÉS de Protocol_HandleUART.
 * Garantiza que cualquier SendSimuCMD se ejecute cuando el buffer RX
 * está libre, evitando colisiones o saturación del pipeline.
 * ============================================================ */
void HandlePendingReplies(void) {
    
    /* =========================================================
     * 1. ERRORES CRÍTICOS Y EXCEPCIONES 
     * ========================================================= */
	if (Ev.reply_error) {
		Ev.reply_error = 0;
		sys_state = ST_ERROR;

		// NUEVO: Parada de emergencia, motor en 1
		PORTD |= (1 << PD0);

		SendSimuCMD(0x51, NULL, 0);
	}
    
    if (Ev.reply_error_start) {
        Ev.reply_error_start = 0;
        SendSimuCMD(0x5A, NULL, 0); // CMD 0x5A: Error - Intento de arranque sin configuración
    }

    /* =========================================================
     * 2. CONEXIÓN Y CONTROL DE ESTADO DEL SISTEMA
     * ========================================================= */
    if (Ev.reply_send_alive) { 
        Ev.reply_send_alive = 0;
        SendSimuCMD(0xF0, NULL, 0); // Ping saliente del Micro hacia Qt
    }
    
    if (Ev.reply_ack_alive) {
        Ev.reply_ack_alive = 0;
        uint8_t ack = 0x0D; 
        SendSimuCMD(0xF0, &ack, 1); // ACK (0x0D) respondiendo al Ping de Qt
    }
    
    if (Ev.reply_send_start) {
        Ev.reply_send_start = 0;
        SendSimuCMD(0x50, NULL, 0); // Confirmamos a Qt que la cinta arrancó
    }
    
    if (Ev.reply_send_stop) {
        Ev.reply_send_stop = 0;
        SendSimuCMD(0x51, NULL, 0); // Confirmamos a Qt que la cinta se detuvo
    }
    
    if (Ev.reply_send_reset) {
        Ev.reply_send_reset = 0;
        SendSimuCMD(0x53, NULL, 0); // Confirmamos a Qt que la memoria fue borrada
    }

    /* =========================================================
     * 3. CONFIRMACIONES DE CONFIGURACIÓN Y MODOS
     * ========================================================= */
    if (Ev.reply_send_mode_ack) {
        Ev.reply_send_mode_ack = 0; 
        uint8_t payload = Ev.isMode; 
        SendSimuCMD(0x59, &payload, 1); // Confirmamos cambio de modo (0=Tiempo, 1=IR)
    }

    if (Ev.reply_send_config) {
        Ev.reply_send_config = 0;
        uint8_t pl[3];
        pl[0] = config_salidas[0];
        pl[1] = config_salidas[1];
        pl[2] = config_salidas[2];
        SendSimuCMD(0x40, pl, 3);       // Confirmamos los tamaños asignados a las salidas
    }
    
    if (Ev.reply_send_speed_ack) {
        Ev.reply_send_speed_ack = 0;
        uint8_t payload_ack = 0x0D;
        SendSimuCMD(0x54, &payload_ack, 1); // Confirmamos cambio manual de velocidad
    }
    
    if (Ev.reply_send_timeout_ack) {
        Ev.reply_send_timeout_ack = 0;
        uint8_t pl[2];
        pl[0] = (uint8_t)(WaitTime & 0xFF);         // Byte Bajo del tiempo de reacción
        pl[1] = (uint8_t)((WaitTime >> 8) & 0xFF);  // Byte Alto del tiempo de reacción
        SendSimuCMD(0x55, pl, 2);       // Confirmamos nuevo tiempo de WaitTime
    }
	if (Ev.reply_send_manual_timeout_ack) {
		Ev.reply_send_manual_timeout_ack = 0;
		uint8_t pl = Ev.manual_timeout_enabled;
		SendSimuCMD(0x56, &pl, 1);
	}
	/* =========================================================
     * CONFIRMACIÓN DE RETARDO CENTRADO HCSR04 (CMD 0x57)
     * ========================================================= */
	if (Ev.reply_send_wait_center_ack) {
		Ev.reply_send_wait_center_ack = 0; // Bajamos la bandera de evento
		
		uint8_t pl[2];
		// Desarmamos el entero de 16 bits en 2 bytes (Little Endian)
		pl[0] = (uint8_t)(SensorCajas.wait_time_center & 0xFF);        // Byte Bajo
		pl[1] = (uint8_t)((SensorCajas.wait_time_center >> 8) & 0xFF); // Byte Alto
		
		// Enviamos la trama con el CMD 0x57 y los 2 bytes de payload
		SendSimuCMD(0x57, pl, 2);
	}
	/* =========================================================
     * 4. TELEMETRÍA Y CINEMÁTICA EN TIEMPO REAL
     * ========================================================= */
    if (Ev.reply_send_BeltVel) {
        Ev.reply_send_BeltVel = 0;
        
        uint8_t pl[5];
        pl[0] = BeltVel;              // Byte 0: Velocidad calculada en cm/s
        pl[1] = lastboxtype;          // Byte 1: Tamaño de la caja (6, 8 o 10)
        pl[2] = KickFIFO[0].count;    // Byte 2: Cantidad de patadas pendientes en S0
        pl[3] = KickFIFO[1].count;    // Byte 3: Cantidad de patadas pendientes en S1
        pl[4] = KickFIFO[2].count;    // Byte 4: Cantidad de patadas pendientes en S2
        
        SendSimuCMD(0x58, pl, 5);     // Enviamos la actualización con 5 bytes
    }
	/* =========================================================
     * CONFIRMACIÓN DE TIEMPOS DE ACTUADOR (CMD 0x62)
     * ========================================================= */
    if (Ev.reply_send_act_times_ack) {
        Ev.reply_send_act_times_ack = 0;
        SendSimuCMD(0x62, NULL, 0); // Respondemos con 0x62 vacío para confirmar
    }
}
/* ============================================================
 * TABLA DE COMANDOS
 * -----------------------------------------------------------------------------
 * Relaciona cada CMD con su función callback.
 * El decodificador Protocol_DecodeCMD (Protocol_UNER.c) itera esta tabla
 * para despachar el handler correcto al recibir una trama válida.
 * ============================================================ */
const _sCommand command_table[] = {
    /* =========================================================
     * 1. CONTROL DE SISTEMA Y CONEXIÓN (HANDSHAKE)
     * ========================================================= */
    { 0xF0, Cmd_AckAlive              },   /* Ping / Conexión establecida */
    { 0x50, Cmd_Start                 },   /* Arrancar la cinta */
    { 0x51, Cmd_AckStop               },   /* Detener la cinta */
    { 0x53, Cmd_AckReset              },   /* Reinicio de memoria y hardware */

    /* =========================================================
     * 2. MODOS DE OPERACIÓN Y CONFIGURACIÓN PARÁMETROS
     * ========================================================= */
    { 0x59, Cmd_SetMode               },   /* Cambiar Modo (0=Tiempo/Abierto, 1=IR/Cerrado) */
    { 0x40, Cmd_ConfigCinta           },   /* Asignar qué tamaño de caja va a cada salida */
    { 0x54, Cmd_AckVelocidad          },   /* Modificar velocidad manual (BeltVel) */
    { 0x56, Cmd_SetManualTimeout      },   /* Alternar Timeout Automático vs Manual */
    { 0x55, Cmd_ConfigTimeout         },   /* Modificar el tiempo de reacción/espera del servo */
	{0x57, Cmd_SetWaitCenter          },	
	{ 0x60, Cmd_Calibracion           },   /* Calibración del Sensor Ultrasónico */
	{ 0x62, Cmd_SetActuatorTimes      },   /* NUEVO: Configurar tiempos de servo */
    /* =========================================================
     * 3. EVENTOS DE HARDWARE Y SENSORES (SIMULADOS O FÍSICOS)
     * ========================================================= */
    { 0x5F, Cmd_NuevaCaja             },   /* Ingreso: Ultrasónico midió una caja en la entrada */
    { 0x5E, Cmd_SensorEvent           },   /* Tránsito: Se tapó/destapó un sensor IR de zona */

    /* =========================================================
     * 4. CONTROL MANUAL DE ACTUADORES
     * ========================================================= */
    { 0x52, Cmd_AckActuador           },   /* Orden forzada para patear o retraer un brazo */
};


#define MAX_COMMANDS (sizeof(command_table) / sizeof(_sCommand))

/* ============================================================
 * DECODIFICADOR DE COMANDOS
 * -----------------------------------------------------------------------------
 * Recibe el CMD ya validado (checksum OK) desde Protocol_HandleUART
 * y busca en command_table el handler correspondiente.
 *
 * Protocol_DecodeCMD se llama desde Protocol_UNER.c en el estado GET_CKS.
 * Su prototipo está declarado en Protocol_UNER.h; la implementación
 * vive aquí para que tenga acceso directo a la tabla de la aplicación.
 *
 * Para agregar un nuevo comando:
 *   1. Escribir la función callback  void Cmd_NuevoNombre(void) { ... }
 *   2. Añadir una entrada { 0xXX, Cmd_NuevoNombre } en command_table.
 *   No se necesita tocar ni Protocol_UNER.c ni Protocol_UNER.h.
 * ============================================================ */
void Protocol_DecodeCMD(uint8_t cmd,
                        const protocol_command_t *table,
                        uint8_t table_size)
{
    /*
     * Parámetros table / table_size son ignorados aquí: la tabla canónica
     * de esta aplicación es command_table (definida arriba).
     * Si se quisiera reutilizar la librería con múltiples tablas, bastaría
     * con pasar la tabla correcta desde Protocol_HandleUART y eliminar la
     * referencia directa a command_table.
     */
    (void)table;
    (void)table_size;

    for (uint8_t i = 0; i < MAX_COMMANDS; i++) {
        if (command_table[i].cmd_id == cmd) {
            if (command_table[i].execute) command_table[i].execute();
            return;
        }
    }
    /* Comando desconocido — aviso por debug */
    TxSendString("UNER:Cmd_Unknown\n");
}

/* ============================================================
 *                        HARDWARE
 * ============================================================ */

void DoStartBotton() {
    SendSimuCMD(0xF0, NULL, 0); // CMD 0xF0, sin puntero de datos, longitud
	Ev.box_entry_active = 1; 
}

void DoStopBotton() {
    // Condición: Solo actuar si el sistema NO está en IDLE (está conectado)
    // También ignoramos si ya hubo un error crítico (ST_ERROR)
    if (sys_state == ST_RUNNING || sys_state == ST_READY) {

        // 1. Mandar comando de parada según protocolo (0x51)
        SendSimuCMD(0x51, NULL, 0);

        // 2. Seguridad: Forzar retracción física de todos los brazos (0x52)
        // Esto evita que queden a mitad de camino si el simulador se pausa
        uint8_t pl[2] = {0x07, 0x00}; // Mask 0x07 (brazos 0,1,2), Pos 0x00 (retraer)
        SendSimuCMD(0x52, pl, 2);

        // 3. Actualizar estado local a READY (esperando START)
        sys_state = ST_READY;

        TxSendString("\r\n[!] STOP: Cinta detenida.\r\n");
    } else {
        // Si estamos en ST_IDLE o ST_ERROR, no hacemos nada
        TxSendString("\r\n[?] STOP ignorado: No hay conexion activa.\r\n");
    }
}

void DoResetBotton() {
    // 1. Mandar comando de Reset físico al simulador (CMD 0x53)
    SendSimuCMD(0x53, NULL, 0);

    // 2. Borrar el contenido de las colas (Arreglos)
    // Usamos memset para llenar de ceros toda la memoria de los arreglos
    memset(Queue0, 0, sizeof(Queue0));
    memset(Queue1, 0, sizeof(Queue1));
    memset(Queue2, 0, sizeof(Queue2));

    // 3. Resetear contadores de elementos de cada tramo
    Qelements0 = 0;
    Qelements1 = 0;
    Qelements2 = 0;

    // 4. Resetear variables de traspaso de cajas entre zonas
    lastboxtype = 0;
    last_box_Q1 = 0;
    last_box_Q2 = 0;

    // 5. Resetear todas las banderas de eventos de la FSM
    Ev.box_entry_active = 0;
    Ev.ir0_active       = 0;
    Ev.ir1_active       = 0;
    Ev.ir2_active       = 0;
    Ev.movQ0            = 0;
    Ev.movQ1            = 0;
    Ev.movQ2            = 0;
    Ev.box_entry_Q1     = 0;
    Ev.box_entry_Q2     = 0;

    // 6. Resetear actuadores a estado de reposo
    for (uint8_t i = 0; i < 3; i++) {
        Actuator[i].state        = ACT_IDLE;
        Actuator[i].timestamp_ms = 0;
    }

    // 7. Volver al estado de espera inicial
    sys_state = ST_IDLE;

    TxSendString("\r\n[!] RESET INTEGRAL: Software y Hardware limpios.\r\n");
}

/* Control del pin TRIGGER (Salida) */
void Sensor_Trig(uint8_t state) {
	if (state) PORTB |= (1 << PB1);  // Set a 1 (antes era PORTD, PD7)
	else       PORTB &= ~(1 << PB1); // Reset a 0
}

/* Lectura del pin ECHO (Entrada) */
uint8_t Sensor_Echo(void) {
	return (PINB & (1 << PB2)) ? 1 : 0; // (antes era PIND, PD6)
}

/* Generador continuo de microsegundos */
uint32_t Sensor_GetUs(void) {
    /* * Teoría de acumulación temporal:
     * El Timer 1 (TCNT1) se reinicia a 0 cada 32.7 ms. Si el pulso de sonido
     * viaja justo cuando ocurre este reinicio, una resta simple daría un valor
     * negativo (corrompiendo la medición). 
     * Al calcular el 'delta' en 16 bits sin signo, la matemática de desbordamiento 
     * de C corrige el salto automáticamente. Luego, sumamos ese delta a un 
     * acumulador estático de 32 bits, creando una línea de tiempo infinita y segura.
     */
	static uint16_t last_tcnt = 0;
	static uint32_t accumulated_ticks = 0; // Acumulamos TICKS

	uint16_t current_tcnt = TCNT1;
	uint16_t delta_ticks = current_tcnt - last_tcnt;

	accumulated_ticks += delta_ticks; // Suma sin división truncada
	last_tcnt = current_tcnt;

	return (accumulated_ticks / 2); // Devolvemos el tiempo en us
	}
/* ============================================================
 * DEBUG LEDs EN PUERTO B
 *   PB5 — Estado del sistema
 * ============================================================ */
void UpdateDebugLEDs(void) {
	switch (sys_state) {
		case ST_IDLE:
		// Parpadeo muy tenue (ON durante 100ms cada 2 segundos)
		if ((tick_ms % 2000) < 100) {
			PORTB |= (1 << PB5);
			} else {
			PORTB &= ~(1 << PB5);
		}
		break;
		
		case ST_READY:
		// Fijo ON cuando hay conexión pero la cinta está detenida
		PORTB |= (1 << PB5);
		break;
		
		case ST_RUNNING:
		// Parpadeo rápido (toggle cada 100ms) clasificando
		if ((tick_ms - debug_led_ts) >= DEBUG_FAST_MS) {
			PORTB ^= (1 << PB5);
			debug_led_ts = tick_ms;
		}
		break;
		
		case ST_ERROR:
		// Parpadeo lento de advertencia (toggle cada 500ms)
		if ((tick_ms - debug_led_ts) >= 500) {
			PORTB ^= (1 << PB5);
			debug_led_ts = tick_ms;
		}
		break;
	}
}

/*
 * EnviarTextoDebug: Agarra un texto plano, mide su largo, 
 * y lo mete adentro de una trama UNER con el comando 0x70.
 */
void SendText(const char* texto) {
    uint8_t len = strlen(texto);
    SendSimuCMD(0x70, (uint8_t*)texto, len);
}
/* ============================================================
 * DEBUG UART
 * ============================================================ */
void DebugQueues(void) {
	char buffer[64]; // Creamos una hoja en blanco en memoria
	char temp[8];

	SendText("--- ESTADO DE CINTA ---");

	// Imprimir Queue 0
	strcpy(buffer, "Q0: [");
	for (uint8_t i = 0; i < MaxQueue; i++) {
		sprintf(temp, "%d ", Queue0[i]);
		strcat(buffer, temp); // Pegamos el número al final del texto
	}
	strcat(buffer, "]");
	SendText(buffer);

	// Imprimir Queue 1
	strcpy(buffer, "Q1: [");
	for (uint8_t i = 0; i < MaxQueue; i++) {
		sprintf(temp, "%d ", Queue1[i]);
		strcat(buffer, temp);
	}
	strcat(buffer, "]");
	SendText(buffer);

	// Imprimir Queue 2
	strcpy(buffer, "Q2: [");
	for (uint8_t i = 0; i < MaxQueue; i++) {
		sprintf(temp, "%d ", Queue2[i]);
		strcat(buffer, temp);
	}
	strcat(buffer, "]");
	SendText(buffer);
}
/* ============================================================
 * INICIALIZACIONES
 * ============================================================ */

void InitUART0(void) {
    UCSR0A = (1 << U2X0);    /* Double speed */
    UBRR0  = 16;              /* 115200 bps con U2X @ 16 MHz */
    UCSR0C = (3 << UCSZ00);  /* 8N1 */
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void InitPort(void){
	/* PB5 — Heartbeat LED (Salida) */
	DDRB |= (1 << PB5);
	
	/* === ACTUALIZACIÓN: Nuevos pines para TRIGGER y ECHO ===
	 * TRIGGER: PB1 (antes PD7)
	 * ECHO:    PB2 (antes PD6)
	 */
	DDRB |= (1 << PB1);       // TRIGGER como salida
	DDRB &= ~(1 << PB2);      // ECHO como entrada
	
	/* === ACTUALIZACIÓN: Botones reubicados a Puerto C ===
	 * (antes estaban en PB0, PB1, PB2 pero PB1 y PB2 ahora son sensores)
	 */
// 	DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));  // Entradas
// 	PORTC |= ((1 << PC0) | (1 << PC1) | (1 << PC2));  // Pull-ups activadas
	
	/* === ACTUALIZACIÓN: Servos en nuevos puertos ===
	 * SERVO 1: PD7 (antes PD2)
	 * SERVO 2: PB4 (antes PD3)
	 * SERVO 3: PB3 (antes PD4)
	 */
	
	DDRD |= (1 << PD7);       // SERVO 1 (PWM)
	DDRB |= (1 << PB4);       // SERVO 2 (PWM)
	DDRB |= (1 << PB3);       // SERVO 3 (PWM)
	
	/* === ACTUALIZACIÓN: Sensores IR reubicados al Puerto D ===
	 * IrEntry (S?): PD2 (antes PC0)
	 * IrQ0 (S?):    PD3 (antes PC1)
	 * IrQ1 (S?):    PD4 (antes PC2)
	 * IrQ2 (S?):    PD5 (antes PC3)
	 */
	DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5)); // Entradas
	
	//Activamos las Pull-ups internas para evitar ruido fantasma
	PORTD |= ((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5));
	
	// ============================================================
	// CONTROL DEL MOTOR DE LA CINTA (PD0)
	// Lógica: 0 = Motor ANDANDO | 1 = Motor FRENADO
	// ============================================================
	DDRC |= (1 << PC0);           // Configurar PD0 como SALIDA
	PORTC |= (1 << PC0);          // INICIO: Inicializar en 1 (MOTOR FRENAD
}

/*
 * InitTimer0 — CTC, prescaler=256, OCR0A=124
 * Periodo = (OCR0A+1) * (prescaler / F_CPU)
 *         = 125 * (256 / 16.000.000)
 *         = 125 * 16µs = 2 ms  ? ISR COMPA cada 2 ms
 */
void InitTimer0(void) {
    TCCR0A = (1 << WGM01);          /* Modo CTC (necesario para COMPA) */
    TCCR0B = (1 << CS02);           /* Prescaler = 256                  */
    OCR0A  = 124;                   /* TOP = 124 ? periodo = 2 ms       */
    TIMSK0 = (1 << OCIE0A);         /* Habilitar interrupción COMPA     */
}

/*
 * InitTimer1 — Normal mode, prescaler=8
 * Desbordamiento cada: 65536 * (8 / 16.000.000) ? 32.77 ms
 * 30 desbordamientos ? 983 ms ? 1 Hz para heartbeat
 */
void InitTimer1(void) {
	TCCR1A = 0;
	TCCR1B = (1 << CS11);           /* Prescaler = 8     */
}

/* ============================================================
 * FUNCIÓN AUXILIAR
 * ============================================================ */
void TogglePin(volatile uint8_t *port, uint8_t pin) {
    *port ^= (1 << pin);
}

void SetTimeOutServo(void) {
	// Si NO está en manual, calculamos el tiempo en base a la velocidad
	if (!Ev.manual_timeout_enabled) {
		if (BeltVel > 0) {
			
			// Fórmula cinemática optimizada: t = (d * 10) / v_recibida
			uint32_t dist_total_mm = DIST_SENSOR_TO_SERVO_MM + ((ANCHO_CAJA_CM * 10) / 2);
			WaitTime = (uint16_t)((dist_total_mm * 100) / BeltVel);
		}
	}
	
	// En cualquier caso (Manual o Auto), avisamos a HandlePendingReplies
	// que despache el ACK 0x55 con el valor final de WaitTime.
	Ev.reply_send_timeout_ack = true;
}

void Inject_RX_Command(const uint8_t *trama, uint8_t len) {
	uint8_t ucsrb_respaldo = UCSR0B;
	
	UCSR0B &= ~(1 << RXCIE0);

	for (uint8_t i = 0; i < len; i++) {
		uint8_t next_iw = (Rx.rBuf.iw + 1) & MASK;
		if (next_iw != Rx.rBuf.ir) {
			Rx.rBuf.buf[Rx.rBuf.iw] = trama[i];
			Rx.rBuf.iw = next_iw;
		}
	}
	
	/* BORRAMOS LA LLAMADA RECURSIVA QUE ESTABA ACÁ */
	UCSR0B = ucsrb_respaldo;
}

/* ============================================================
 * INTERRUPCIONES
 * ============================================================ */

/*
 * NOTA: ISR(USART_RX_vect) e ISR(USART_UDRE_vect) están definidas
 * en Protocol_UNER.c junto con los ring buffers que manejan.
 * Definirlas aquí causaría "duplicate ISR" en tiempo de enlace.
 */

/*
 * Timer0 CTC — Base de tiempo del sistema.
 * Se dispara cada 2 ms (F_CPU=16MHz, prescaler=256, OCR0A=124).
 * Incrementa tick_ms que usan los módulos de actuadores y debug.
 */
ISR(TIMER0_COMPA_vect) {
    tick_ms += 2;
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void) {
    cli();

    /* Inicializar periféricos */
    InitPort();
    InitUART0();
    InitTimer0();
    InitTimer1();

    /*
     * Inicializar la librería de protocolo.
     * Protocol_Init pone a cero los índices de los ring buffers
     * de Rx y Tx, y resetea la máquina de estados a WAIT_U.
     */
    Protocol_Init();

    /* Inicializar cola de cajas */
    Ev.movQ0 = 0;
    Ev.movQ1 = 0;
    Ev.movQ2 = 0;

    /* Inicializar flags de respuesta TX diferida */
    Ev.reply_send_start = 0;
    Ev.reply_send_stop  = 0;
    Ev.reply_send_reset = 0;
    Ev.reply_error      = 0;

    /* Inicializar actuadores */
    for (uint8_t i = 0; i < 3; i++) {
        Actuator[i].state        = ACT_IDLE;
        Actuator[i].timestamp_ms = 0;
    }

    /* Inicializar configuración de salidas a 0 */
    /*config_salidas[0] = 0;
    config_salidas[1] = 0;
    config_salidas[2] = 0;*/
	
	/* ============================================================
     * === ACTUALIZACIÓN: INICIALIZAR LOS SERVOS EN NUEVOS PINES ===
     * SERVO 0: PD7 (antes PD2)
     * SERVO 1: PB4 (antes PD3)
     * SERVO 2: PB3 (antes PD4)
     * ============================================================ */
    SG90_Init(&Servo[0], &PORTD, PD7); 
    SG90_Init(&Servo[1], &PORTB, PB3); 
    SG90_Init(&Servo[2], &PORTB, PB4);  
	
	for(uint8_t i = 0; i<=2; i++){
		SG90_SetAngle(&Servo[i], 130);
	}
	
	/* ============================================================
     * Inicializar el HCSR04
     * ============================================================ */
	last_sensor_trigger = 0;
	SensorCajas.wait_time_center = 0;
	SensorCajas.wait_time_center = 1000;
	SensorCajas.trigger_write = Sensor_Trig;
	SensorCajas.echo_read     = Sensor_Echo;
	SensorCajas.get_us        = Sensor_GetUs;
	SensorCajas.state         = HCSR_IDLE;
	
	/* ============================================================
     * === ACTUALIZACIÓN: INICIALIZAR LOS SENSORES IR ===
     * (Ya actualizados en las variables globales arriba)
     * IrEntry: PD2 (antes PC0)
     * IrQ0:    PD3 (antes PC1)
     * IrQ1:    PD4 (antes PC2)
     * IrQ2:    PD5 (antes PC3)
     * ============================================================ */
    TCRT5000_Init(&IrEntry);
    TCRT5000_Init(&IrQ0);
    TCRT5000_Init(&IrQ1);
    TCRT5000_Init(&IrQ2);
	
	/* === ACTUALIZACIÓN: Inicializar TRIGGER en bajo (PB1, antes PD7) === */
	PORTC |= (1 << PC0);          // INICIO: Lo ponemos en 1 para que arranque FRENADO
	
	WaitTime = 0;
	Ev.manual_timeout_enabled = 1;
    sei();
	
    SendSimuCMD(0xF0, NULL, 0); // Mandamos al comienzo de cada reset el estado IDLE
	
	Ev.hw_sensors_enabled = 1; // Establecemos como prioridad el estado de uso de hardware. 
	Ev.isMode = 1; 
	
	
    /* ============================================================
     * LOOP PRINCIPAL — Completamente no bloqueante
     * ============================================================ */
	while (1) {
		/* =========================================================
			* FASE 1: ENTRADAS (Comunicaciones y Sensores Físicos)
			* Recolectamos toda la información del mundo exterior primero.
			* ========================================================= */
		Protocol_HandleUART();     // 1.1 Leer comandos de Qt (puede cambiar modos o estados)
		HCSR04();                  // 1.2 Leer Ultrasónico e IR de entrada (Mide/Clasifica)
		HandlePhysicalIRs();       // 1.3 Leer IRs de las zonas (Si estamos en Modo IR)

		/* =========================================================
			* FASE 2: PROCESAMIENTO (Lógica y Máquinas de Estado)
			* Tomamos decisiones basados en las entradas frescas.
			* ========================================================= */
		HandleTimeMode();          // 2.1 Calcula velocidades y genera triggers virtuales (Modo Tiempo)
		HandleQueue();             // 2.2 Desplaza las cajas y decide a qué servo le toca patear

		/* =========================================================
			* FASE 3: SALIDAS (Actuadores y Respuestas UART)
			* Ejecutamos las acciones físicas y avisamos a la HMI.
			* ========================================================= */
		HandleActuators();         // 3.1 Mueve físicamente los servos (Si la FASE 2 lo ordenó)
		HandlePendingReplies();    // 3.2 Envía ACKs, telemetría y velocidades a Qt

		/* =========================================================
			* FASE 4: MANTENIMIENTO (Housekeeping / Debug)
			* Tareas no críticas.
			* ========================================================= */
		UpdateDebugLEDs();         // 4.1 Parpadeo del LED de estado en PB5
	}
}