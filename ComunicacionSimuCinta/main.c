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
 * 2. ESTADOS DEL SISTEMA Y DEBUG (LEDs en Puerto C / Etiquetas A0-A1)
 * -----------------------------------------------------------------------------
 * LED PC0 (A0) - Estado General:
 * - ST_IDLE    (0): OFF - Esperando Handshake (0xF0).
 * - ST_READY   (1): ON Fijo - Conectado, esperando Inicio (0x50).
 * - ST_RUNNING (2): Toggle Rápido (100ms) - Clasificación en proceso.
 * - ST_ERROR   (3): Toggle Lento (500ms) - Fallo de Reset o Comunicación.
 *
 * LED PC1 (A1) - Actividad:
 * - Pulso de 160ms al activar un actuador (0x52).
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

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdlib.h>

#include "dBounce.h"

/* ============================================================
 * CONSTANTES GLOBALES
 * ============================================================ */
#define BUFSIZE 256  
#define MASK            (BUFSIZE - 1)
#define MAX_PAYLOAD     32
#define ACT_EXTEND_MS   160     /* Margen sobre los 150 ms del actuador   */
#define DEBUG_FAST_MS   100     /* Periodo toggle PA0 en ST_RUNNING       */
#define MaxQueue        10      /* Maximo número de cajas en cinta        */
/* ============================================================
 * ENUMS
 * ============================================================ */

typedef enum {
    ST_IDLE,        /* Esperando conexión (enviamos 0xF0 periódicamente) */
    ST_READY,       /* Conectado, esperando START (0x50)                  */
    ST_RUNNING,     /* Clasificando cajas                                 */
    ST_ERROR        /* Fallo o Reset                                      */
} system_state_t;

typedef enum {
    WAIT_U,
    WAIT_N,
    WAIT_E,
    WAIT_R,
    GET_LEN,
    WAIT_DP,
    GET_CMD,
    GET_PAYLOAD,
    GET_CKS
} rx_state_t;

/* Estados del actuador */
typedef enum {
    ACT_IDLE,       /* Retraído, disponible                  */
    ACT_EXTENDING,  /* Extendido, esperando para retraer     */
    ACT_RETRACTING  /* Retrayendo, esperando confirmación    */
} actuator_state_t;

/* ============================================================
 * ESTRUCTURAS
 * ============================================================ */

typedef struct {
    uint8_t buf[BUFSIZE];
    volatile uint8_t iw;
    volatile uint8_t ir;
} _sRingBuf;

typedef struct {
    _sRingBuf  rBuf;
    rx_state_t hdrst;
    uint8_t    cks;
    uint8_t    nBytes;
    uint8_t    payloadLen;
    uint8_t    current_cmd;
    uint8_t    payload[MAX_PAYLOAD];
    uint8_t    payload_idx;
} _sRx;

typedef struct {
    _sRingBuf rBuf;
    const char *pMsg;
} _sTx;


/* Estado de cada actuador */
typedef struct {
    actuator_state_t state;
    uint32_t         timestamp_ms;   /* Momento en que comenzó el estado actual */
} _sActuator;

typedef void (*cmd_handler_t)(void);

typedef struct {
    uint8_t       cmd_id;
    cmd_handler_t execute;
} _sCommand;

/* ============================================================
 * BANDERAS DE EVENTOS (FLAGS)
 * ============================================================ */
typedef struct {
	/* --- Eventos de Clasificación (Los que SI usás) --- */
	volatile uint8_t box_entry_active; // Entrada desde el medidor
	volatile uint8_t ir0_active;       // Sensores físicos
	volatile uint8_t ir1_active;
	volatile uint8_t ir2_active;

	/* --- Control de Tareas Multitask --- */
	volatile uint8_t movQ0;            // Semáforos de movimiento
	volatile uint8_t movQ1;
	volatile uint8_t movQ2;
	
	volatile uint8_t box_entry_Q1;     // Sala de espera entre zonas
	volatile uint8_t box_entry_Q2;
} _sEventFlags;

/* ============================================================
 * PROTOTIPOS
 * ============================================================ */
void InitUART0(void);
void InitTimer0(void);
void InitTimer1(void);
void InitPort(void);

/* TX */
void TxAddChar(uint8_t data);
void TxSendString(const char *msg);
void HandleTX(void);
void SendSimuCMD(uint8_t cmd, uint8_t *payload, uint8_t len);

/* RX / Protocolo */
void HandleUART(void);
void DecodeCMD(uint8_t cmd);

/* Lógica de clasificación */
//void ClassifyBox(uint8_t outNum);
void FireActuator(uint8_t outNum, uint8_t extend);
void HandleActuators(void);

/* Cola */
void HandleQueue();

/* Debug */
void UpdateDebugLEDs(void);
void DebugQueues(void);

/* Misc */
void TogglePin(volatile uint8_t *port, uint8_t pin);

/* Hardware */ 
void DoStartBotton();
void DoStopBotton();
void DoResetBotton();


/* ============================================================
 * VARIABLES GLOBALES
 * ============================================================ */
_sRx         Rx;
_sTx         Tx;

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

/* Última caja medida recibida (0x5F), guardada en la cola */

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

/* Manejo del debounce de los botones */

debounce_t StartBotton = {
	.pressed_count = 0,
	.prev_state = 1,
	.onPress = DoStartBotton,
	.onRelease = NULL
};

debounce_t StopBotton = {
	.pressed_count = 0,
	.prev_state = 1,
	.onPress = DoStopBotton,
	.onRelease = NULL
};

debounce_t ResetBotton = {
	.pressed_count = 0,
	.prev_state = 1,
	.onPress = DoResetBotton,
	.onRelease = NULL
};
/* ============================================================
							QUEUE
 * ============================================================ */

void HandleQueue() {
	// --- TRAMO 0 (Entrada desde el Medidor 0x5F) ---
	if (Ev.box_entry_active && !Ev.movQ0) {
		if (Qelements0 < MaxQueue) {
			Queue0[MaxQueue - 1 - Qelements0] = lastboxtype;
			Qelements0++;
			Ev.box_entry_active = 0;
			//DebugQueues();
		}
	}

	if (Ev.ir0_active && !Ev.movQ0) {
		Ev.ir0_active = 0;
		if (Qelements0 > 0) {
			if (Queue0[MaxQueue - 1] == config_salidas[0]) {
				Actuator[0].state = ACT_EXTENDING;
				Actuator[0].timestamp_ms = tick_ms;
				FireActuator(0, 1);
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
				Actuator[1].state = ACT_EXTENDING;
				Actuator[1].timestamp_ms = tick_ms;
				FireActuator(1, 1);
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
				Actuator[2].state = ACT_EXTENDING;
				Actuator[2].timestamp_ms = tick_ms;
				FireActuator(2, 1);
			}
			// Para Queue2, si no se patea, simplemente se elimina (descarte)
			// No hay Queue3, así que el lugar queda vacío.
			Queue2[MaxQueue - 1] = 0;
			Qelements2--;
			Ev.movQ2 = 1; // Inicia tarea de desplazamiento en Q2
		}
	}

	// --- TAREAS DE DESPLAZAMIENTO (SHIFTING SERIALIZADO) ---)
	
	if (Ev.movQ0) {
		static uint8_t i0 = 1;
		Queue0[MaxQueue - i0] = Queue0[MaxQueue - i0 - 1];
		if (++i0 == MaxQueue) { Queue0[0] = 0; Ev.movQ0 = 0; i0 = 1; /*DebugQueues();*/ }
	}

	if (Ev.movQ1) {
		static uint8_t i1 = 1;
		Queue1[MaxQueue - i1] = Queue1[MaxQueue - i1 - 1];
		if (++i1 == MaxQueue) { Queue1[0] = 0; Ev.movQ1 = 0; i1 = 1; /*DebugQueues();*/ }
	}

	if (Ev.movQ2) {
		static uint8_t i2 = 1;
		Queue2[MaxQueue - i2] = Queue2[MaxQueue - i2 - 1];
		if (++i2 == MaxQueue) { Queue2[0] = 0; Ev.movQ2 = 0; i2 = 1; /*DebugQueues();*/ }
	}
}

/* ============================================================
 * IMPLEMENTACIÓN — TX
 * ============================================================ */

void TxAddChar(uint8_t data) {
    uint8_t next_iw = (Tx.rBuf.iw + 1) & MASK;
    if (next_iw != Tx.rBuf.ir) {
        Tx.rBuf.buf[Tx.rBuf.iw] = data;
        Tx.rBuf.iw = next_iw;
        UCSR0B |= (1 << UDRIE0);   /* Habilita interrupción UDRE para vaciar buffer */
    }
}

/*
 * TxSendString — Agrega una cadena ASCII directamente al ring buffer de TX.
 * Usado para respuestas de texto plano (debug, ACKs de texto).
 */
void TxSendString(const char *msg) {
    while (*msg) {
        TxAddChar((uint8_t)*msg++);
    }
}

/*
 * HandleTX — Vacía el pMsg pendiente (modo streaming char a char).
 * Se llama desde el loop principal. Permite enviar strings largos
 * sin bloquear el CPU.
 */
void HandleTX(void) {
    if (Tx.pMsg == NULL) return;
    if (*Tx.pMsg != '\0') {
        uint8_t next_iw = (Tx.rBuf.iw + 1) & MASK;
        if (next_iw != Tx.rBuf.ir) {
            TxAddChar((uint8_t)*Tx.pMsg);
            Tx.pMsg++;
        }
    } else {
        Tx.pMsg = NULL;
    }
}

/*
 * SendSimuCMD — Arma y envía una trama UNER completa al simulador.
 *
 * Formato de trama:
 *   [U][N][E][R][LEN][:][CMD][payload[0]...payload[len-1]][CKS]
 *
 * Donde:
 *   LEN = 1 (CMD) + len (bytes de payload)
 *   CKS = XOR de todos los bytes desde 'U' hasta el último payload
 *
 * @param cmd     Byte de comando a enviar (ej: 0xF0, 0x52)
 * @param payload Puntero al array de bytes del payload (puede ser NULL si len=0)
 * @param len     Cantidad de bytes de payload
 */
void SendSimuCMD(uint8_t cmd, uint8_t *payload, uint8_t len) {
	uint8_t header[] = { 'U', 'N', 'E', 'R' };
	uint8_t length_byte = 2 + len; // Base 2 (Token + CMD)
	uint8_t separator = ':';
	uint8_t cks = 0;

	// Checksum: XOR desde 'U' hasta el último byte del payload 
	for (uint8_t i = 0; i < 4; i++) cks ^= header[i];
	cks ^= length_byte;
	cks ^= separator;
	cks ^= cmd;
	if(payload != NULL && len > 0) {
		for (uint8_t i = 0; i < len; i++) cks ^= payload[i];
	}

	// Transmisión física
	for (uint8_t i = 0; i < 4; i++) TxAddChar(header[i]);
	TxAddChar(length_byte);
	TxAddChar(separator);
	TxAddChar(cmd);
	if(payload != NULL && len > 0) {
		for (uint8_t i = 0; i < len; i++) TxAddChar(payload[i]);
	}
	TxAddChar(cks);
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
	
	PORTC |= (1 << PC1);
}

/*
 * HandleActuators — Máquina de estados para los 3 actuadores.
 * Se llama desde el loop principal (NO bloqueante).
 * Usa tick_ms para medir los 150 ms de transición.
 */
void HandleActuators(void) {
    for (uint8_t i = 0; i < 3; i++) {
        switch (Actuator[i].state) {

            case ACT_IDLE:
                /* Nada que hacer, esperando orden */
                break;

            case ACT_EXTENDING:
                /* Esperar ACT_EXTEND_MS ms y luego retraer */
				if ((tick_ms - Actuator[i].timestamp_ms) >= ACT_EXTEND_MS) {
					Actuator[i].state = ACT_RETRACTING;
					Actuator[i].timestamp_ms = tick_ms;
					FireActuator(i, 0); // Orden de retracción física

					
					/* --- MENSAJE DE DEBUG PARA HÉRCULES --- */
					/*
					TxSendString("<<< RETRAYENDO BRAZO: ");
					TxAddChar(i + '0');
					TxSendString("\r\n");
					*/
				}
                break;

            case ACT_RETRACTING:
                /* Esperar otro ACT_EXTEND_MS ms para considerar el brazo disponible */
                if ((tick_ms - Actuator[i].timestamp_ms) >= ACT_EXTEND_MS) {
                    Actuator[i].state = ACT_IDLE;
                    PORTC &= ~(1 << PC1);     /* Apagar LED de clasificación */
                }
                break;
        }
    }
}

/* ============================================================
 * CALLBACKS DE COMANDOS (RX desde el simulador)
 * ============================================================ */

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
	// Verificamos el payload según el manual
	if (Rx.payload[0] == 0x0D) {
		sys_state = ST_READY; // Ahora sí, el micro sabe que hay alguien del otro lado
		
		// El paso lógico siguiente: Encender cinta (0x50)
		SendSimuCMD(0x50, NULL, 0);
		} else {
		// Si responde otra cosa, es un error de protocolo
		sys_state = ST_ERROR;
	}
}

/*
 * Cmd_ConfigCinta — Respuesta del simulador al START (0x50 SIMU?MICRO).
 * Payload: [v*10][boxType0][boxType1][boxType2]
 * Acción: guardar configuración y transitar a ST_RUNNING.
 */
void Cmd_ConfigCinta(void) {
	config_salidas[0] = Rx.payload[1];
	config_salidas[1] = Rx.payload[2];
	config_salidas[2] = Rx.payload[3];

	sys_state = ST_RUNNING;
	
	// AGREGAR ESTA LÍNEA: Avisar a la PC que la cinta arrancó
	SendSimuCMD(0x50, NULL, 0);
}

/*
 * Cmd_AckStop — ACK de detención (0x51 SIMU?MICRO, payload=0x0D).
 */
void Cmd_AckStop(void) {
	sys_state = ST_READY; // El micro cambia su estado interno
	
	// Le enviamos el comando 0x51 de vuelta a la PC como confirmación
	SendSimuCMD(0x51, NULL, 0);
}

/*
 * Cmd_AckReset — ACK de reset (0x53 SIMU?MICRO).
 * payload[0] = 0x0D ? reset OK
 * payload[0] = 0x0A ? no pudo resetearse
 */
void Cmd_AckReset(void) {
	if (Rx.payload[0] == 0x0D) {
		sys_state = ST_IDLE;
		
		// --- AGREGAR ESTA LÍNEA ---
		// Avisamos a la PC que el reset fue exitoso
		SendSimuCMD(0x53, NULL, 0);
		//DoResetBotton();
		} else {
		sys_state = ST_ERROR;
	}
}

/*
 * Cmd_AckActuador — ACK del simulador al CMD 0x52.
 * payload[0] = 0xFF ? OK
 * No se necesita acción adicional; el timing lo gestiona HandleActuators.
 */
void Cmd_AckActuador(void) {
    /* ACK recibido, la FSM de actuadores maneja el resto */
}

/*
 * Cmd_AckVelocidad — ACK del simulador al CMD 0x54.
 * payload[0] = v*10, payload[1] = 0x0D
 */
void Cmd_AckVelocidad(void) {
    /* ACK de velocidad, se podría actualizar una variable local si fuera necesario */
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
	uint8_t i = 0;

	// Recorremos TODO el payload de a pares [outNum, irState]
	while (i + 1 < Rx.payloadLen) {
		uint8_t outNum   = Rx.payload[i];
		uint8_t irState  = Rx.payload[i + 1];
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
	// 1. Guardamos el tipo de caja en la variable de resguardo
	// Rx.payload[0] tiene el 6, 8 o 10
	lastboxtype = Rx.payload[0];

	// 2. Levantamos el flag para que el while(1) sepa que hay una caja nueva
	Ev.box_entry_active = 1;
}

/* ============================================================
 * TABLA DE COMANDOS
 * ============================================================ */
const _sCommand command_table[] = {
    { 0xF0, Cmd_AckAlive      },   /* ACK conexión del simulador            */ 
    { 0x50, Cmd_ConfigCinta   },   /* ACK inicio + configuración de salidas */
    { 0x51, Cmd_AckStop       },   /* ACK detención                         */
    { 0x52, Cmd_AckActuador   },   /* ACK activación de actuador            */
    { 0x53, Cmd_AckReset      },   /* ACK reset                             */
    { 0x54, Cmd_AckVelocidad  },   /* ACK cambio de velocidad               */
    { 0x5E, Cmd_SensorEvent   },   /* Evento de sensor IR                   */
    { 0x5F, Cmd_NuevaCaja     },   /* Nueva caja medida                     */
};
#define MAX_COMMANDS (sizeof(command_table) / sizeof(_sCommand))

/* ============================================================
 * DECODIFICADOR DE COMANDOS
 * ============================================================ */
void DecodeCMD(uint8_t cmd) {
    for (uint8_t i = 0; i < MAX_COMMANDS; i++) {
        if (command_table[i].cmd_id == cmd) {
            if (command_table[i].execute) command_table[i].execute();
            return;
        }
    }
    /* Comando desconocido — enviar respuesta de error en texto */
    TxSendString("UNER:Cmd_Unknown\n");
}

/* ============================================================
 *							HARDWARE
 * ============================================================ */

void DoStartBotton(){
	SendSimuCMD(0xF0, NULL, 0); // CMD 0xF0, sin puntero de datos, longitud
}

void DoStopBotton(){
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
	}
	else {
		// Si estamos en ST_IDLE o ST_ERROR, no hacemos nada
		TxSendString("\r\n[?] STOP ignorado: No hay conexion activa.\r\n");
	}
}
void DoResetBotton(){
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
	for (uint8_t i = 0; i < 3; i++){
		Actuator[i].state        = ACT_IDLE;
		Actuator[i].timestamp_ms = 0;
	}

	// 7. Volver al estado de espera inicial
	sys_state = ST_IDLE;


	TxSendString("\r\n[!] RESET INTEGRAL: Software y Hardware limpios.\r\n");
}



/* ============================================================
 * INTERRUPCIONES
 * ============================================================ */

/* RX UART — guarda byte en ring buffer de recepción */
ISR(USART_RX_vect) {
    uint8_t data    = UDR0;
    uint8_t next_iw = (Rx.rBuf.iw + 1) & MASK;
    if (next_iw != Rx.rBuf.ir) {
        Rx.rBuf.buf[Rx.rBuf.iw] = data;
        Rx.rBuf.iw               = next_iw;
    }
}

/* UDRE — vacía el ring buffer de TX byte a byte */
ISR(USART_UDRE_vect) {
    if (Tx.rBuf.ir != Tx.rBuf.iw) {
        UDR0        = Tx.rBuf.buf[Tx.rBuf.ir];
        Tx.rBuf.ir  = (Tx.rBuf.ir + 1) & MASK;
    } else {
        UCSR0B &= ~(1 << UDRIE0);   /* Buffer vacío ? deshabilitar interrupción */
    }
}

/*
 * Timer0 CTC — Base de tiempo del sistema.
 * Se dispara cada 2 ms (F_CPU=16MHz, prescaler=256, OCR0A=124).
 * Incrementa tick_ms que usan los módulos de actuadores y debug.
 */
ISR(TIMER0_COMPA_vect) {
    tick_ms += 2;
	
	Debounce(&StartBotton, &PINC, (1 << PC2));
	Debounce(&StopBotton, &PINC, (1 << PC3));
	Debounce(&ResetBotton,&PINC, (1<<PC4));
}

/*
 * Timer1 OVF — Heartbeat LED en PB5.
 * Prescaler=8 ? OVF cada ~32 ms. 30 OVFs ? 960 ms ? 1 Hz.
 */
ISR(TIMER1_OVF_vect) {
    ovf_counter_hb++;
    if (ovf_counter_hb >= 30) {
        PORTB       ^= (1 << PB5);   /* Toggle LED heartbeat */
        ovf_counter_hb = 0;
    }
}

/* ============================================================
 * PROTOCOLO RX — MÁQUINA DE ESTADOS
 * ============================================================ */
void HandleUART(void) {
	// Procesamos TODO el buffer de una, no de a un byte
	while (Rx.rBuf.ir != Rx.rBuf.iw){
		uint8_t b  = Rx.rBuf.buf[Rx.rBuf.ir];
		Rx.rBuf.ir = (Rx.rBuf.ir + 1) & MASK;
		
		switch (Rx.hdrst) {
			case WAIT_U:
				if (b == 'U') { Rx.cks = b; Rx.hdrst = WAIT_N; }
			break;
			case WAIT_N:
				if (b == 'N') { Rx.cks ^= b; Rx.hdrst = WAIT_E; }
						else           { Rx.hdrst = WAIT_U; }
			break;
			case WAIT_E:
				if (b == 'E') { Rx.cks ^= b; Rx.hdrst = WAIT_R; }
						else           { Rx.hdrst = WAIT_U; }
			break;
			case WAIT_R:
				if (b == 'R') { Rx.cks ^= b; Rx.hdrst = GET_LEN; }
				else           { Rx.hdrst = WAIT_U; }
			break;
			case GET_LEN:
				Rx.nBytes  = b;
				Rx.cks    ^= b;
				Rx.hdrst   = WAIT_DP;
			break;
			case WAIT_DP:
				if (b == ':') { Rx.cks ^= b; Rx.hdrst = GET_CMD; }
				else           { Rx.hdrst = WAIT_U; }
			break;
			case GET_CMD:
				Rx.current_cmd = b;
				Rx.cks ^= b;
				Rx.payloadLen = (Rx.nBytes >= 2) ? (Rx.nBytes - 2) : 0;
				Rx.payload_idx = 0;
				Rx.hdrst = (Rx.payloadLen > 0) ? GET_PAYLOAD : GET_CKS;
			break;
			case GET_PAYLOAD:
				if (Rx.payload_idx < MAX_PAYLOAD) {
					Rx.payload[Rx.payload_idx] = b;
				}
				Rx.payload_idx++;
				Rx.cks ^= b;
				if (Rx.payload_idx >= Rx.payloadLen) Rx.hdrst = GET_CKS;
			break;
			case GET_CKS:
				if (b == Rx.cks) {
					DecodeCMD(Rx.current_cmd);
				}
				/* Si el checksum falla, la trama se descarta silenciosamente */
				Rx.hdrst = WAIT_U;
			break;
		}
	}
}

/* ============================================================
 * DEBUG LEDs EN PUERTO A
 *   PA0 — Estado del sistema
 *   PA1 — Actividad de actuadores (manejado en FireActuator/HandleActuators)
 * ============================================================ */
void UpdateDebugLEDs(void) {
	switch (sys_state) {
		case ST_IDLE:
			// Un parpadeo muy tenue (cada 2 segundos) para saber que el micro vive
			if ((tick_ms % 2000) < 100) PORTC |= (1 << PC0);
			else PORTC &= ~(1 << PC0);
		break;	
		case ST_READY:
			PORTC |= (1 << PC0); // Fijo cuando hay conexión
		break;
		case ST_RUNNING:
			if ((tick_ms - debug_led_ts) >= DEBUG_FAST_MS) {
				PORTC ^= (1 << PC0); // Toggle rápido clasificando
				debug_led_ts = tick_ms;
			}
		break;
	}
}
/* ============================================================
 * DEBUG UART EN PUERTO A
 *   PA0 — Estado del sistema
 *   PA1 — Actividad de actuadores (manejado en FireActuator/HandleActuators)
 * ============================================================ */
void DebugQueues(void) {
	
	TxSendString("\r\n--- ESTADO DE CINTA ---\r\n");
	
	// Formateamos Queue 0
	TxSendString("Q0: [");
	for(uint8_t i=0; i<MaxQueue; i++) {
		TxAddChar(Queue0[i] == 0 ? '-' : (Queue0[i] == 10 ? 'X' : Queue0[i] + '0'));
		TxAddChar(' ');
	}
	TxSendString("]\r\n");

	// Formateamos Queue 1
	TxSendString("Q1: [");
	for(uint8_t i=0; i<MaxQueue; i++) {
		TxAddChar(Queue1[i] == 0 ? '-' : (Queue1[i] == 10 ? 'X' : Queue1[i] + '0'));
		TxAddChar(' ');
	}
	TxSendString("]\r\n");

	// Formateamos Queue 2
	TxSendString("Q2: [");
	for(uint8_t i=0; i<MaxQueue; i++){
		TxAddChar(Queue2[i] == 0 ? '-' : (Queue2[i] == 10 ? 'X' : Queue2[i] + '0'));
		TxAddChar(' ');
	}
	TxSendString("]\r\n-----------------------\r\n");
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

void InitPort(void) {
	/* PB5 — Heartbeat LED (Salida) */
	DDRB |= (1 << PB5);

	/* PC0 y PC1 como salidas (LEDs de estado y actuadores) */
	DDRC |= (1 << PC0) | (1 << PC1);
	PORTC &= ~((1 << PC0) | (1 << PC1));

	/* --- CONFIGURACIÓN BOTONES / ENTRADAS --- */
	/* 1. Configurar PC2, PC3 y PC4 como entradas (DDR = 0) */
	DDRC &= ~((1 << PC2) | (1 << PC3) | (1 << PC4));
	
	/* 2. Activar Pull-ups internas (PORT = 1 mientras DDR = 0) */
	PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4);
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
    TIMSK1 = (1 << TOIE1);          /* Habilitar OVF ISR */
}

/* ============================================================
 * FUNCIÓN AUXILIAR
 * ============================================================ */
void TogglePin(volatile uint8_t *port, uint8_t pin) {
    *port ^= (1 << pin);
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

    /* Inicializar estructuras RX/TX */
	
    Rx.rBuf.iw   = 0;
    Rx.rBuf.ir   = 0;
    Rx.hdrst     = WAIT_U;
    Tx.rBuf.iw   = 0;
    Tx.rBuf.ir   = 0;
    Tx.pMsg      = NULL;

    /* Inicializar cola de cajas */
	Ev.movQ0 = 0;
	Ev.movQ1 = 0;
	Ev.movQ2 = 0; 
    /* Inicializar actuadores */
    for (uint8_t i = 0; i < 3; i++){
        Actuator[i].state        = ACT_IDLE;
        Actuator[i].timestamp_ms = 0;
    }

    /* Inicializar configuración de salidas a 0 */
    config_salidas[0] = 0;
    config_salidas[1] = 0;
    config_salidas[2] = 0;
	
    sei();
	
	SendSimuCMD(0xF0, NULL, 0); // mandamos al comienzo de cada reset el estado IDLE 
	
    /* ============================================================
     * LOOP PRINCIPAL — Completamente no bloqueante (mas o menos) 
     * ============================================================ */

	while (1) {
	   
		// --- TASKS ---
	   
		HandleUART();
		HandleTX();
		HandleActuators();
		HandleQueue();
		UpdateDebugLEDs();
	}
}