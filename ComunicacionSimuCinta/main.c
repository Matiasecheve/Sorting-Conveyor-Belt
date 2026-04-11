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

/* ============================================================
 * INCLUDES
 * ============================================================ */
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

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

/* ============================================================
 * CONSTANTES GLOBALES
 * ============================================================
 * BUFSIZE, MASK y MAX_PAYLOAD ya vienen de Protocol_UNER.h    */
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

/*
 * rx_state_t (WAIT_U … GET_CKS) ya está definido en Protocol_UNER.h.
 * No se redefine aquí para evitar conflictos de compilación.
 */

/* Estados del actuador */
typedef enum {
    ACT_IDLE,       /* Retraído, disponible                  */
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
    /*
     * --- Respuestas TX pendientes ---
     * Los callbacks de comandos NO deben llamar SendSimuCMD directamente
     * porque se ejecutan dentro de Protocol_HandleUART (loop principal con
     * interrupciones activas). Si el simulador Qt responde inmediatamente,
     * la nueva trama entra por ISR RX mientras todavía estamos procesando
     * la anterior, saturando el pipeline.
     * Solución: el callback solo levanta un flag; HandlePendingReplies
     * despacha la respuesta en la siguiente iteración del loop principal,
     * cuando HandleUART ya terminó y el buffer RX está libre.
     */
    volatile uint8_t reply_send_start;   /* 0x50 ? arrancar cinta           */
    volatile uint8_t reply_send_stop;    /* 0x51 ? confirmar detención       */
    volatile uint8_t reply_send_reset;   /* 0x53 ? confirmar reset           */
    volatile uint8_t reply_error;        /* transitar a ST_ERROR             */
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

/* Misc */
void TogglePin(volatile uint8_t *port, uint8_t pin);

/* Hardware */
void DoStartBotton();
void DoStopBotton();
void DoResetBotton();

/* ============================================================
 * MACROS DE COMPATIBILIDAD
 * -----------------------------------------------------------------------------
 * El código de aplicación usaba TxAddChar / TxSendString / SendSimuCMD
 * directamente. Estos macros redirigen de forma transparente a la librería
 * sin tener que renombrar cada llamada en todo el archivo.
 * ============================================================ */
#define TxAddChar(d)            Protocol_TxAddChar(d)
#define TxSendString(s)         Protocol_TxSendString(s)
#define SendSimuCMD(cmd, pl, l) Protocol_SendSimuCMD(cmd, pl, l)

/* ============================================================
 * VARIABLES GLOBALES
 * ============================================================
 * Rx y Tx son extern protocol_rx_t / protocol_tx_t declarados
 * en Protocol_UNER.h y definidos en Protocol_UNER.c            */

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
 *                          QUEUE
 * ============================================================ */

void HandleQueue() {
    // --- TRAMO 0 (Entrada desde el Medidor 0x5F) ---
    if (Ev.box_entry_active && !Ev.movQ0) {
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
                Actuator[0].state = ACT_EXTENDING;
                Actuator[0].timestamp_ms = tick_ms;
				FireActuator(0,1);
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

    if (Ev.movQ2) {
        static uint8_t i2 = 1;
        Queue2[MaxQueue - i2] = Queue2[MaxQueue - i2 - 1];
        if (++i2 == MaxQueue) { Queue2[0] = 0; Ev.movQ2 = 0; i2 = 1; DebugQueues(); }
    }
}

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
void HandleTX(void) {
    /* El ring buffer de TX lo vacía la ISR USART_UDRE_vect.
     * No se requiere acción adicional en el loop principal. */
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

    //PORTC |= (1 << PC1);
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
    /*
     * NO llamamos SendSimuCMD aquí. Este callback se ejecuta dentro de
     * Protocol_HandleUART; si enviamos el 0x50 ahora, el simulador Qt
     * puede responder con la trama de Config antes de que HandleUART
     * termine, saturando el buffer RX y corrompiendo el estado de la
     * máquina de estados del protocolo.
     * Solución: levantar un flag y dejar que HandlePendingReplies
     * despache el 0x50 en la próxima iteración del loop principal.
     */
    if (Rx.payload[0] == 0x0D) {
        sys_state = ST_READY;
        Ev.reply_send_start = 1;   /* HandlePendingReplies enviará 0x50 */
    } else {
        Ev.reply_error = 1;        /* HandlePendingReplies transitará a ST_ERROR */
    }
}

/*
 * Cmd_ConfigCinta — Respuesta del simulador al START (0x50 SIMU?MICRO).
 * Payload: [v*10][boxType0][boxType1][boxType2]
 * Acción: guardar configuración y transitar a ST_RUNNING.
 */
void Cmd_ConfigCinta(void) {
    /*
     * Guardamos la config 
     */
    config_salidas[0] = Rx.payload[1];
    config_salidas[1] = Rx.payload[2];
    config_salidas[2] = Rx.payload[3];

    sys_state = ST_RUNNING;
}

/*
 * Cmd_AckStop — ACK de detención (0x51 SIMU?MICRO, payload=0x0D).
 */
void Cmd_AckStop(void) {
    sys_state = ST_READY;
}

/*
 * Cmd_AckReset — ACK de reset (0x53 SIMU?MICRO).
 * payload[0] = 0x0D ? reset OK
 * payload[0] = 0x0A ? no pudo resetearse
 */
void Cmd_AckReset(void) {
    if (Rx.payload[0] == 0x0D) {
        sys_state = ST_IDLE;
    } else {
        Ev.reply_error = 1;
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
    // 1. Guardamos el tipo de caja en la variable de resguardo
    // Rx.payload[0] tiene el 6, 8 o 10
    lastboxtype = Rx.payload[0];

    // 2. Levantamos el flag para que el while(1) sepa que hay una caja nueva
    Ev.box_entry_active = 1;
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
void HandlePendingReplies(void) {
    if (Ev.reply_error) {
        Ev.reply_error = 0;
        sys_state = ST_ERROR;
        /* Opcional: enviar trama de error al simulador si el protocolo lo define */
    }

    if (Ev.reply_send_start) {
        Ev.reply_send_start = 0;
        SendSimuCMD(0x50, NULL, 0);
    }

    if (Ev.reply_send_stop) {
        Ev.reply_send_stop = 0;
        SendSimuCMD(0x51, NULL, 0);
    }

    if (Ev.reply_send_reset) {
        Ev.reply_send_reset = 0;
        SendSimuCMD(0x53, NULL, 0);
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
        case ST_ERROR:
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
    for (uint8_t i = 0; i < MaxQueue; i++) {
        TxAddChar(Queue0[i] == 0 ? '-' : (Queue0[i] == 10 ? 'X' : Queue0[i] + '0'));
        TxAddChar(' ');
    }
    TxSendString("]\r\n");

    // Formateamos Queue 1
    TxSendString("Q1: [");
    for (uint8_t i = 0; i < MaxQueue; i++) {
        TxAddChar(Queue1[i] == 0 ? '-' : (Queue1[i] == 10 ? 'X' : Queue1[i] + '0'));
        TxAddChar(' ');
    }
    TxSendString("]\r\n");

    // Formateamos Queue 2
    TxSendString("Q2: [");
    for (uint8_t i = 0; i < MaxQueue; i++) {
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
	
	/* Configurar los puertos de los servos */
	DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4);
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

    Debounce(&StartBotton, &PINC, (1 << PC2));
    Debounce(&StopBotton,  &PINC, (1 << PC3));
    Debounce(&ResetBotton, &PINC, (1 << PC4));
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
    config_salidas[0] = 0;
    config_salidas[1] = 0;
    config_salidas[2] = 0;
	
	/* ============================================================
     * INICIALIZAR LOS SERVOS 
     * ============================================================ */
    SG90_Init(&Servo[0], &PORTD, PD2);
    SG90_Init(&Servo[1], &PORTD, PD3);
    SG90_Init(&Servo[2], &PORTD, PD4);
	
	for(uint8_t i = 0; i<=2; i++){
		SG90_SetAngle(&Servo[i], 130);
	}
	
    sei();
	
	
    SendSimuCMD(0xF0, NULL, 0); // Mandamos al comienzo de cada reset el estado IDLE

    /* ============================================================
     * LOOP PRINCIPAL — Completamente no bloqueante (mas o menos)
     * ============================================================ */
    while (1) {

        // --- TASKS ---
        Protocol_HandleUART();     /* Procesa bytes del buffer RX y llama a Protocol_DecodeCMD */
        HandlePendingReplies();    /* Despacha respuestas TX diferidas por los callbacks RX      */
        HandleActuators();
        HandleQueue();
        UpdateDebugLEDs();
    }
}
