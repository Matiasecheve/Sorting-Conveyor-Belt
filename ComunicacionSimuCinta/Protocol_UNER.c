/*
 * Protocolo_UNER.c
 */ 

#include "Protocol_UNER.h"
#include <avr/interrupt.h>

protocol_rx_t Rx;
protocol_tx_t Tx;

void Protocol_Init(void) {
    Rx.rBuf.iw = 0;
    Rx.rBuf.ir = 0;
    Rx.hdrst   = WAIT_U;
    Tx.rBuf.iw = 0;
    Tx.rBuf.ir = 0;
}

void Protocol_TxAddChar(uint8_t data) {
    uint8_t next_iw = (Tx.rBuf.iw + 1) & MASK;
    if (next_iw != Tx.rBuf.ir) {
        Tx.rBuf.buf[Tx.rBuf.iw] = data;
        /*
         * Orden obligatorio en AVR:
         *   1. Escribir el dato en el buffer  (ya hecho arriba).
         *   2. Actualizar iw — escritura atomica porque es uint8_t.
         *   3. Habilitar UDRIE dentro de una seccion atomica.
         *
         * Si habilitaramos UDRIE ANTES de actualizar iw, la ISR podria
         * dispararse, leer el iw viejo, ver buffer "vacio" (ir == iw_viejo)
         * y deshabilitar UDRIE, dejando el byte sin enviar para siempre.
         *
         * cli/SREG en lugar de cli/sei para no forzar interrupciones si
         * esta funcion se llama desde un contexto que ya las tenia inhibidas.
         */
        Tx.rBuf.iw = next_iw;
        uint8_t sreg = SREG;
        cli();
        UCSR0B |= (1 << UDRIE0);
        SREG = sreg;
    }
}

void Protocol_TxSendString(const char *msg) {
    while (*msg) {
        Protocol_TxAddChar((uint8_t)*msg++);
    }
}

void Protocol_SendSimuCMD(uint8_t cmd, uint8_t *payload, uint8_t len) {
    uint8_t header[] = { 'U', 'N', 'E', 'R' };
    uint8_t length_byte = 2 + len;
    uint8_t separator = ':';
    uint8_t cks = 0;

    for (uint8_t i = 0; i < 4; i++) cks ^= header[i];
    cks ^= length_byte;
    cks ^= separator;
    cks ^= cmd;
    if(payload != NULL && len > 0) {
        for (uint8_t i = 0; i < len; i++) cks ^= payload[i];
    }

    for (uint8_t i = 0; i < 4; i++) Protocol_TxAddChar(header[i]);
    Protocol_TxAddChar(length_byte);
    Protocol_TxAddChar(separator);
    Protocol_TxAddChar(cmd);
    if(payload != NULL && len > 0) {
        for (uint8_t i = 0; i < len; i++) Protocol_TxAddChar(payload[i]);
    }
    Protocol_TxAddChar(cks);
}

/* ============================================================
 * ISR UART — definidas aqui porque son las unicas que acceden
 * directamente a los ring buffers Rx.rBuf y Tx.rBuf.
 * Tenerlas en main.c creaba una dependencia oculta: cualquier
 * cambio en los tipos de buffer podia romper las ISRs sin aviso
 * del compilador. Centralizarlas en la libreria garantiza que
 * buffer y sus ISRs siempre esten sincronizados.
 *
 * El main.c NO debe definir ISR(USART_RX_vect) ni
 * ISR(USART_UDRE_vect). Solo Timer0, Timer1 y debounce.
 * ============================================================ */

/*
 * ISR RX — guarda cada byte recibido en el ring buffer de recepcion.
 * Si el buffer esta lleno (next_iw == ir) el byte se descarta;
 * es preferible perder un byte que corromper el indice iw.
 */
ISR(USART_RX_vect) {
    uint8_t data    = UDR0;
    uint8_t next_iw = (Rx.rBuf.iw + 1) & MASK;
    if (next_iw != Rx.rBuf.ir) {        /* solo encola si hay espacio */
        Rx.rBuf.buf[Rx.rBuf.iw] = data;
        Rx.rBuf.iw               = next_iw;
    }
    /* si no hay espacio el byte se descarta silenciosamente */
}

/*
 * ISR UDRE — vacia el ring buffer de TX byte a byte.
 *
 * Patron correcto para AVR:
 *   - Si hay datos: enviar el siguiente byte y avanzar ir.
 *   - Si NO hay datos: deshabilitar UDRIE *antes* de salir.
 *     Deshabilitar al final (patron incorrecto) deja una ventana
 *     donde UDRIE esta activo con buffer vacio, causando que la
 *     ISR se dispare en loop vacio y sature la CPU.
 *
 * Protocol_TxAddChar habilita UDRIE cada vez que encola un byte,
 * por lo que el ciclo se auto-sostiene mientras haya datos.
 */
ISR(USART_UDRE_vect) {
    if (Tx.rBuf.ir != Tx.rBuf.iw) {
        UDR0       = Tx.rBuf.buf[Tx.rBuf.ir];
        Tx.rBuf.ir = (Tx.rBuf.ir + 1) & MASK;
        /*
         * No deshabilitamos UDRIE aqui aunque el buffer quede vacio
         * despues de este byte: si quedaron mas bytes, la ISR se
         * volvera a disparar y los enviara. Si no quedan, la proxima
         * dispara con ir == iw y entra al else.
         */
    }
}

void Protocol_HandleUART(void) {
    /*
     * Leemos iw UNA SOLA VEZ antes del loop.
     * La ISR RX puede actualizar Rx.rBuf.iw en cualquier momento;
     * si lo leyeramos dentro del while podriamos procesar bytes
     * parcialmente escritos o nunca terminar el loop si llegan
     * bytes mas rapido de lo que los procesamos.
     * Con la snapshot, procesamos exactamente los bytes disponibles
     * al momento de entrar a HandleUART y dejamos los nuevos para
     * la proxima llamada desde el loop principal.
     */
    uint8_t iw_snapshot = Rx.rBuf.iw;
    while (Rx.rBuf.ir != iw_snapshot){
        uint8_t b  = Rx.rBuf.buf[Rx.rBuf.ir];
        Rx.rBuf.ir = (Rx.rBuf.ir + 1) & MASK;
        
        switch (Rx.hdrst) {
            case WAIT_U:
                if (b == 'U') { Rx.cks = b; Rx.hdrst = WAIT_N; }
                break;
            case WAIT_N:
                if (b == 'N') { Rx.cks ^= b; Rx.hdrst = WAIT_E; }
                else Rx.hdrst = WAIT_U;
                break;
            case WAIT_E:
                if (b == 'E') { Rx.cks ^= b; Rx.hdrst = WAIT_R; }
                else Rx.hdrst = WAIT_U;
                break;
            case WAIT_R:
                if (b == 'R') { Rx.cks ^= b; Rx.hdrst = GET_LEN; }
                else Rx.hdrst = WAIT_U;
                break;
            case GET_LEN:
                Rx.nBytes = b;
                Rx.cks ^= b;
                Rx.hdrst = WAIT_DP;
                break;
            case WAIT_DP:
                if (b == ':') { Rx.cks ^= b; Rx.hdrst = GET_CMD; }
                else Rx.hdrst = WAIT_U;
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
                    /*
                     * Checksum OK: despachar el comando.
                     * Protocol_DecodeCMD esta implementado en main.c y tiene
                     * acceso directo a la tabla de comandos de la aplicacion.
                     * Los parametros table/table_size son opcionales; la
                     * implementacion del main usa su propia tabla interna.
                     */
                    Protocol_DecodeCMD(Rx.current_cmd, NULL, 0);
                }
                /* Si el checksum falla, la trama se descarta silenciosamente */
                Rx.hdrst = WAIT_U;
                break;
        }
    }
}