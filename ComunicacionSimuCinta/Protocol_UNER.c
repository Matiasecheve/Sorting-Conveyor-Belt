/*
 * Protocolo_UNER.c
 */ 

#include "Protocol_UNER.h"

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
        Tx.rBuf.iw = next_iw;
        UCSR0B |= (1 << UDRIE0); 
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

void Protocol_HandleUART(void) {
    while (Rx.rBuf.ir != Rx.rBuf.iw){
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
                    // El usuario debe llamar a Protocol_DecodeCMD en el main pasando su tabla
                    // O podemos dejar esta lógica para que el main la maneje externamente
                }
                Rx.hdrst = WAIT_U;
                break;
        }
    }
}