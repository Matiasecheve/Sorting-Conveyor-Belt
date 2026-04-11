/*
 * Protocolo_UNER.h
 * Creado: 2026
 * Autor: Gemini AI (Basado en el desarrollo de matia)
 */ 

#ifndef PROTOCOLO_UNER_H_
#define PROTOCOLO_UNER_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* --- Configuración --- */
#define BUFSIZE         256  
#define MASK            (BUFSIZE - 1)
#define MAX_PAYLOAD     32

/* --- Enums de Estado --- */
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

/* --- Estructuras de Buffer Circular --- */
typedef struct {
    uint8_t buf[BUFSIZE];
    volatile uint8_t iw;
    volatile uint8_t ir;
} ring_buf_t;

typedef struct {
    ring_buf_t rBuf;
    rx_state_t hdrst;
    uint8_t    cks;
    uint8_t    nBytes;
    uint8_t    payloadLen;
    uint8_t    current_cmd;
    uint8_t    payload[MAX_PAYLOAD];
    uint8_t    payload_idx;
} protocol_rx_t;

typedef struct {
    ring_buf_t rBuf;
} protocol_tx_t;

/* --- Estructura para la Tabla de Comandos --- */
typedef void (*cmd_handler_t)(void);

typedef struct {
    uint8_t       cmd_id;
    cmd_handler_t execute;
} protocol_command_t;

/* --- Variables Globales Externas --- */
extern protocol_rx_t Rx;
extern protocol_tx_t Tx;

/* --- Prototipos de Funciones --- */
void Protocol_Init(void);
void Protocol_HandleUART(void);
void Protocol_TxAddChar(uint8_t data);
void Protocol_TxSendString(const char *msg);
void Protocol_SendSimuCMD(uint8_t cmd, uint8_t *payload, uint8_t len);

/* Estas funciones deben ser definidas en el main o donde se procese la lógica */
void Protocol_DecodeCMD(uint8_t cmd, const protocol_command_t *table, uint8_t table_size);

#endif /* PROTOCOLO_UNER_H_ */