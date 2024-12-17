/*
 * USART.h
 *
 *  Created on: Aug 5, 2024
 *      Author: muskaan jain
 */



#ifndef USART_H
#define USART_H

#include <USART_Def.h>
// Define USART Status Codes
typedef enum {
    USART_SUCCESS = 0,
    USART_ERROR_INVALID_PARAM = -1,
    USART_ERROR_INVALID_USART = -2,
    USART_ERROR_BUFFER_FULL = -3,
    USART_ERROR_NO_DATA = -4,

} USART_Status;

// USART Configuration Structure
typedef struct {
    USART_TypeDef *Port;                  // USART peripheral
    uint32_t baudrate;                    // Baud rate
    uint32_t clock_source;                // Clock source
    uint8_t over_sampling;                // Over-sampling (0 for 16x, 1 for 8x)
    uint8_t mode;                         // Mode (0 for Rx/Tx, 1 for Rx, 2 for Tx)
    uint8_t parity;                       // Parity (0 for none, 1 for even, 2 for odd)
    uint8_t stop_bits;                    // Stop bits (0 for 1, 1 for 0.5, 2 for 2, 3 for 1.5)
    uint8_t hardware_flow;               // Hardware flow control (0 for none, 1 for RTS, 2 for CTS, 3 for RTS/CTS)
    GPIO_TypeDef *TX_Port;               // GPIO port for TX pin
    uint8_t TX_Pin;                      // TX pin number
    uint8_t TX_Alternate_Function;       // TX alternate function number
    GPIO_TypeDef *RX_Port;               // GPIO port for RX pin
    uint8_t RX_Pin;                      // RX pin number
    uint8_t RX_Alternate_Function;       // RX alternate function number
    GPIO_TypeDef *CTS_Port;               // GPIO port for CTS pin (if hardware flow control)
    uint8_t CTS_Pin;
    GPIO_TypeDef *RTS_Port;               // GPIO port for RTS pin (if hardware flow control)
    uint8_t RTS_Pin;
    uint8_t interrupt;
    uint8_t DMA_Enable;
} USART_Config;

// Function prototypes

USART_Status USART_Init(USART_Config *config);
USART_Status USART_DisableClock(USART_Config *config);
void USART_Config_Reset(USART_Config *config);
USART_Status USART_Transmit(USART_TypeDef *USARTx, const char* data);
uint16_t USART_Receive(USART_TypeDef *USARTx, uint16_t *buffer, int length);
void USART2_IRQHandler(void);

#endif // USART_H
