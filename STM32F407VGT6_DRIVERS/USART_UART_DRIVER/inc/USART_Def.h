/*
 * USART_Def.h
 *
 *
 *  Created on: Dec 8, 2024
 *      Author: muskaan jain
 */

#ifndef USART_DEF_H_
#define USART_DEF_H_

#include "main.h"
// USART_Oversampling Enumeration
typedef enum {
    USART_OVERSAMPLING_BY_16 = 0,
    USART_OVERSAMPLING_BY_8 = 1
} USART_Oversampling;

static const struct Frame_Length
{
	uint8_t Bit_8;
	uint8_t Bit_9;
}Frame_Length = {0,1};


// Enumeration for different clock buses
typedef enum {
    CLOCK_SOURCE_APB1 = 0, // APB1 Bus
    CLOCK_SOURCE_APB2 = 1, // APB2 Bus

}ClockSource_Bus;



//USART MODES
enum{
	USART_MODE_ONLY_TX,
	USART_MODE_ONLY_RX,
	USART_MODE_ONLY_BOTH,
};
static const struct Stop_Bits
{
	uint16_t BIT_1;
	uint16_t BIT_0_5;
	uint16_t BIT_2;
	uint16_t BIT_1_5;
}Stop_Bits ={0,1<<12,2<<12,3<<12};

static const struct USART_Mode
{

	uint8_t Disable;
	uint8_t Asynchronous;
	uint8_t Synchronous;
	uint8_t Single_Wire_Half_Duplex;
	uint8_t IrDA;
	uint8_t LIN;
	uint8_t SmartCard;
	uint8_t SmartCard_Clock;
}USART_Mode = {0,1,2,3,4,5,6,7};


static const struct USART2_TX_Pin
{
	uint8_t PA2;
	uint8_t PD5;
}USART2_TX_Pin = {2,5};

static const struct USART2_RX_Pin
{
	uint8_t PA3;
	uint8_t PD6;
}USART2_RX_Pin = {3,6};

static const struct USART2_CLK_Pin
{
	uint8_t PA4;
	uint8_t PD7;
}USART2_CLK_Pin = {4,7};

static const struct USART2_CTS_Pin
{
	uint8_t PA0;
	uint8_t PD3;
}USART2_CTS_Pin = {0,3};

static const struct USART2_RTS_Pin
{
	uint8_t PA1;
	uint8_t PD4;
}USART2_RTS_Pin = {1,4};

static const struct USART_Parity
{
	uint16_t  Enable;//PCE Bit
	uint16_t Disable;//No Parity
	uint16_t Even;//PCE Enabled , PS = 0
	uint16_t Odd;//PCE Enabled , PS = 1
}USART_Parity = {.Enable=(1<<10),.Disable=0,.Even= (1<<10),.Odd = (1<<10)|(1<<9)};


static const struct DMA_Enable
{
	uint8_t TX_Enable;
	uint8_t TX_Disable;
	uint8_t RX_Enable;
	uint8_t RX_Disable;
}DMA_Enable={0,1,2,3};

static const struct Hardware_Flow
{
	uint8_t Disable;
	uint8_t CTS_Enable;
	uint8_t RTS_Enable;
	uint8_t CTS_RTS_Enable;
}Hardware_Flow={0,1,2,3};



#endif /* USART_DEF_H_ */




