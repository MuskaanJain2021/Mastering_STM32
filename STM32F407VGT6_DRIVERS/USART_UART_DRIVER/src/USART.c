
/*
 * USART.c
 *
 *  Created on: Aug 5, 2024
 *      Author: Muskaan Jain
 */

#include "main.h"    // Include main.h for common definitions and utility functions
#include "USART.h"
#include "GPIO.h"
#include "stm32f407xx.h"


USART_Status USART_DisableClock(USART_Config *config) {
    if (config == NULL || config->Port == NULL) {
        return USART_ERROR_INVALID_PARAM; // Error: invalid configuration
    }

    // Determine which USART peripheral is being used and disable its clock
    if (config->Port == USART1) {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN; // Disable USART1 clock
    }
    else if (config->Port == USART2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN; // Disable USART2 clock
    }
    else if (config->Port == USART3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN; // Disable USART3 clock
    }
    else if (config->Port == UART4) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN; // Disable UART4 clock
    }
    else if (config->Port == UART5) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN; // Disable UART5 clock
    }
    else if (config->Port == USART6) {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN; // Disable USART6 clock
    }
    else {
        return USART_ERROR_INVALID_USART; // Error: invalid USART peripheral
    }

    return USART_SUCCESS; // Success
}

// Function to get clock frequency based on selected clock source
static uint32_t get_clock_frequency(uint32_t clock_source) {
    switch (clock_source) {
        case CLOCK_SOURCE_APB1: return SystemAPB1_Clock_Speed();  // Assuming clock_source 0 for APB1
        case CLOCK_SOURCE_APB2: return SystemAPB2_Clock_Speed();  // Assuming clock_source 1 for APB2

        default: return 0; // Invalid clock source
    }

}

//// Function to calculate BRR value based on pclk, baudrate, and oversampling
//static uint16_t calculate_baudrate_div(uint32_t pclk, uint32_t baudrate, uint8_t over_sampling) {
//    uint16_t mantissa;
//    uint8_t fraction;
//    uint16_t brr;
//    uint32_t usartdiv_fixed;
//
//    if (over_sampling == USART_OVERSAMPLING_BY_16) { // 16x oversampling
//        // USARTDIV = pclk / (baudrate * 16)
//        // Fixed-point scaling: Multiply by 100 to preserve two decimal places
//        usartdiv_fixed = (pclk * 100U) / (baudrate * 16U);
//        mantissa = usartdiv_fixed / 100U;
//        // Add 50 for rounding (equivalent to adding 0.5 before truncation)
//        fraction = ((usartdiv_fixed % 100U) * 16U + 50U) / 100U;
//        // Ensure fraction fits within 4 bits
//        fraction &= 0x0F;
//        brr = (mantissa << 4) | fraction;
//    } else if (over_sampling == USART_OVERSAMPLING_BY_8) { // 8x oversampling
//        // USARTDIV = pclk / (baudrate * 8)
//        usartdiv_fixed = (pclk * 100U) / (baudrate * 8U);
//        mantissa = usartdiv_fixed / 100U;
//        fraction = ((usartdiv_fixed % 100U) * 8U + 50U) / 100U;
//        // Ensure fraction fits within 3 bits
//        fraction &= 0x7;
//        brr = (mantissa << 4) | fraction;
//    } else {
//        // Invalid oversampling factor; default to 16x oversampling
//        usartdiv_fixed = (pclk * 100U) / (baudrate * 16U);
//        mantissa = usartdiv_fixed / 100U;
//        fraction = ((usartdiv_fixed % 100U) * 16U + 50U) / 100U;
//        fraction &= 0xF;
//        brr = (mantissa << 4) | fraction;
//    }
//
//    return brr;
//}



static uint16_t calculate_baudrate_div(uint32_t pclk, uint32_t baudrate, uint8_t over_sampling) {
    uint32_t mantissa = 0;
    uint32_t fraction = 0;
    uint32_t brr = 0;
    uint32_t usartdiv_fixed = 0;
    uint32_t temp = 0;

    if (over_sampling == USART_OVERSAMPLING_BY_16) { // 16x oversampling

        // Fixed-point scaling: Multiply by 100 to preserve two decimal places
        usartdiv_fixed = (pclk * 100) / (baudrate * 16);
        mantissa = usartdiv_fixed / 100;
        //Extract the fraction part
        fraction  =  (usartdiv_fixed - (mantissa * 100));
        fraction = ((( fraction * 16)+ 50) / 100) & ((uint8_t)0x0F);
        brr = (mantissa << 4) | fraction;

    } else if (over_sampling == USART_OVERSAMPLING_BY_8) { // 8x oversampling


        usartdiv_fixed = (pclk * 25) / (baudrate * 2);
        mantissa = usartdiv_fixed / 100;
        temp |= (mantissa << 4);
        //Extract the fraction part
        fraction  =  (usartdiv_fixed - (mantissa * 100));
        fraction = ((( fraction * 8) + 50) / 100) & ((uint8_t)0x07);

        brr =  temp | fraction;

    } else {


    }
    brr = 0x1117;
    return brr;
}//445c

// Function to set baud rate
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t baudrate, uint32_t clock_src, USART_Oversampling  over_sampling) {
	uint32_t pclk = get_clock_frequency(clock_src);
	USARTx->BRR = calculate_baudrate_div(pclk, baudrate, over_sampling);

}
void USART_Config_Reset(USART_Config *config)
{
	config->mode = USART_Mode.Disable;
	config->hardware_flow = Hardware_Flow.Disable;
	config->baudrate = 9600;
	config->DMA_Enable = DMA_Enable.RX_Disable | DMA_Enable.TX_Disable;
	config->interrupt = DISABLE;
}
// Function to initialize GPIO pins for USART/UART
static void GPIO_Init_USART_UART(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t alt_function) {
    GPIO_PinConfig_t GpioConfig;

    // Configure GPIO pin as alternate function
    GpioConfig.GPIO_PinNumber = pin;
    GpioConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE_FUNCTION;
    GpioConfig.GPIO_PinOType = GPIO_OUTPUT_TYPE_PUSH_PULL;
    GpioConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;
    GpioConfig.GPIO_PinAltFunMode = alt_function;

    GPIO_Pin_Init(GPIOx, pin, GpioConfig.GPIO_PinMode, GpioConfig.GPIO_PinOType, GpioConfig.GPIO_PinSpeed, GpioConfig.GPIO_PinPuPdControl, GpioConfig.GPIO_PinAltFunMode);
}

// Generic USART Initialization Function
USART_Status USART_Init(USART_Config *config) {
    if (config == NULL || config->Port == NULL) {
        return USART_ERROR_INVALID_PARAM; // Error: invalid configuration
    }

    // Enable clock access to GPIO and USART
    GPIO_ClockControl(config->TX_Port, ENABLE);
    GPIO_ClockControl(config->RX_Port, DISABLE);

    if (config->Port == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (config->Port == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if (config->Port == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    } else if (config->Port == UART4) {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    } else if (config->Port == UART5) {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    } else if (config->Port == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    } else {
        return USART_ERROR_INVALID_USART; // Error: invalid USART peripheral
    }

    // Initialize GPIO pins for USART TX and RX
    GPIO_Init_USART_UART(config->TX_Port, config->TX_Pin, config->TX_Alternate_Function);
    GPIO_Init_USART_UART(config->RX_Port, config->RX_Pin, config->RX_Alternate_Function);


    //Initialise CTS and RTS pins for hardware flow control if enabled








    // Set baud rate
    uart_set_baudrate(config->Port, config->baudrate, config->clock_source, config->over_sampling);


    // Reset CR1 and CR2 registers
    config->Port->CR1 = 0;
    config->Port->CR2 = 0;

    // Configure USART mode (Tx, Rx, or both)
    if (config->mode == USART_MODE_ONLY_TX) {
        config->Port->CR1 |= USART_CR1_TE;
        config->Port->CR1 &= ~USART_CR1_RE;
    } else if (config->mode == USART_MODE_ONLY_RX) {
        config->Port->CR1 |= USART_CR1_RE;
        config->Port->CR1 &= ~USART_CR1_TE;
    } else if (config->mode == USART_MODE_ONLY_BOTH) {
        config->Port->CR1 |= USART_CR1_TE | USART_CR1_RE;
    }

    //Config Parity
    if (config->parity == USART_Parity.Even)
    {
    	config->Port->CR1 |= USART_CR1_PCE;
    	config->Port->CR1 &= ~USART_CR1_PS;
    }else if (config->parity == USART_Parity.Odd){
    	config->Port->CR1 |= USART_CR1_PCE;
    	config->Port->CR1|= USART_CR1_PS;
    }else {
    	config->Port->CR1 &= ~USART_CR1_PCE;
    }

    // Configure USART
//    config->Port->CR1 = (config->mode & 0x03) |
//                        (config->parity ? USART_CR1_PCE : 0) |
//                        (config->hardware_flow & 0x03) << 8;

    //configure stop bits
    config->stop_bits &= ~(0x3 << 12);//clear stop bits
    config->Port->CR2 |= (config->stop_bits & 0x3) << 12;


    //configure hardware flow control in CR3




//    // Enable DMA if configured
//    if (config->DMA_Enable & ) {
//        config->Port->CR3 |= USART_CR3_DMAR;
//      }
//    if (config->DMA_Enable & USART_DMA_TX_ENABLE) {
//        config->Port->CR3 |= USART_CR3_DMAT;
//      }

    //configure oversampling
    if (config->over_sampling ==USART_OVERSAMPLING_BY_8)//X 8 oversampling
    {
    	config->Port->CR1 |= USART_CR1_OVER8;
    }else{
    	config->Port->CR1 &= ~(USART_CR1_OVER8);
    }

    //Enable Interrupts if configured

    // Enable USART
    config->Port->CR1 |= USART_CR1_UE;

    //Enable DMA if configured
    return USART_SUCCESS;
}

// Function to transmit data via USART
//void USART_Transmit(USART_TypeDef *USARTx, const char* data) {
//    if (USARTx == NULL || data == NULL) {
//        return; // Error: invalid parameters
//    }
//
//    while (*data) {
//        while (!(USARTx->SR & USART_SR_TXE)); // Wait until transmit data register is empty
//        USARTx->DR = *data++; // Load the data into the data register
//    }
//    while (!(USARTx->SR & USART_SR_TC)); // Wait until transmission is complete
//}

USART_Status USART_Transmit(USART_TypeDef *USARTx, const char* data) {
    if (USARTx == NULL || data == NULL) {
        return USART_ERROR_INVALID_PARAM; // Error: invalid parameters
    }

    uint8_t is_9bit_frame = (USARTx->CR1 & USART_CR1_M) ? 1 : 0; // Check the M bit for 9-bit frame

    while (*data) {
        while (!(USARTx->SR & USART_SR_TXE)); // Wait until transmit data register is empty

        if (is_9bit_frame) {
            // For 9-bit transmission, ensure the MSB is cleared or set as needed
            uint16_t data_to_send = (uint16_t)(*data);
            USARTx->DR = data_to_send; // Transmit the 9-bit data
        } else {
            USARTx->DR = *data; // Transmit the 8-bit data
        }
        data++;
    }

    while (!(USARTx->SR & USART_SR_TC)); // Wait until transmission is complete
    return USART_SUCCESS;
}

// Function to receive data via USART
//uint16_t USART_Receive(USART_TypeDef *USARTx, char* buffer, int length) {
//    if (USARTx == NULL || buffer == NULL || length <= 0) {
//        return 0; // Error: invalid parameters
//    }
//
//    for (int i = 0; i < length; ++i) {
//        while (!(USARTx->SR & USART_SR_RXNE)); // Wait until RXNE is set
//        buffer[i] = USARTx->DR; // Receive data
//    }
//    return length; // Return number of received bytes
//}

// Function to receive data via USART
uint16_t USART_Receive(USART_TypeDef *USARTx, uint16_t* buffer, int length) {
    if (USARTx == NULL || buffer == NULL || length <= 0) {
        return 0; // Error: invalid parameters
    }

    uint8_t is_9bit_frame = (USARTx->CR1 & USART_CR1_M) ? 1 : 0; // Check the M bit for 9-bit frame

    for (int i = 0; i < length; ++i) {
        while (!(USARTx->SR & USART_SR_RXNE)); // Wait until RXNE is set (data ready)

        if (is_9bit_frame) {
            // Read 9-bit data (DR contains up to 9 bits in LSB of the 16-bit register)
            buffer[i] = (uint16_t)(USARTx->DR & 0x01FF); // Mask to extract 9 bits
        } else {
            // Read 8-bit data
            buffer[i] = (uint8_t)(USARTx->DR & 0x00FF); // Mask to extract 8 bits
        }
    }

    return length; // Return number of received frames
}


// USART2 Interrupt Handler
//void USART2_IRQHandler(void) {
//    static char rxbuffer[40];
//    static char index = 0;
//
//    if (USART2->SR & USART_SR_RXNE) {
//        rxbuffer[index++] = USART2->DR;  // Read the data
//        if (rxbuffer[index - 1] == '\n') {
//            rxbuffer[index] = '\0';  // Null-terminate the string
//            index = 0;  // Reset index for next message
//            // Process the received message (e.g., print or handle it)
//        }
//    }
//}
