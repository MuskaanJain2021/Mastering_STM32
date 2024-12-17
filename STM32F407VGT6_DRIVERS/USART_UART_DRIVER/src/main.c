#include <stm32f407xx.h>
#include "GPIO.h"
#include "USART.h"
#include <stdio.h>
#include "main.h"
USART_Config USART_CONFIG;

void USARTx_Config(USART_Config *config)
{
	 if (config == NULL)
	    return;

	config->Port = USART2;  // Assign USART peripheral
	config->baudrate = 9600;  // Baud rate configuration
	config->clock_source = CLOCK_SOURCE_APB1;// Clock source (0 for APB1, as USART2 is typically on APB1)
	//config->over_sampling = USART_OVERSAMPLING_BY_16;// Over-sampling (0 for 16x, 1 for 8x)
	config->mode = USART_MODE_ONLY_TX; // USART mode (0: Tx/Rx, 1: Rx only, 2: Tx only)
	config->parity = USART_Parity.Disable;// Parity configuration (0: None, 1: Even, 2: Odd)
	config->stop_bits = Stop_Bits.BIT_1;  // Stop bits configuration (BIT_1: 1 Stop bit, BIT_0_5: 0.5, BIT_2: 2, BIT_1_5: 1.5)
	config->hardware_flow = Hardware_Flow.Disable;// Hardware flow control (0: None, 1: RTS, 2: CTS, 3: RTS/CTS)

	// TX Pin Configuration
    config->TX_Port = GPIOA;// GPIO Port A
	config->TX_Pin = USART2_TX_Pin.PA2;  // Pin PA2 for USART2 TX
	config->TX_Alternate_Function =GPIO_AF_USART2; // Alternate Function for USART2 TX

    // RX Pin Configuration
	config->RX_Port = GPIOA;                // GPIO Port A
	config->RX_Pin = USART2_RX_Pin.PA3;     // Pin PA3 for USART2 RX
	config->RX_Alternate_Function = GPIO_AF_USART2; // Alternate Function for USART2 RX

	//CTS and RTS Pins (Not used since hardware_flow is 0)
	config->CTS_Pin = DISABLE;
    config->RTS_Pin = DISABLE;

    config->DMA_Enable = DISABLE;
    config->interrupt = DISABLE;
 }

int  main(void)
{
    // Data to be transmitted via USART
	const char data[]= "Welcome To My CUSTOM USART DRIVER";

	// Initialize system clocks and other MCU configurations
	MCU_Clock_Setup();

	//Configure USART2 settings
	USARTx_Config(&USART_CONFIG);

	// Initialize USART2 with the specified configuration
	USART_Status status = USART_Init(&USART_CONFIG);
	 if (status != USART_SUCCESS)
	 {
	        GPIO_ToggleOutputPin(GPIOD,12);
	        while(1);
	 }

	  // Main loop: Transmit data every 2 seconds
	    while(1)
	    {
	        status = USART_Transmit(USART_CONFIG.Port, data);
	        if (status != USART_SUCCESS) {
	            // Handle transmission error (e.g., reset USART)
	        	USART_Config_Reset(&USART_CONFIG);
	            status = USART_Init(&USART_CONFIG);
	            if (status != USART_SUCCESS) {
	            	GPIO_ToggleOutputPin(GPIOD,13);
	                while(1);
	            }
	        }
	        Delay_ms(2000); // Implement Delay_ms based on your system's timing

       }
 return 0;

}
