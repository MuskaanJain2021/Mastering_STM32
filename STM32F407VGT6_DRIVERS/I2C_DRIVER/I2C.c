#include "I2C.h"
#include "I2C_Def.h"
#include "GPIO.h"


/* I2C Clock Enable */
void I2C_Clock_Enable(I2C_Config *config) {
    if (config->I2Cx == I2C1) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (config->I2Cx == I2C2) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else if (config->I2Cx == I2C3) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    }
}

/* I2C Clock Disable */
void I2C_Clock_Disable(I2C_Config *config) {
    if (config->I2Cx == I2C1) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
    } else if (config->I2Cx == I2C2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
    } else if (config->I2Cx == I2C3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
    }
}

/* I2C Initialization */
void I2C_Init(I2C_Config *config) {
    I2C_Clock_Enable(config);

    /* Configure GPIO for SDA and SCL */
    if (config->I2Cx == I2C1) {
    	GPIO_Pin_Init(GPIOB, I2C1_Pins.SCL_PB6.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP, I2C1_Pins.SCL_PB6.altFunction);
    	GPIO_Pin_Init(GPIOB, I2C1_Pins.SDA_PB7.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP, I2C1_Pins.SDA_PB8.altFunction);
    } else if (config->I2Cx == I2C2) {
    	GPIO_Pin_Init(GPIOB, I2C2_Pins.SCL_PB10.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP ,I2C2_Pins.SCL_PB10.altFunction);
    	GPIO_Pin_Init(GPIOB, I2C2_Pins.SDA_PB11.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP ,I2C2_Pins.SDA_PB11.altFunction);
    } else if (config->I2Cx == I2C3) {
    	GPIO_Pin_Init(GPIOA, I2C3_Pins.SCL_PA8.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP ,I2C3_Pins.SCL_PA8.altFunction);
    	GPIO_Pin_Init(GPIOC, I2C3_Pins.SDA_PC9.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH,GPIO_PULL_UP ,I2C3_Pins.SDA_PC9.altFunction);
    }

    /* Set I2C Mode (Master/Slave) */
    if (config->mode == I2C_MODE_MASTER) {
        config->I2Cx->CR1 |= I2C_CR1_PE; // Enable peripheral
        config->I2Cx->CCR = config->speed; // Set clock speed
    } else if (config->mode == I2C_MODE_SLAVE) {
        config->I2Cx->OAR1 = (config->ownAddress << 1); // Set own address in slave mode
    }
}

/* Start condition */
void I2C_Start(I2C_TypeDef *I2Cx) {
    I2Cx->CR1 |= I2C_CR1_START;
    while (!(I2Cx->SR1 & I2C_SR1_SB)); // Wait for the start bit to be set
}

/* Stop condition */
void I2C_Stop(I2C_TypeDef *I2Cx) {
    I2Cx->CR1 |= I2C_CR1_STOP;
}

/* Write address with direction (Read/Write) */
I2C_Status I2C_WriteAddress(I2C_TypeDef *I2Cx, uint16_t address, uint8_t direction) {
    I2Cx->DR = (address << 1) | direction;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR)); // Wait until the address is sent/acknowledgemet of addr
    I2C_ClearADDRFlag(I2Cx);
    return I2C_OK;
}

/* Master Transmit */
I2C_Status I2C_Master_Transmit(I2C_Config *config, uint16_t address, const uint8_t *data, uint16_t size, uint32_t timeout) {
    I2C_Start(config->I2Cx);
    I2C_WriteAddress(config->I2Cx, address, I2C_Direction_Transmitter);
    for (uint16_t i = 0; i < size; i++) {
        config->I2Cx->DR = data[i];
        while (!(config->I2Cx->SR1 & I2C_SR1_TXE)); // Wait for TXE (Transmit buffer empty)
    }
    I2C_Stop(config->I2Cx);
    return I2C_OK;
}

/* Slave Receive-Master sending data to slave */
I2C_Status I2C_Slave_Receive(I2C_Config *config, uint8_t *data, uint16_t size, uint32_t timeout) {
    while (size--) {
        while (!(config->I2Cx->SR1 & I2C_SR1_RXNE)); // Wait for RXNE (Receive buffer not empty)
        *data++ = config->I2Cx->DR; // Read data
    }
    return I2C_OK;
}
/* Check for I2C error status */
I2C_Status I2C_CheckError(I2C_TypeDef *I2Cx) {
    if (I2Cx->SR1 & I2C_SR1_AF) {
        return I2C_ERROR_ACK_FAILURE; // Acknowledge failure
    }
    if (I2Cx->SR1 & I2C_SR1_OVR) {
        return I2C_ERROR_OVERRUN; // Overrun error
    }
    if (I2Cx->SR1 & I2C_SR1_BERR) {
        return I2C_ERROR_BUS; // Bus error
    }
    return I2C_OK; // No error
}

/* Clear the ADDR flag */
void I2C_ClearADDRFlag(I2C_TypeDef *I2Cx) {
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

/* Enable DMA for I2C */
void I2C_EnableDMA(I2C_Config *config) {
    config->I2Cx->CR2 |= I2C_CR2_DMAEN;
}

/* Disable DMA for I2C */
void I2C_DisableDMA(I2C_Config *config) {
    config->I2Cx->CR2 &= ~I2C_CR2_DMAEN;
}

void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState) {
    if (NewState == ENABLE) {
        I2Cx->CR1 |= I2C_CR1_ACK;  // Set the ACK bit in CR1 register to enable acknowledgment.
    } else {
        I2Cx->CR1 &= ~I2C_CR1_ACK; // Clear the ACK bit in CR1 register to disable acknowledgment.
    }
}


void I2C_Reset(I2C_Config *config)
{
	config->Speed_Mode = Speed_Mode.FM_Mode;
	config->Interrupts = Interrupts.Disable;
	config->mode = I2C_MODE_MASTER;
	I2C_Clock_Disable(config);

}
