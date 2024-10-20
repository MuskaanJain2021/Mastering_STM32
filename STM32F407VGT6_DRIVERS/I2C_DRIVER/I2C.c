/*
 * I2C.c
 *
 *
 *Author: muskaan jain
 */

#include "I2C.h"
#include "I2C_Def.h"
#include "GPIO.h"

/**
 * @brief  Enables the clock for the specified I2C peripheral.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral.
 * @note   This function sets the corresponding bit in the APB1ENR register
 *         to enable the clock for the selected I2C peripheral (I2C1, I2C2, or I2C3).
 */
void I2C_Clock_Enable(I2C_Config *config)
{
    if (config->I2Cx == I2C1)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    }
    else if (config->I2Cx == I2C2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    }
    else if (config->I2Cx == I2C3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    }
}
/**
 * @brief  Disables the clock for the specified I2C peripheral.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral.
 * @note   This function clears the corresponding bit in the APB1ENR register
 *         to disable the clock for the selected I2C peripheral (I2C1, I2C2, or I2C3).
 */
void I2C_Clock_Disable(I2C_Config *config)
{
    if (config->I2Cx == I2C1)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
    }
    else if (config->I2Cx == I2C2)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
    }
    else if (config->I2Cx == I2C3)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
    }
}

/**
 * @brief  Initializes the I2C peripheral based on the provided configuration.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral, including mode and GPIO settings.
 * @note   This function sets up the I2C mode (Master/Slave), configures the GPIO pins
 *         for SCL and SDA, and enables the I2C peripheral.
 */
void I2C_Init(I2C_Config *config)
{
    I2C_Clock_Enable(config);

    /* Configure GPIO for SDA and SCL */
    if (config->I2Cx == I2C1)
    {
        GPIO_Pin_Init(GPIOB, I2C1_Pins.SCL_PB6.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C1_Pins.SCL_PB6.altFunction);
        GPIO_Pin_Init(GPIOB, I2C1_Pins.SDA_PB7.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C1_Pins.SDA_PB8.altFunction);
    }
    else if (config->I2Cx == I2C2)
    {
        GPIO_Pin_Init(GPIOB, I2C2_Pins.SCL_PB10.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C2_Pins.SCL_PB10.altFunction);
        GPIO_Pin_Init(GPIOB, I2C2_Pins.SDA_PB11.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C2_Pins.SDA_PB11.altFunction);
    }
    else if (config->I2Cx == I2C3)
    {
        GPIO_Pin_Init(GPIOA, I2C3_Pins.SCL_PA8.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C3_Pins.SCL_PA8.altFunction);
        GPIO_Pin_Init(GPIOC, I2C3_Pins.SDA_PC9.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C3_Pins.SDA_PC9.altFunction);
    }

    /* Set I2C Mode (Master/Slave) */
    if (config->mode == I2C_MODE_MASTER)
    {
        config->I2Cx->CR1 |= I2C_CR1_PE;   // Enable peripheral
        config->I2Cx->CCR = config->speed; // Set clock speed
    }
    else if (config->mode == I2C_MODE_SLAVE)
    {
        config->I2Cx->OAR1 = (config->ownAddress << 1); // Set own address in slave mode
    }
}

/**
 * @brief  Generates an I2C start condition.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function sets the START bit in the CR1 register to initiate
 *         communication and waits until the start condition is generated.
 */
void I2C_Start(I2C_TypeDef *I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_START; /* Set the START bit to begin communication */
    while (!(I2Cx->SR1 & I2C_SR1_SB))
        ; // Wait for the start bit to be set
}

/**
 * @brief  Generates an I2C stop condition.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function sets the STOP bit in the CR1 register to terminate
 *         communication after a transmission or reception.
 */
void I2C_Stop(I2C_TypeDef *I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_STOP; /* Set the STOP bit to end communication */
}

/**
 * @brief  Sends the I2C address with the read/write direction.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @param  address: 7-bit address of the I2C device.
 * @param  direction: Direction of communication (Read/Write).
 * @return I2C_Status: Returns the status of the operation (I2C_OK, I2C_ERROR).
 * @note   This function sends the 7-bit address along with the direction (R/W bit)
 *         and waits until the address is acknowledged by the slave.
 */
I2C_Status I2C_WriteAddress(I2C_TypeDef *I2Cx, uint16_t address, uint8_t direction)
{
    I2Cx->DR = (address << 1) | direction; /* Send the 7-bit address and the direction bit */
    while (!(I2Cx->SR1 & I2C_SR1_ADDR))
        ;                    // Wait until the address is sent/acknowledgemet of addr
    I2C_ClearADDRFlag(I2Cx); // Clear the ADDR flag
    return I2C_OK;
}

/**
 * @brief  Transmits data as a master to the specified slave.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral.
 * @param  address: 7-bit address of the slave device.
 * @param  data: Pointer to the data to be transmitted.
 * @param  size: Size of the data buffer to be transmitted.
 * @param  timeout: Timeout value for the transmission.
 * @return I2C_Status: Returns the status of the transmission (I2C_OK, I2C_ERROR).
 * @note   This function transmits data from the master to the specified
 *         slave by writing data byte-by-byte to the I2C data register (DR).
 */
I2C_Status I2C_Master_Transmit(I2C_Config *config, uint16_t address, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    I2C_Start(config->I2Cx);
    I2C_WriteAddress(config->I2Cx, address, I2C_Direction_Transmitter);
    for (uint16_t i = 0; i < size; i++)
    {
        config->I2Cx->DR = data[i];
        while (!(config->I2Cx->SR1 & I2C_SR1_TXE))
            ; // Wait for TXE (Transmit buffer empty)
    }
    I2C_Stop(config->I2Cx);
    return I2C_OK;
}

/**
 * @brief  Slave mode receive function.
 * @param  config: Pointer to I2C_Config structure containing the configuration.
 * @param  data: Pointer to buffer to store received data.
 * @param  size: Number of bytes to receive.
 * @param  timeout: Timeout value for reception.
 * @retval I2C_Status: Reception status (OK or error).
 */
I2C_Status I2C_Slave_Receive(I2C_Config *config, uint8_t *data, uint16_t size, uint32_t timeout)
{
    while (size--)
    {
        while (!(config->I2Cx->SR1 & I2C_SR1_RXNE))
            ;                       // Wait for RXNE (Receive buffer not empty)
        *data++ = config->I2Cx->DR; // Read data
    }
    return I2C_OK;
}
/**
 * @brief  Checks the I2C peripheral for any error conditions.
 * @param  I2Cx: Pointer to I2C peripheral.
 * @retval I2C_Status: Status of the I2C error (OK or specific error code).
 */
I2C_Status I2C_CheckError(I2C_TypeDef *I2Cx)
{
    if (I2Cx->SR1 & I2C_SR1_AF)
    {
        return I2C_ERROR_ACK_FAILURE; // Acknowledge failure
    }
    if (I2Cx->SR1 & I2C_SR1_OVR)
    {
        return I2C_ERROR_OVERRUN; // Overrun error
    }
    if (I2Cx->SR1 & I2C_SR1_BERR)
    {
        return I2C_ERROR_BUS; // Bus error
    }
    return I2C_OK; // No error
}

/**
 * @brief Clear the ADDR flag in the I2C status registers.
 *
 * This function is used to clear the Address Sent (ADDR) flag in the
 * I2C status register after the address has been acknowledged.
 * It reads the status register to clear the ADDR flag. This is
 * necessary to avoid any further interruptions in communication.
 *
 * @param I2Cx Pointer to the I2C peripheral base address.
 *              This should be one of the I2C instances (I2C1, I2C2, or I2C3).
 *
 * @note
 *  - The ADDR flag is set when an address is sent by the master or a slave address is matched
 *    in slave mode. This function must be called after the address is transmitted to clear the flag.
 *  - The ADDR flag can only be cleared by reading SR1 followed by SR2.
 */
void I2C_ClearADDRFlag(I2C_TypeDef *I2Cx)
{
    (void)I2Cx->SR1; // Read SR1 to clear the ADDR flag
    (void)I2Cx->SR2; // Read SR2 to complete the clearing process
}

// /* Enable DMA for I2C */
// void I2C_EnableDMA(I2C_Config *config) {
//     config->I2Cx->CR2 |= I2C_CR2_DMAEN;
// }

/**
 * @brief  Disables DMA requests for I2C data transfer.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function clears the DMAEN bit in the CR2 register to stop
 *         the I2C peripheral from sending DMA requests for data transfers.
 */
void I2C_DisableDMA(I2C_TypeDef *I2Cx)
{
    /* Clear DMAEN bit in CR2 register to disable DMA requests */
    I2Cx->CR2 &= ~I2C_CR2_DMAEN;
}

**
 * @brief  Enables DMA requests for I2C data transfer.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function sets the DMAEN bit in the CR2 register to allow
 *         the I2C peripheral to send requests to the DMA controller for data transfers.
 */
void I2C_EnableDMA(I2C_TypeDef *I2Cx)
{
    /* Set DMAEN bit in CR2 register to enable DMA requests */
    I2Cx->CR2 |= I2C_CR2_DMAEN;
}

/**
 * @brief Configure the acknowledgment state for the I2C peripheral.
 *
 * This function enables or disables the acknowledgment (ACK) feature in the 
 * I2C communication. When enabled, the I2C peripheral will send an ACK bit 
 * after receiving a byte of data. When disabled, it will send a NACK bit instead.
 *
 * @param I2Cx Pointer to the I2C peripheral base address.
 *              This should be one of the I2C instances (I2C1, I2C2, or I2C3).
 *
 * @param NewState The desired acknowledgment state.
 *                 This can be:
 *                 - ENABLE: Enable acknowledgment.
 *                 - DISABLE: Disable acknowledgment.
 *
 * @note 
 *  - This function should be called before starting a data transfer to 
 *    ensure the acknowledgment state is set correctly.
 *  - Acknowledgment is crucial in I2C communication to confirm data reception 
 *    between the master and slave devices.
 */
void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState) {
    if (NewState == ENABLE) {
        I2Cx->CR1 |= I2C_CR1_ACK;  // Set the ACK bit in CR1 register to enable acknowledgment.
    } else {
        I2Cx->CR1 &= ~I2C_CR1_ACK; // Clear the ACK bit in CR1 register to disable acknowledgment.
    }
}


/**
 * @brief Resets the I2C peripheral configuration to default values.
 *
 * This function resets the I2C peripheral to its default settings, including
 * speed mode, interrupts, and operational mode. The clock is also disabled
 * during the reset process.
 *
 * @param config Pointer to the I2C_Config structure that holds the configuration.
 */
void I2C_Reset(I2C_Config *config)
{
    /* Set I2C speed mode to Fast Mode (FM_Mode) */
    config->Speed_Mode = Speed_Mode.FM_Mode;

    /* Disable interrupts */
    config->Interrupts = Interrupts.Disable;

    /* Set I2C mode to Master */
    config->mode = I2C_MODE_MASTER;

    /* Disable I2C clock */
    I2C_Clock_Disable(config);
}
