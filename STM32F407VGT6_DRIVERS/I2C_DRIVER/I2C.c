/*
 * I2C.c
 *
 *
 *Author: muskaan jain
 */

#include "I2C.h"
#include "I2C_Def.h"
#include "GPIO.h"

extern I2C_Config config;

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
 *         the configuration information for the I2C peripheral, including mode, speed,
 *         interrupt settings, DMA settings, and GPIO configurations for the SDA and SCL lines.
 *
 * @note   This function performs several tasks:
 *         - Enables the clock for the I2C peripheral.
 *         - Configures the GPIO pins for SDA and SCL lines with alternate function, open-drain mode, and pull-up resistors.
 *         - Sets the I2C communication speed based on the selected standard or fast mode.
 *         - Configures the rise time of the I2C signals.
 *         - Sets the I2C peripheral in Master or Slave mode.
 *         - Configures interrupts (Error, Event, Buffer) and DMA control if enabled.
 *         - Enables or disables clock stretching in Slave mode.
 *         - Enables the I2C peripheral after configuration.
 */
void I2C_Init(I2C_Config *config)
{
    /* Enable the I2C Clock */
    I2C_Clock_Enable(config);

    /* Configure GPIO for SDA and SCL pins */
    if (config->I2Cx == I2C1)
    {
        GPIO_Pin_Init(GPIOB, I2C1_Pins.SCL_PB6.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C1_Pins.SCL_PB6.altFunction);
        GPIO_Pin_Init(GPIOB, I2C1_Pins.SDA_PB7.pinNumber, GPIO_MODE_AF, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_HIGH, GPIO_PULL_UP, I2C1_Pins.SDA_PB7.altFunction);
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

    /* Disable I2C Peripheral before making configurations */
    config->I2Cx->CR1 &= ~I2C_CR1_PE;

    /* Reset the I2C peripheral and disable clock stretching */
    config->I2Cx->CR1 |= I2C_CR1_SWRST | I2C_CR1_NOSTRETCH;
    config->I2Cx->CR1 &= ~I2C_CR1_SWRST;

    /* Configure Interrupts (Disable, Error, Event, Buffer) */
    config->I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN); /* Clear all interrupt bits first */
    if (config->Interrupts.Disable == 0)                                         /* If interrupts are enabled */
    {
        if (config->Interrupts.Buffer)
            config->I2Cx->CR2 |= I2C_CR2_ITBUFEN;
        if (config->Interrupts.Error)
            config->I2Cx->CR2 |= I2C_CR2_ITERREN;
        if (config->Interrupts.Event)
            config->I2Cx->CR2 |= I2C_CR2_ITEVTEN;
    }

    /* Configure DMA (Disable, TX DMA, RX DMA) */
    config->I2Cx->CR2 &= ~I2C_CR2_DMAEN; // Clear DMA enable bit
    if (config->DMA_Control.Disable == 0)
    {
        if (config->DMA_Control.TX_DMA_Enable || config->DMA_Control.RX_DMA_Enable)
        {
            I2C_EnableDMA(config->I2Cx); /* Enable DMA for I2C if TX or RX DMA is enabled */
        }
        else
        {
            I2C_DisableDMA(config->I2Cx); /* Disable DMA if not needed */
        }
    }

    /* Configure I2C Speed Mode (Standard or Fast Mode) */
    if (config->speed_mode.FM_Mode)
    {
        /*
         * APB1 clock = 42 MHz
         * I2C Fast Mode formula for CCR:
         *   - Fast mode: T_high + T_low = CCR * (Tpclk1)
         *   - Set CCR value by dividing T_high + T_low by the clock period (Tpclk1 = 1/APB1 clock)
         *   - Also, enable duty cycle control for fast mode.
         */
        config->I2Cx->CR2 = 30;                        /* APB1 clock frequency is 42 MHz */
        config->I2Cx->CCR = (1 << 15) | (1 << 14) | 5; /* Fast mode configuration */
        config->I2Cx->TRISE = 30;                      /* Maximum rise time for fast mode is configured as 300 ns */
    }
    else
    {
        /*
         * Standard Mode I2C Setup:
         *  - Standard mode CCR calculation: CCR = (APB1_CLK / (2 * I2C_SPEED_STANDARD))
         *  - TRISE = Maximum rise time (standard mode: 1000 ns)
         */
        config->I2Cx->CR2 = 25;    /* APB1 clock frequency is 42 MHz */
        config->I2Cx->CCR = 0x28;  /* Standard mode clock control */
        config->I2Cx->TRISE = 0x8; /* Maximum rise time for standard mode */
    }

    /* Set I2C Mode (Master or Slave) */
    if (config->mode == I2C_MODE_MASTER)
    {
        /* Master mode configuration */
        config->I2Cx->CCR = config->speed; /* Set clock speed for master mode */
    }
    else if (config->mode == I2C_MODE_SLAVE)
    {
        /* Slave mode configuration */
        config->I2Cx->OAR1 = (config->ownAddress << 1); /* Set own address for slave mode */
    }
    /*Disable clock stretching*/
    config->I2Cx->CR1 |= I2C_CR1_NOSTRETCH;
    /* Enable I2C peripheral after configuration */
    config->I2Cx->CR1 |= I2C_CR1_PE;
}

/**
 * @brief  Generates an I2C start condition.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @retval 0 if successful, -1 if timeout occurs.
 * @note   This function sets the START bit in the CR1 register to initiate
 *         communication and waits until the start condition is generated.
 */
int I2C_Start(I2C_TypeDef *I2Cx)
{
    int timeout = 1000;               // Correct variable name
    I2Cx->CR1 |= I2C_CR1_START;       /* Set the START bit to begin communication */
    while (!(I2Cx->SR1 & I2C_SR1_SB)) // Wait for the start bit to be set
    {
        if (timeout == 0)
        {
            return I2C_ERROR_TIMEOUT; // Return error code on timeout
        }
        timeout--; // Decrement timeout counter
    }
    return 0; // Return success code
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
    I2C_TypeDef *I2Cx = config->I2cx;
    // Set the TxRxState based on the direction
    if (direction == I2C_READ)
    {
        config->TxRxState = I2C_BUSY_IN_RX;
    }
    else
    {
        config->TxRxState = I2C_BUSY_IN_TX;
    }

    // Send a 7-bit address  with direction bit
    WRITE_REG(I2CX->DR = (address << 1) | direction); /* Send the 7-bit address and the direction bit */
    while (!READ_BIT(I2Cx->SR1 & I2C_SR1_ADDR))
        ; // Wait until the address is sent/acknowledgemet of addr

    I2C_ClearADDRFlag(I2Cx); // Clear the ADDR flag

    return I2C_OK;
}

/**
 * @brief  Transmits data as a master to the specified slave.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral.
 * @param  address: 7-bit address of the slave device.
 * @param  data: Pointer to the data to be transmitted.
 * @param  size: Size of the data buffer to be transmitted(no of bytes to transmit).
 * @param  timeout: Timeout value for the transmission.
 * @return I2C_Status: Returns the status of the transmission.
 *         - I2C_OK: Transmission successful.
 *         - I2C_BUSY: I2C bus is busy.
 *         - I2C_NACK_RECEIVED: NACK received from slave.
 *         - I2C_ERROR_TIMEOUT: Timeout occurred.
 *         - I2C_ERROR_BUS: Bus error detected.
 *         
 * @note   This function transmits data from the master to the specified
 *         slave by writing data byte-by-byte to the I2C data register (DR).
 */
I2C_Status I2C_Master_Transmit(I2C_Config *config, uint16_t address, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    I2C_TypeDef *I2Cx = config->I2Cx;
    I2C_Status status;
    status = I2C_Start(config->I2Cx);
    if (status != I2C_OK)
    {
        return status;
    }
    status = I2C_WriteAddress(config->I2Cx, address, I2C_Direction_Transmitter);
    if (status != I2C_OK)
    {
        I2C_Stop(config->I2Cx);
        return status;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        uint32_t TXE_timeout = timeout; // Reset timeout for each byte
        WRITE_REG(I2Cx->DR, data[i]);   // Write data to DR

        while (!(READ_BIT(I2Cx->SR1 & I2C_SR1_TXE)))
        {
            if (--TXE_timeout == 0)
            {
                I2C_Stop(I2Cx);
                return I2C_TIMEOUT
            }
            // Error Check
            if (READ_BIT(I2Cx->SR1, I2C_SR1_AF))
            {
                I2C_Stop(I2Cx);
                return I2C_NACK_RECEIVED;
            }
        }
    }

    // Wait until BTF flag is set
    uint32_t BTF_timeout = timeout;
    while (!(READ_BIT(O2C->SR1, I2C_SR1_BTF)))
    {
        if (BTF_timeout == 0)
        {
            I2C_Stop(I2Cx);
            return I2C_TIMEOUT
        }
    }
    // Generate Stop Condition
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
    I2C_TypeDef *I2Cx = config->I2Cx;
    while (size--)
    {
        uint32_t RXNE_timeout = timeout;
        while (!(READ_BIT(config->I2Cx->SR1, I2C_SR1_RXNE))) // Wait for RXNE (Receive buffer not empty)
        {
            if (--RXNE_timeout == 0)
            {
                return I2C_ERROR_TIMEOUT;
            }
            if (READ_BIT(I2Cx->SR1, I2C_SR1_OVR))
            {
                return I2C_ERROR_OVERRUN;
            }
            // Check for Stop condition
            if (READ_BIT(I2Cx->SR1, I2C_SR1_STOPF))
            {
                // Clear STOPF flag
                (void)I2Cx->SR1;
                WRITE_REG(I2Cx->CR1, I2Cx->CR1);
                return I2C_ERROR_STOP;
            }
        }
        *data++ = READ_REG(I2Cx->DR); // Read data
    }
    // Wait for Stop condition after receiving data
    uint32_t STOPF_timeout = timeout;
    while (!(READ_BIT(I2Cx->SR1, I2C_SR1_STOPF)))
    {
        if (--STOPF_timeout == 0)
        {
            return I2C_ERROR_TIMEOUT;
        }
        // Check for errors
        if (READ_BIT(I2Cx->SR1, I2C_SR1_OVR))
        {
            return I2C_ERROR_OVERRUN;
        }
        if (READ_BIT(I2Cx->SR1, I2C_SR1_BERR))
        {
            return I2C_ERROR_BUS;
        }
    }

    // Clear STOPF flag
    (void)I2Cx->SR1;
    WRITE_REG(I2Cx->CR1, I2Cx->CR1);
    return I2C_OK;
}
/**
 * @brief  Checks the I2C peripheral for any error conditions.
 * @param  I2Cx: Pointer to I2C peripheral.
 * @retval I2C_Status: Status of the I2C error (OK or specific error code).
 */
I2C_Status I2C_CheckError(I2C_TypeDef *I2Cx)
{
    if (READ_BIT(I2Cx->SR1, I2C_SR1_AF))
    {
        // Clear the AF flag by writing 0
        CLEAR_BIT(I2Cx->SR1, I2C_SR1_AF);
        return I2C_ERROR_ACK_FAILURE; // Acknowledge failure
    }
    if (READ_BIT(I2Cx->SR1, I2C_SR1_OVR))
    {
        CLEAR_BIT(I2Cx->SR1, I2C_SR1_OVR);
        return I2C_ERROR_OVERRUN; // Overrun error
    }
    if (READ_BIT(I2Cx->SR1, I2C_SR1_ARLO))
    {
        CLEAR_BIT(I2Cx->SR1, I2C_SR1_ARLO);
        return I2C_ERROR_ARBITRATION_LOST; // arbitration loss error
    }
    if (READ_BIT(I2Cx->SR1, I2C_SR1_BERR))
    {
        CLEAR_BIT(I2Cx->SR1, I2C_SR1_BERR);
        return I2C_ERROR_BUS; // bus error
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

    // check if the device is in the master mode
    if (READ_BIT(I2Cx->SR2, I2C_SR2_MSL))
    {
        // Device now in master mode

        // Check if the given device is recieving data
        if (config->TxRxState == I2C_BUSY_IN_RX)
        {
            // check if only 1 byte is to be recieved
            if (config->RxSize == 1)
            {
                // Disable the acknowledgement
                I2C_AcknowledgeConfig(I2Cx, DISABLE);
            }
        }
    }

    // Clear ADDR flag by reading SR1 and SR2
    (void)I2Cx->SR1; // Read SR1 to clear the ADDR flag
    (void)I2Cx->SR2; // Read SR2 to complete the clearing process
}
/**
 * @brief  Checks if the bus is busy
 *
 * @param  I2Cx: I2C peripheral
 * @return I2C_Status: Status of the bus
 */
I2C_Status I2C_CheckBusBusy(I2C_TypeDef *I2Cx)
{
    if (READ_BIT(I2Cx->SR2, I2C_SR2_BUSY))
    {
        return I2C_BUSY;
    }
    return I2C_OK;
}

/**
 * @brief Checks the status of the specified I2C flag.
 * @param I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, or I2C3).
 * @param flag: The I2C flag to check. Example flags:
 *         - I2C_SR1_TXE: Transmit buffer empty
 *         - I2C_SR1_RXNE: Receive buffer not empty
 *         - I2C_SR1_BTF: Byte transfer finished
 *         - I2C_SR1_ADDR: Address matched
 *         - I2C_SR1_STOPF: Stop condition detected
 *         - I2C_SR1_SB: Start bit generated
 *         - I2C_SR1_AF: Acknowledge failure
 * @return uint8_t: 1 if the flag is set, 0 if the flag is reset.
 */
uint8_t I2C_GetFlagStatus(I2C_Config I2Cx, uint32_t flag)
{
    if (READ_BIT(I2Cx->SR1, flag) != 0)
    {
        return 1; /*Set the given flag*/
    }
    else
    {
        return 0; /*Reset the given flag*/
    }
}

/**
 * @brief  Disables DMA requests for I2C data transfer.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function clears the DMAEN bit in the CR2 register to stop
 *         the I2C peripheral from sending DMA requests for data transfers.
 */
void I2C_DisableDMA(I2C_TypeDef *I2Cx)
{
    /* Clear DMAEN bit in CR2 register to disable DMA requests */
    CLEAR_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
}

/**
 * @brief  Enables DMA requests for I2C data transfer.
 * @param  I2Cx: Pointer to the I2C peripheral (I2C1, I2C2, I2C3).
 * @note   This function sets the DMAEN bit in the CR2 register to allow
 *         the I2C peripheral to send requests to the DMA controller for data transfers.
 */
void I2C_EnableDMA(I2C_TypeDef *I2Cx)
{
    /* Set DMAEN bit in CR2 register to enable DMA requests */
    SET_BIT(I2Cx->CR2, I2C_CR2_DMAEN);
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
void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
    if (NewState == ENABLE)
    {
        SET_BIT(I2Cx->CR1, I2C_CR1_ACK); // Set the ACK bit in CR1 register to enable acknowledgment.
    }
    else
    {
        CLEAR_BIT(I2Cx->CR1, I2C_CR1_ACK); // Clear the ACK bit in CR1 register to disable acknowledgment.
    }
}
/**
 * @brief  Enables or disables callback events (interrupts) for the I2C peripheral in slave mode.
 * @param  pI2Cx: Pointer to the I2C peripheral (I2C1, I2C2, or I2C3).
 * @param  config: Pointer to the I2C_Config structure containing the interrupt configuration.
 * @param  EnorDi: ENABLE to enable interrupts, DISABLE to disable.
 * @note   This function uses the I2C_Config structure to selectively enable or disable
 *         Error, Event, and Buffer interrupts for I2C communication.
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, I2C_Config *config, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        /* Enable Event Interrupt if configured */
        if (config->Interrupts.Event)
            SET_BIT(pI2Cx->CR2, I2C_CR2_ITEVTEN);

        /* Enable Buffer Interrupt if configured */
        if (config->Interrupts.Buffer)
            SET_BIT(pI2Cx->CR2, I2C_CR2_ITBUFEN);

        /* Enable Error Interrupt if configured */
        if (config->Interrupts.Error)
            SET_BIT(pI2Cx->CR2, I2C_CR2_ITERREN);
    }
    else // Disable interrupts
    {
        /* Disable Event Interrupt */
        CLEAR_BIT(pI2Cx->CR2, I2C_CR2_ITEVTEN);

        /* Disable Buffer Interrupt */
        CLEAR_BIT(pI2Cx->CR2, I2C_CR2_ITBUFEN);

        /* Disable Error Interrupt */
        CLEAR_BIT(pI2Cx->CR2, I2C_CR2_ITERREN);
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
    config->speed_mode.SM_Mode = 0;
    config->speed_mode.FM_Mode = 1;

    /* Disable interrupts */
    config->Interrupts.Disable = 1;
    config->Interrupts.Error = 0;
    config->Interrupts.Event = 0;
    config->Interrupts.Buffer = 0;

    /* Set I2C mode to Master */
    config->mode = I2C_MODE_MASTER;

    /* Disable I2C clock */
    I2C_Clock_Disable(config);
}
