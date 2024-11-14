/*
 * I2C.h
 *
 *
 *  Author: muskaan jain
 */

#ifndef I2C_H_
#define I2C_H_

#include "main.h"
#include "GPIO.h"
#include "I2C_Def.h"
#include "DMA.h"
#include <stm32f4xx.h>
#include <stm32f407xx.h> 
/* Error codes */
typedef enum
{
    I2C_OK, // Operation completed successfully
    I2C_ERROR_STOP,// Stop condition error
    I2C_BUSY, // I2C bus is busy
    I2C_ERROR_TIMEOUT, // Specific timeout error during an operation
    I2C_NACK_RECEIVED,   // NACK received after address or data byte
    I2C_ERROR_OVERRUN, // Data overrun/underrun error
    I2C_ARBITRATION_LOST, // Arbitration lost in master mode
    I2C_ERROR_BUS         // Hardware-related bus error
} I2C_Status;

/* I2C Speeds */
typedef enum
{
    I2C_SPEED_STANDARD = 100000, // 100kHz
    I2C_SPEED_FAST = 400000,     // 400kHz
    I2C_SPEED_HIGH = 1000000     // 1MHz
} I2C_Speed;

/* I2C Modes */
typedef enum
{
    I2C_MODE_MASTER,
    I2C_MODE_SLAVE
} I2C_Mode;

/* Mode of operation: Standard or Fast Mode */
typedef struct I2C_SPEED_MODE
{
    uint8_t SM_Mode;
    uint8_t FM_Mode;
} I2C_SPEED_MODE;
/* I2C Operation States */
typedef enum
{
    I2C_READY = 0,           // I2C is ready
    I2C_BUSY_IN_TX,          // I2C is busy in transmission
    I2C_BUSY_IN_RX           // I2C is busy in reception
} I2C_State;

/* I2C Interrupt Control */
typedef struct I2C_Interrupts
{
    uint16_t Disable;
    uint16_t Error;
    uint16_t Event;
    uint16_t Buffer;
} I2C_Interrupts;

/* I2C DMA Control */
typedef struct I2C_DMA_Control
{
    uint8_t Disable;
    uint8_t TX_DMA_Enable;
    uint8_t RX_DMA_Enable;
} I2C_DMA_Control;

/* Callback function types */
typedef void (*I2C_Callback)(void);

/* I2C Callback Structure */
typedef struct
{
    I2C_Callback onTransmitComplete; // Callback for transmission completion
    I2C_Callback onReceiveComplete;  // Callback for reception completion
    I2C_Callback onError;            // Callback for error handling
} I2C_Callbacks;

/* I2C Configuration structure */
typedef struct
{
    I2C_TypeDef *I2Cx;           // I2C peripheral (I2C1, I2C2, or I2C3)
    I2C_Mode mode;               // Master or Slave mode
    I2C_Speed speed;             // Standard, Fast, or High-speed mode
    I2C_SPEED_MODE speed_mode;   // SM or FM mode
    uint16_t ownAddress;         // Used for slave mode
    I2C_Interrupts Interrupts;   // Interrupt configuration (Disable, Error, Event, Buffer)
    I2C_DMA_Control DMA_Control; // DMA configuration (TX, RX)
    I2C_Callbacks callbacks;     // Callback functions for events
    uint8_t TxRxState;           //Transmission/Reception State
    uint16_t Rxsize;             //Size of recieved data
    uint16_t Txsize;             //size of data to be transmitted 

} I2C_Config;

/* Function prototypes */
/**
 * @brief Initializes the I2C peripheral
 *
 * @param config: Configuration structure for I2C
 */
void I2C_Init(I2C_Config *config);

/**
 * @brief Enables I2C Clock
 *
 * @param config: Configuration structure for I2C
 */
void I2C_Clock_Enable(I2C_Config *config);

/**
 * @brief Disables I2C Clock
 *
 * @param config: Configuration structure for I2C
 */
void I2C_Clock_Disable(I2C_Config *config);

/**
 * @brief Starts I2C communication
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 */
int I2C_Start(I2C_TypeDef *I2Cx);

/**
 * @brief Stops I2C communication
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 */
void I2C_Stop(I2C_TypeDef *I2Cx);

/**
 * @brief Sends address on I2C bus
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 * @param address: 7-bit slave address
 * @param direction: Direction of communication (Read/Write)
 * @return I2C_Status: Status of the operation
 */
I2C_Status I2C_WriteAddress(I2C_TypeDef *I2Cx, uint16_t address, uint8_t direction);

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

//uint8_t I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t flag);
/**
 * @brief Master transmit data
 *
 * @param config: I2C configuration
 * @param address: Slave address
 * @param data: Pointer to data to be transmitted
 * @param size: Size of the data
 * @param timeout: Timeout for the operation
 * @return I2C_Status: Status of the operation
 */
I2C_Status I2C_Master_Transmit(I2C_Config *config, uint16_t address, const uint8_t *data, uint16_t size, uint32_t timeout);

/**
 * @brief Slave receive data
 *
 * @param config: I2C configuration
 * @param data: Pointer to buffer for received data
 * @param size: Size of the data
 * @param timeout: Timeout for the operation
 * @return I2C_Status: Status of the operation
 */
I2C_Status I2C_Slave_Receive(I2C_Config *config, uint8_t *data, uint16_t size, uint32_t timeout);

/**
 * @brief Clears ADDR flag after address is matched
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 */
void I2C_ClearADDRFlag(I2C_TypeDef *I2Cx);

/**
 * @brief Configures Acknowledge control
 *
 * @param I2Cx: I2C peripheral
 * @param NewState: ENABLE/DISABLE the acknowledgment
 */
void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState);

/**
 * @brief Writes a 7-bit address on I2C bus
 *
 * @param I2Cx: I2C peripheral
 * @param address: 7-bit slave address
 * @param direction: Direction of communication (Read/Write)
 * @return I2C_Status: Status of the operation
 */
I2C_Status I2C_WriteAddress(I2C_TypeDef *I2Cx, uint16_t address, uint8_t direction);

/**
 * @brief Checks if the bus is busy
 *
 * @param I2Cx: I2C peripheral
 * @return I2C_Status: Status of the bus
 */
I2C_Status I2C_CheckBusBusy(I2C_TypeDef *I2Cx);

/**
 * @brief Enables DMA for I2C
 *
 * @param config: I2C configuration
 */
void I2C_EnableDMA(I2C_Config *config);

/**
 * @brief Disables DMA for I2C
 *
 * @param config: I2C configuration
 */
void I2C_DisableDMA(I2C_Config *config);

/**
 * @brief Resets I2C peripheral
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 */
void I2C_Reset(I2C_TypeDef *I2Cx);

/**
 * @brief Checks for I2C error flags
 *
 * @param I2Cx: I2C peripheral (I2C1, I2C2, I2C3)
 * @return I2C_Status: Error status
 */
I2C_Status I2C_CheckError(I2C_TypeDef *I2Cx);
/**
 * @brief  Transmits data as a slave to the master in Slave Transmit Mode.
 * @param  config: Pointer to the I2C_Config structure that contains
 *         the configuration information for the I2C peripheral.
 * @param  data: Pointer to the data buffer to be transmitted.
 * @param  size: Size of the data buffer to be transmitted.
 * @return I2C_Status: Returns the status of the transmission (I2C_OK, I2C_ERROR).
 * @note   This function handles the slave transmit sequence, ensuring that
 *         data is loaded to the Data Register and the interface stretches SCL low
 *         if no data is available in DR to transmit. It clears the necessary flags
 *         and ensures smooth data transmission with clock stretching.
 */

/**
 * @brief IRQ Handlers for event and error interrupts
 */
void I2C_EV_IRQHandler(I2C_Config *config);
void I2C_ER_IRQHandler(I2C_Config *config);
/**
 * @brief Enables or disables specified callback events (interrupts) for the I2C peripheral in slave mode.
 *
 * @param pI2Cx: I2C peripheral (I2C1, I2C2, or I2C3)
 * @param config: Configuration structure with enabled interrupts
 * @param EnorDi: Set to ENABLE to enable specified events or DISABLE to disable them
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, I2C_Config *config, uint8_t EnorDi);

#endif /* I2C_H_ */
