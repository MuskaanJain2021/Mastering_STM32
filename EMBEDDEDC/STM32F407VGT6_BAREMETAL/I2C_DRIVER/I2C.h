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

/* Error codes */
typedef enum {
    I2C_OK,
    I2C_ERROR,
    I2C_BUSY,
    I2C_TIMEOUT,
    I2C_NACK_RECEIVED,
    I2C_ARBITRATION_LOST,    // Arbitration lost in master mode
    I2C_BUS_ERROR            // Hardware-related bus error
} I2C_Status;

/* I2C Speeds */
typedef enum {
    I2C_SPEED_STANDARD = 100000,   // 100kHz
    I2C_SPEED_FAST = 400000,       // 400kHz
    I2C_SPEED_HIGH = 1000000       // 1MHz
} I2C_Speed;

/* I2C Modes */
typedef enum {
    I2C_MODE_MASTER,
    I2C_MODE_SLAVE
} I2C_Mode;

/* Mode of operation: Standard or Fast Mode */
typedef struct I2C_SPEED_MODE {
    uint8_t SM_Mode;
    uint8_t FM_Mode;
} I2C_SPEED_MODE;

/* Callback function types */
typedef void (*I2C_Callback)(void);

/* I2C Callback Structure */
typedef struct {
    I2C_Callback onTransmitComplete;   // Callback for transmission completion
    I2C_Callback onReceiveComplete;    // Callback for reception completion
    I2C_Callback onError;              // Callback for error handling
} I2C_Callbacks;

/* I2C Configuration structure */
typedef struct {
    I2C_TypeDef *I2Cx;            // I2C peripheral (I2C1, I2C2, or I2C3)
    I2C_Mode mode;                // Master or Slave mode
    I2C_Speed speed;              // Standard, Fast, or High-speed mode
    I2C_SPEED_MODE speed_mode;    // SM or FM mode
    uint16_t ownAddress;          // Used for slave mode
    FunctionalState dmaEnabled;   // DMA enable flag
    I2C_Callbacks callbacks;      // Callback functions for events
    uint8_t interruptEnabled;     // Interrupt enable/disable flag
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
void I2C_Start(I2C_TypeDef *I2Cx);

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
 * @brief Writes a 10-bit address on I2C bus
 *
 * @param I2Cx: I2C peripheral
 * @param address: 10-bit slave address
 * @param direction: Direction of communication (Read/Write)
 * @return I2C_Status: Status of the operation
 */
I2C_Status I2C_WriteAddress10Bit(I2C_TypeDef *I2Cx, uint16_t address, uint8_t direction);

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

#endif /* I2C_H_ */
