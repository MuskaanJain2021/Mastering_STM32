/*
 * I2C_Def.h
 *
 *  
 *Author: muskaan jain
 */

#ifndef I2C_DEF_H_
#define I2C_DEF_H_

#include "main.h"
#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_Direction_Transmitter I2C_WRITE
#define I2C_Direction_Receiver    I2C_READ
/* Structure to hold I2C Pin configuration */
typedef struct {
    uint8_t pinNumber;       // GPIO Pin number
    uint8_t altFunction;     // Alternate function for the pin
} I2C_PinConfig;

/* Structure for I2C Pin Pair (SCL, SDA) */
typedef struct {
    I2C_PinConfig SCL;       // SCL Pin configuration
    I2C_PinConfig SDA;       // SDA Pin configuration
} I2C_PinPair;

/* I2C1 Pin Definitions */
static const I2C_PinPair I2C1_Pins[] = {
    { {6, GPIO_AF4_I2C1}, {7, GPIO_AF4_I2C1} },  // PB6 (SCL), PB7 (SDA)
    { {8, GPIO_AF4_I2C1}, {9, GPIO_AF4_I2C1} }   // PB8 (SCL), PB9 (SDA)
};

/* I2C2 Pin Definitions */
static const I2C_PinPair I2C2_Pins = {
    { {10, GPIO_AF4_I2C2}, {11, GPIO_AF4_I2C2} }  // PB10 (SCL), PB11 (SDA)
};

/* I2C3 Pin Definitions */
static const I2C_PinPair I2C3_Pins = {
    { {8, GPIO_AF4_I2C3}, {9, GPIO_AF4_I2C3} }    // PA8 (SCL), PC9 (SDA)
};



#endif /* I2C_DEF_H_ */
