# I2C Driver for  STM32F407VGT6 

 This project provides an I2C driver for STM32 microcontrollers, enabling communication between the microcontroller and I2C-compatible devices. The driver includes support for both master and slave modes, as well as error handling and DMA integration.

## Features

- I2C Clock Control: Enables and disables the I2C peripheral clock.
- I2C Initialization: Configures GPIO pins and sets the I2C operating mode (master/slave).
- I2C Start and Stop Conditions: Implements the necessary protocols to start and stop I2C communication.
- I2C Addressing: Supports writing to a specified I2C address with read/write direction.
- Data Transmission: Facilitates master transmission and slave data reception.
- Error Checking: Provides functionality to check for I2C error statuses.
- DMA Support: Allows for DMA configuration to enhance data transmission efficiency.
- Bus Status Checking: Checks if the I2C bus is currently busy.
- Acknowledgment Configuration: Configures acknowledgment settings for I2C communication.
- Reset Functionality: Resets the I2C configuration to default settings.

## Execution Process

1. Compile the Driver: Use the provided Makefile to compile the driver code.
2. Initialization: Initialize the I2C peripheral with the desired configuration.
3. Perform I2C Operations: Execute the desired I2C operations (e.g., read, write).
4. Check for Errors: Monitor for any error statuses during communication.
5. Clean Up: Disable the I2C clock and reset the configuration if necessary.


## 🚀 Getting Started

### Prerequisites

- Hardware: 
  - STM32F407VGT6 board
  - I2C peripherals for testing
- Software:
  - STM32CubeIDE or your preferred IDE for development

### Installation

1. Clone the Repository:

    git clone https://github.com/MuskaanJain2021/Mastering_STM32.git
2. Navigate to the I2C Driver Directory:  
   
    cd Mastering_STM32/STM32F407VGT6_DRIVERS/I2C_DRIVER
   
3. Include the Driver:
   Add the driver files to your project in STM32CubeIDE or your chosen development environment.

4. Configure Your Project:
   Make sure your I2C settings match your device specifications.


## List of Operations

  - Clock Control: Enable/Disable the I2C clock.
  - Initialization: Set up GPIO pins and configure the I2C peripheral.
  - Start Condition: Initiate I2C communication.
  - Stop Condition: Terminate I2C communication.
  - Addressing: Write to I2C addresses with read/write direction.
  - Data Transmission: Master and slave data operations.
  - Error Handling: Check and handle I2C errors.
  - DMA Operations: Enable/Disable DMA for I2C communication.
  - Bus Status Check: Verify if the bus is busy.
  - Acknowledge Configuration: Set acknowledgment settings for data receipt.
  - Reset Configuration: Reset the I2C settings to defaults.


## 📄 License

  This project is licensed under the MIT License - see the [LICENSE] file for details.


## 📞 Contact

Got questions or need support? Open an issue in this repo, or reach out to me at muskaanembed2023@gmail.com.

