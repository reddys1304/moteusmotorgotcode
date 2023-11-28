// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
This C++ code snippet provides the definition of a class `Stm32Serial` in the `moteus` namespace. It is designed for use in STM32-based systems (a popular family of ARM Cortex-M microcontrollers) and is intended to facilitate serial communication. The class is designed to work with the Mbed OS, a popular real-time operating system for embedded devices.

### Class: `Stm32Serial`

- **Purpose**: To configure and manage serial communication on STM32 microcontrollers.

### Nested Structure: `Options`

- **`tx` and `rx` (PinName)**: These are the transmit (TX) and receive (RX) pins for the serial interface. They are set to `NC` (Not Connected) by default, and the user is expected to configure them according to their hardware setup.
- **`baud_rate` (int)**: The baud rate for serial communication, defaulting to 115200.

### Constructor: `Stm32Serial(const Options&)`

- **Functionality**: The constructor takes an `Options` structure as an argument and configures the serial port accordingly. This includes setting up the pins for TX and RX and initializing the serial hardware with the specified baud rate.
- **8X Oversampling Note**: Unlike Mbed's `RawSerial` class, `Stm32Serial` uses 8X oversampling when necessary, allowing higher baud rates. Oversampling in serial communication helps in accurately sampling the incoming bitstream, especially at higher baud rates where timing precision becomes critical.

### Member Variables

- **`uart_` (USART_TypeDef*)**: Pointer to the USART (Universal Synchronous/Asynchronous Receiver-Transmitter) hardware peripheral. USART is the communication block found in STM32 microcontrollers for handling serial communication.
- **`huart_` (UART_HandleTypeDef)**: A handle structure that contains the configuration information for the specified UART module. This is used by the STM32 HAL (Hardware Abstraction Layer) library for managing the UART peripheral.

### Member Functions

- **`uart_name()`**: Returns the name (identifier) of the UART peripheral used by the instance of `Stm32Serial`. This is useful for identifying which UART block (like USART1, USART2, etc.) is being used.
- **`uart()`**: Returns a pointer to the USART hardware peripheral, allowing direct access to the hardware registers for advanced operations.
- **`huart()`**: Returns a pointer to the UART handle structure, which contains configuration and state information about the UART peripheral.

### Conclusion

The `Stm32Serial` class provides a streamlined and efficient way to handle serial communications on STM32 microcontrollers within the Mbed OS environment. It gives flexibility in setting up the serial port and is designed to handle higher baud rates effectively with 8X oversampling. This class would be particularly useful in embedded systems requiring reliable and high-speed serial communication, such as in robotics, industrial controls, or communication interfaces.
#pragma once

#include "mbed.h"
#include "serial_api_hal.h"
#include "PeripheralPins.h"

namespace moteus {

/// When constructed, this will configure the given serial port,
/// including marking the relevant pins as alternate function.
///
/// NOTE: Unlike the Mbed RawSerial class, this will use 8X
/// oversampling when necessary to achieve higher baud rates.
class Stm32Serial {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;

    int baud_rate = 115200;
  };

  Stm32Serial(const Options&);

  UARTName uart_name() const {
    return static_cast<UARTName>(reinterpret_cast<uint32_t>(uart_));
  }

  USART_TypeDef* uart() {
    return uart_;
  }

  UART_HandleTypeDef* huart() {
    return &huart_;
  }

 private:
  USART_TypeDef* uart_ = nullptr;
  UART_HandleTypeDef huart_{};
};

}
