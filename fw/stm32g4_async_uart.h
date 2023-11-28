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
This C++ header file defines the `Stm32G4AsyncUart` class, part of the `moteus` project, designed for the STM32G4 series of microcontrollers. The class provides asynchronous communication capabilities over UART (Universal Asynchronous Receiver-Transmitter), leveraging the DMA (Direct Memory Access) hardware feature for efficient data transfer.

### Overview

- The `Stm32G4AsyncUart` class extends the `mjlib::micro::AsyncStream` interface, providing asynchronous UART functionality.
- It is particularly designed for STM32G4 microcontrollers, and it uses DMA for efficient data handling.

### Key Components

- **`Options` Structure**: Configurable settings for the UART interface, including:
  - `tx`, `rx`: Transmit and receive pin names.
  - `dir`: Optional pin name for direction control, useful for RS-485 half-duplex communication.
  - `enable_delay_us`, `disable_delay_us`: Delays for enabling and disabling the direction control pin.
  - `baud_rate`: Communication speed.
  - `rx_buffer_size`: Size of the receive buffer.
  - `rx_dma`, `tx_dma`: DMA channels for receive and transmit operations.

- **Constructor (`Stm32G4AsyncUart`)**: Initializes the UART with the specified options.

- **`AsyncReadSome` Method**: Asynchronously reads data into a provided buffer, using a callback to indicate completion.

- **`AsyncWriteSome` Method**: Asynchronously writes data from a buffer, with a callback for completion notification.

- **`Poll` Method**: Must be called regularly to process asynchronous UART events, such as the completion of read/write operations.

### Usage

- The `Stm32G4AsyncUart` class is used in applications where non-blocking UART communication is essential, especially in scenarios involving high data rates or when other tasks need to run concurrently without being blocked by UART communication.

- It is suitable for applications like sensor interfacing, data logging, communication with other microcontrollers or computers, especially in embedded systems based on the STM32G4 series.

### Advantages

- **Efficiency**: By using DMA, the CPU is freed from the overhead of handling each byte of data transfer, which is especially advantageous at higher baud rates.
- **Non-blocking Operations**: Asynchronous methods allow other parts of the application to run without waiting for the completion of UART communication.
- **Buffer Management**: The implementation includes a buffer system for incoming data, reducing the risk of data loss.

### Considerations

- The implementation is specific to STM32G4 microcontrollers and uses STM32 HAL functions and direct register manipulation.
- Proper handling of DMA and UART interrupts is crucial for reliable and efficient operation.
- Regular calls to `Poll` are required to handle asynchronous UART events.
#pragma once

#include "mbed.h"

#include "PinNames.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"

#include "fw/millisecond_timer.h"

namespace moteus {

/// Presents a single USART on the STM32G4 as an AsyncStream.
class Stm32G4AsyncUart : public mjlib::micro::AsyncStream {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;

    // If non-NC, will be set to 1 while transmitting and left at 0
    // otherwise.  Useful for half-duplex RS-485 connections.
    PinName dir = NC;

    // If 'dir' is set, and we are enabling, wait this long after
    // enabling before beginning transmission.
    uint8_t enable_delay_us = 0;

    // And wait this long before disabling.
    uint8_t disable_delay_us = 2;

    int baud_rate = 115200;

    size_t rx_buffer_size = 128u;

    DMA_Channel_TypeDef* rx_dma = DMA1_Channel2;
    DMA_Channel_TypeDef* tx_dma = DMA1_Channel1;
  };

  Stm32G4AsyncUart(mjlib::micro::Pool* pool,
                   MillisecondTimer* timer,
                   const Options&);
  ~Stm32G4AsyncUart() override;

  void AsyncReadSome(const mjlib::base::string_span&,
                     const mjlib::micro::SizeCallback&) override;
  void AsyncWriteSome(const std::string_view&,
                      const mjlib::micro::SizeCallback&) override;

  // Call frequently.
  void Poll();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
