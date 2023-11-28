// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
ChatGPT
This C++ code defines the Stm32G4DmaUart class for managing UART communication on STM32 microcontrollers using DMA (Direct Memory Access). It provides options for configuring the UART, including pins, baud rate, and DMA channels. The class supports asynchronous read and write operations using DMA for efficient data transfer. It allows queuing characters for transmission and provides functions to start, check, and finish DMA-based read and write operations. Additionally, it includes methods for retrieving UART request codes and type conversions. This code is essential for high-performance UART communication in embedded systems.

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
The provided code is a C++ class named `Stm32G4DmaUart`, part of the `mjbots/moteus` GitHub repository, designed to interface with UART (Universal Asynchronous Receiver/Transmitter) hardware on STM32G4 microcontrollers using DMA (Direct Memory Access). This approach allows efficient data transfer without burdening the CPU. Let's dissect its key components:

### Class `Stm32G4DmaUart`
- It's under the `moteus` namespace and uses the `mbed` framework, a popular choice for ARM microcontroller-based applications.

### Structure `Options`
- Holds configuration options for the UART interface:
  - `PinName tx, rx`: Transmit and receive pin assignments.
  - `int baud_rate`: Communication speed.
  - `DMA_Channel_TypeDef* rx_dma, tx_dma`: Pointers to DMA channel configuration for receive and transmit operations.

### Constructor
- Initializes the UART interface with the provided `Options`.
- Configures DMA channels for both transmission and reception, including setting up DMA MUX (multiplexer), which is essential for routing DMA requests to the correct DMA channel.
- Uses `Stm32Serial` class for handling the UART settings.
- Asserts (`MJ_ASSERT`) that the UART and DMA channels are correctly initialized.

### Destructor
- Ensures that any ongoing DMA read or write operations are completed before the object is destroyed.

### Methods
1. **write_char(data)**
   - Writes a single character to the UART data register.

2. **start_dma_read(output)**
   - Begins a DMA read operation into the provided buffer.

3. **is_dma_read_finished()**
   - Checks if the DMA read operation is complete.

4. **read_bytes_remaining()**
   - Returns the number of bytes left to read in the current DMA operation.

5. **finish_dma_read()**
   - Aborts an ongoing DMA read operation.

6. **start_dma_write(data)**
   - Starts a DMA write operation with the provided data.

7. **is_dma_write_finished()**
   - Checks if the DMA write operation is complete.

8. **write_bytes_remaining()**
   - Returns the number of bytes left to write in the current DMA operation.

9. **finish_dma_write()**
   - Aborts an ongoing DMA write operation.

### Utility Functions
- **GetUartRxRequest(uart)** and **GetUartTxRequest(uart)**
  - Return the DMA request line associated with the specified UART for receive and transmit operations, respectively.
- **u32(value)**
  - A template function to cast a value to `uint32_t`, used for handling memory addresses and hardware registers.

### Usage Context
This class is an advanced example of interfacing with microcontroller hardware for efficient data transfer. It demonstrates handling UART communication with DMA to offload the CPU, which is critical in high-performance embedded systems like robotic controllers where real-time processing is crucial.

DMA allows for continuous data transfer between the UART and memory without CPU intervention, making this method of communication highly efficient, especially in scenarios where large amounts of data need to be transmitted or received, such as in a robotics control system.

If you have further code segments or specific questions about this class or its interaction with other parts of the system, feel free to share them.

#pragma once

#include "mbed.h"

#include "PinNames.h"

#include "mjlib/base/string_span.h"

#include "fw/ccm.h"
#include "fw/stm32_dma.h"
#include "fw/stm32_serial.h"

namespace moteus {

class Stm32G4DmaUart {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;

    int baud_rate = 115200;

    DMA_Channel_TypeDef* rx_dma = DMA1_Channel1;
    DMA_Channel_TypeDef* tx_dma = DMA1_Channel2;
  };

  Stm32G4DmaUart(const Options& options)
      : options_(options),
        stm32_serial_([&]() {
                        Stm32Serial::Options s_options;
                        s_options.tx = options.tx;
                        s_options.rx = options.rx;
                        s_options.baud_rate = options.baud_rate;
                        return s_options;
                      }()) {

    uart_ = stm32_serial_.uart();
    MJ_ASSERT(uart_ != nullptr);

    MJ_ASSERT(options.rx_dma);

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    dmamux_rx_ = Stm32Dma::SelectDmamux(options_.rx_dma);
    dmamux_tx_ = Stm32Dma::SelectDmamux(options_.tx_dma);

    options_.rx_dma->CCR =
        DMA_PERIPH_TO_MEMORY |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH;
    dmamux_rx_->CCR = GetUartRxRequest(uart_) & DMAMUX_CxCR_DMAREQ_ID;
    options_.rx_dma->CPAR = u32(&uart_->RDR);

    options_.tx_dma->CCR =
        DMA_MEMORY_TO_PERIPH |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH;
    dmamux_tx_->CCR = GetUartTxRequest(uart_) & DMAMUX_CxCR_DMAREQ_ID;
    options_.tx_dma->CPAR = u32(&uart_->TDR);
  }

  ~Stm32G4DmaUart() {
    finish_dma_read();
    finish_dma_write();
  }

  USART_TypeDef* uart() const {
    return uart_;
  }

  // Queue a single character to be written.  We rely on the FIFO to
  // prevent overflow.
  void write_char(uint8_t data) {
    uart_->TDR = data;
  }

  void start_dma_read(mjlib::base::string_span output) MOTEUS_CCM_ATTRIBUTE {
    options_.rx_dma->CNDTR = output.size();
    options_.rx_dma->CMAR = u32(&output[0]);

    options_.rx_dma->CCR |= DMA_CCR_EN;
    uart_->CR3 |= USART_CR3_DMAR;
  }

  bool is_dma_read_finished() MOTEUS_CCM_ATTRIBUTE {
    return options_.rx_dma->CNDTR == 0;
  }

  int read_bytes_remaining() MOTEUS_CCM_ATTRIBUTE {
    return options_.rx_dma->CNDTR;
  }

  // This can be used to abort a request early.
  void finish_dma_read() MOTEUS_CCM_ATTRIBUTE {
    uart_->CR3 &= ~(USART_CR3_DMAR);
    options_.rx_dma->CCR &= ~(DMA_CCR_EN);
  }

  void start_dma_write(std::string_view data) MOTEUS_CCM_ATTRIBUTE {
    options_.tx_dma->CNDTR = data.size();
    options_.tx_dma->CMAR = u32(&data[0]);

    options_.tx_dma->CCR |= DMA_CCR_EN;
    uart_->CR3 |= USART_CR3_DMAT;
  }

  bool is_dma_write_finished() MOTEUS_CCM_ATTRIBUTE {
    return options_.tx_dma->CNDTR == 0;
  }

  int write_bytes_remaining() MOTEUS_CCM_ATTRIBUTE {
    return options_.tx_dma->CNDTR;
  }

  // This can be used to abort a request early.
  void finish_dma_write() MOTEUS_CCM_ATTRIBUTE {
    uart_->CR3 &= ~(USART_CR3_DMAT);
    options_.tx_dma->CCR &= ~(DMA_CCR_EN);
  }

  static uint32_t GetUartRxRequest(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return DMA_REQUEST_USART1_RX;
      case UART_2: return DMA_REQUEST_USART2_RX;
      case UART_3: return DMA_REQUEST_USART3_RX;
    }
    mbed_die();
  }

  static uint32_t GetUartTxRequest(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return DMA_REQUEST_USART1_TX;
      case UART_2: return DMA_REQUEST_USART2_TX;
      case UART_3: return DMA_REQUEST_USART3_TX;
    }
    mbed_die();
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }

  const Options options_;
  Stm32Serial stm32_serial_;

  USART_TypeDef* uart_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_rx_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_tx_ = nullptr;
};

}
