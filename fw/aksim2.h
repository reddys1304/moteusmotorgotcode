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
The `Aksim2` class in the code you provided is designed for communication with an Aksim2 rotary encoder using UART (Universal Asynchronous Receiver/Transmitter) in conjunction with DMA (Direct Memory Access). It is a part of the `mjbots/moteus` repository and intended for use in robotic systems. Let's break down its structure and functionality:

### Class Overview
- The `Aksim2` class is within the `moteus` namespace.
- It interfaces with a UART device, specifically a `Stm32G4DmaUart` object, and uses a `MillisecondTimer` for timing operations.
- The class is designed to periodically query the Aksim2 encoder and process its responses.

### Constructor
- Accepts a configuration object (`aux::UartEncoder::Config`), a pointer to a `Stm32G4DmaUart` object, and a pointer to a `MillisecondTimer`.
- The configuration object likely contains settings pertinent to the operation of the encoder.

### ISR_Update Method
- This method is called periodically (presumably from an Interrupt Service Routine, hence `ISR` in the name).
- It checks the time elapsed since the last query was issued to the encoder and decides whether to issue a new query or to process a pending response.
- If the response to a previous query has not been received within a specified timeout, it aborts the read operation.
- If it's time to issue a new query, it sends a query character ('d') to the encoder and starts reading the response.

### ProcessQuery Method
- This method processes the response from the encoder.
- It checks the response buffer and validates the data. If the data is not as expected, it discards the read operation.
- On receiving valid data, it updates the status (`aux::UartEncoder::Status`) with the position and error/warning status from the encoder.

### Private Method: StartRead
- Initiates a DMA read operation to receive data from the UART into a buffer.

### Member Variables
- `config_`: Configuration settings for the encoder.
- `uart_`: Pointer to the UART interface being used.
- `timer_`: Pointer to the timer used for managing query intervals.
- `query_outstanding_`: A flag to indicate if a query is pending a response.
- `last_query_start_us_`: The timestamp of when the last query was started.
- `buffer_`: A buffer to hold the response from the encoder.

### Usage Context
This class is a typical example of how embedded systems interact with sensors, like rotary encoders, in a real-time environment. It uses efficient data transfer methods (DMA) and timing (interrupts or periodic checks) to communicate with the encoder, process its data, and update the system status accordingly. This kind of implementation is crucial in robotics, where timely and accurate sensor data is necessary for precise control and movement.

If you have more questions about this class or how it interacts with other parts of the system, feel free to ask!
#pragma once

#include "fw/aux_common.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

namespace moteus {

class Aksim2 {
 public:
  Aksim2(const aux::UartEncoder::Config& config,
         Stm32G4DmaUart* uart,
         MillisecondTimer* timer)
      : config_(config),
        uart_(uart),
        timer_(timer) {}

  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    // Now check to see if we can issue a new one.
    const uint32_t now_us = timer_->read_us();
    const uint32_t delta_us = (now_us - last_query_start_us_);

    // Do we have an outstanding query?
    if (query_outstanding_) {
      if (delta_us > static_cast<uint32_t>(2 * config_.poll_rate_us)) {
        // We timed out.
        uart_->finish_dma_read();
        query_outstanding_ = false;
      } else {
        // See if we can finish it.
        ProcessQuery(status);
      }
    }

    // We did not complete the query, so just return.
    if (query_outstanding_) { return; }

    if (delta_us < static_cast<uint32_t>(config_.poll_rate_us)) {
      // Nope, we're not ready to issue another.
      return;
    }

    last_query_start_us_ = now_us;
    query_outstanding_ = true;
    uart_->write_char('d');
    StartRead();
  }

  void ProcessQuery(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (uart_->read_bytes_remaining() > kResyncBytes) { return; }

    if (uart_->read_bytes_remaining() == 0) {
      // We used up our resync bytes without success.  Just try again.
      uart_->finish_dma_read();
      query_outstanding_ = false;
      return;
    }

    if (buffer_[0] != 'd') {
      // Not what we are expecting.  Just fill up our buffer until
      // the timeout.
      return;
    }

    uart_->finish_dma_read();
    query_outstanding_ = false;

    status->value =
        ((buffer_[1] << 16) |
         (buffer_[2] << 8) |
         (buffer_[3] << 0)) >> 2;
    status->aksim2_err = buffer_[3] & 0x01;
    status->aksim2_warn = buffer_[3] & 0x02;
    status->aksim2_status =
        (buffer_[4] << 8) |
        (buffer_[5] << 0);

    status->nonce++;
    status->active = true;
  }

 private:
  void StartRead() MOTEUS_CCM_ATTRIBUTE {
    uart_->start_dma_read(
        mjlib::base::string_span(reinterpret_cast<char*>(&buffer_[0]),
                                 sizeof(buffer_)));
  }

  const aux::UartEncoder::Config config_;
  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  bool query_outstanding_ = false;

  uint32_t last_query_start_us_ = 0;

  static constexpr int kResyncBytes = 3;
  static constexpr int kMaxCount = 50;

  // The "detailed" reply has a header byte, 3 bytes of position, and
  // 2 bytes of status.
  //
  // We have 3 extra bytes so that we could eventually re-synchronize.
  uint8_t buffer_[6 + kResyncBytes] = {};
};

}
