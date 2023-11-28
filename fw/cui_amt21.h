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

The `CuiAmt21` class in the provided code is part of the `mjbots/moteus` project and is specifically designed for interfacing with a CUI AMT21 series of absolute encoders using UART communication. This class encapsulates the functionality required to communicate with the encoder, process its responses, and manage timing and state. Let's break down its key components and functionalities:

### Class Overview
- The `CuiAmt21` class is within the `moteus` namespace.
- It interfaces with a UART device (`Stm32G4DmaUart`) and uses a `MillisecondTimer` for timing the communication with the encoder.

### Constructor
- Initializes the class with the given configuration (`aux::UartEncoder::Config`), a pointer to the `Stm32G4DmaUart` object for UART communication, and a pointer to the `MillisecondTimer` for managing time intervals.

### ISR_Update Method
- This method is called periodically (presumably from an Interrupt Service Routine, hence `ISR` in the name).
- It checks if a query to the encoder is outstanding and processes the response if available.
- If the response is not received within the expected time frame, it resets the query state.
- A new query is issued if the appropriate time interval has passed since the last query.

### ProcessQuery Method
- Processes the response received from the encoder.
- Validates the data, specifically checking the parity bits for data integrity.
- Updates the encoder's status (`aux::UartEncoder::Status`) with the latest position value if the data is valid.

### Private Method: StartRead
- Initiates a DMA read operation to receive data from the UART into a buffer.

### Member Variables
- `config_`: Configuration settings for the encoder.
- `uart_`: Pointer to the UART interface.
- `timer_`: Pointer to the timer used for managing query intervals.
- `query_outstanding_`: A boolean flag to track if a query is pending a response.
- `last_query_start_us_`: Timestamp of the last query initiation.
- `buffer_`: Buffer to store the received data.

### Usage Context
- This class is specifically tailored for communication with CUI AMT21 series encoders, which are absolute encoders commonly used in robotics for precise position sensing.
- The use of UART with DMA indicates a focus on efficient, high-speed data transfer, which is crucial in real-time systems.
- The class abstracts the lower-level details of UART communication, providing a higher-level interface for interacting with the encoder.

In summary, the `CuiAmt21` class is a specialized component for handling communication with a specific type of absolute encoder, demonstrating how embedded systems often encapsulate hardware communication protocols within classes for cleaner, more manageable code, especially in complex applications like robotics.


#pragma once

namespace moteus {

class CuiAmt21 {
 public:
  CuiAmt21(const aux::UartEncoder::Config& config,
           Stm32G4DmaUart* uart,
           MillisecondTimer* timer)
      : config_(config),
        uart_(uart),
        timer_(timer) {}


  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    const uint32_t now_us = timer_->read_us();
    const uint32_t delta_us = (now_us - last_query_start_us_);

    if (query_outstanding_) {
      if (delta_us > static_cast<uint32_t>(2 * config_.poll_rate_us)) {
        // We timed out.
        uart_->finish_dma_read();
        query_outstanding_ = false;
      } else {
        // Check for a response.
        ProcessQuery(status);
      }
    }

    // We didn't manage to finish.  Try again next time.
    if (query_outstanding_) { return; }

    if (delta_us < static_cast<uint32_t>(config_.poll_rate_us)) {
      // We are not ready to issue another request yet.
      return;
    }

    last_query_start_us_ = now_us;
    query_outstanding_ = true;
    uart_->write_char(config_.cui_amt21_address);
    StartRead();
  }

 private:
  void ProcessQuery(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (uart_->read_bytes_remaining() > kResyncBytes) { return; }

    if (uart_->read_bytes_remaining() == 0) {
      // We consumed our resync bytes without finding a header.  Just
      // try again.
      uart_->finish_dma_read();
      query_outstanding_ = false;
      return;
    }

    // Our RS422 lines have to be tied together, which means we should
    // receive our read command echoed back as the first byte.

    if (buffer_[0] != config_.cui_amt21_address) {
      // Not what we were expecting.  Just fill up our buffer until
      // the timeout.
      return;
    }

    uart_->finish_dma_read();
    query_outstanding_ = false;

    // Check the parity bits.
    const uint16_t value = buffer_[1] | (buffer_[2] << 8);

    const auto even_parity = [](uint16_t value) -> bool {
      return (1 ^
              (value & 0x01) ^
              ((value >> 2) & 0x01) ^
              ((value >> 4) & 0x01) ^
              ((value >> 6) & 0x01) ^
              ((value >> 8) & 0x01) ^
              ((value >> 10) & 0x01) ^
              ((value >> 12) & 0x01)) ? true : false;
    };

    const auto odd_parity = [&](uint16_t value) -> bool {
      return even_parity(value >> 1);
    };

    const bool measured_even_parity = even_parity(value);
    const bool measured_odd_parity = odd_parity(value);

    const bool received_odd_parity = !!(value & 0x8000);
    const bool received_even_parity = !!(value & 0x4000);

    if (received_odd_parity != measured_odd_parity ||
        received_even_parity != measured_even_parity) {
      status->checksum_errors++;
      return;
    }

    status->value = value & 0x3fff;
    status->nonce++;
    status->active = true;
  }

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

  uint8_t buffer_[3 + kResyncBytes] = {};
};

}
