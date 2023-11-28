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
This code snippet defines a `Stm32BitbangSpi` class in the `moteus` namespace, which implements a bit-banged (software-based) SPI (Serial Peripheral Interface) communication for STM32 microcontrollers. This class is part of a larger project and is designed to manually control the SPI protocol, a common interface used for communication with various peripheral devices like sensors, SD cards, and more.

### Key Components:

1. **Options Structure**:
   - Holds the configuration parameters for the SPI interface.
   - `mosi`, `miso`, `sck`, `cs`: Pin names for SPI's MOSI (Master Out Slave In), MISO (Master In Slave Out), SCK (Serial Clock), and CS (Chip Select), respectively.
   - `frequency`: Clock frequency for SPI communication.
   - `width`: Bit width of the data to be transferred.
   - `mode`: SPI mode (only mode 1 is supported in this implementation).

2. **Stm32BitbangSpi Class**:
   - Constructor: Initializes the SPI interface with the given `Options` and `MillisecondTimer`.
   - `write` method: Transmits data over SPI and returns the received data.
   - Manages the SPI protocol manually using GPIO pins.
   - Digital I/O pins are used for MOSI, MISO, SCK, and CS.
   - The timing of the signal is managed using the `MillisecondTimer` and a calculated delay (`us_delay_`).

3. **SPI Communication Process**:
   - The `write` method handles the data transfer.
   - The method sends a 16-bit value and reads back a 16-bit response.
   - SPI signals (MOSI, MISO, SCK) are controlled manually, with appropriate delays for synchronization.
   - CS (Chip Select) is used to initiate and terminate the communication with the target device.

### Functionality:

- The class provides a software-based solution for SPI communication, allowing for flexibility in pin selection and control.
- The `write` method is the key function that sends and receives data over SPI. It controls the timing of each bit sent and received.
- This implementation is suitable for scenarios where hardware SPI is unavailable or specific control over the SPI protocol is required.

### Usage:

To use this class in a project:
1. Create an instance of `Stm32BitbangSpi`, providing the necessary GPIO pins and configuration.
2. Use the `write` method to transmit data to and receive data from a peripheral device over SPI.
3. Ensure the rest of the project supports the manual timing and control requirements of this bit-banged implementation.
#pragma once

#include "mbed.h"

#include "fw/millisecond_timer.h"

namespace moteus {
class Stm32BitbangSpi {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    int frequency = 10000000;
    int width = 16;
    int mode = 1;
  };

  Stm32BitbangSpi(MillisecondTimer* timer,
                  const Options& options)
      : timer_(timer),
        cs_(options.cs, 1),
        mosi_(options.mosi, 0),
        miso_(options.miso),
        sck_(options.sck, 0),
        options_(options) {
    if (options.mode != 1) { mbed_die(); }

    us_delay_ = std::max(1, 500000 / options.frequency);
  }

  uint16_t write(uint16_t value) {
    cs_.write(0);
    timer_->wait_us(us_delay_);

    uint16_t result = 0;

    for (int i = options_.width; i > 0; i--) {
      mosi_.write((value & (1 << (i - 1))) ? 1 : 0);
      sck_.write(1);
      timer_->wait_us(us_delay_);

      sck_.write(0);
      result <<= 1;
      result |= miso_.read() ? 1 : 0;
      timer_->wait_us(us_delay_);
    }

    mosi_.write(0);
    cs_.write(1);

    timer_->wait_us(us_delay_);

    return result;
  }

  MillisecondTimer* const timer_;

  DigitalOut cs_;
  DigitalOut mosi_;
  DigitalIn miso_;
  DigitalOut sck_;

  const Options options_;

  uint32_t us_delay_ = 1;
};
}
