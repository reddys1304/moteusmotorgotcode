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

The `MA732` class in the provided code is part of the `mjbots/moteus` project and is designed for interfacing with the MA732 magnetic encoder using SPI (Serial Peripheral Interface) communication. This class encapsulates the functionality required to communicate with the encoder, including configuration and data acquisition. Let's break down its key components and functionalities:

### Overview
- The `MA732` class is within the `moteus` namespace.
- It utilizes the `Stm32Spi` class to handle SPI communication and a `MillisecondTimer` for timing purposes.

### Structure `Options`
- Inherits from `Stm32Spi::Options`, adding specific configuration options for the MA732 encoder, such as `filter_us` and `bct`.

### Constructor
- Initializes the SPI interface with modified options suitable for the MA732 encoder.
- Calls `SetConfig` to configure the encoder with the provided settings.

### Sample Methods
1. **Sample**
   - Performs a blocking SPI transfer to sample data from the encoder.
2. **StartSample**
   - Initiates an SPI write operation to start the sampling process.
3. **FinishSample**
   - Completes the SPI transfer and returns the sampled data.

### SetConfig Method
- Configures the encoder with specified settings, such as filter and bit count time (BCT). It calculates the appropriate register values based on the desired configuration.

### SetRegister Private Method
- Sets a specific register in the MA732 encoder. It writes to the register using SPI, waits for the operation to complete, and then verifies that the desired setting has been successfully applied.

### Usage Context
- The `MA732` class provides a specific interface for the MA732 magnetic encoder, a component commonly used in robotics and motor control applications for precise position sensing.
- The class abstracts the lower-level details of SPI communication, offering a more straightforward and higher-level interface for interacting with the encoder.
- The use of a `MillisecondTimer` for timing control within the class indicates a focus on precise timing and synchronization, important in encoder communication and motor control.

In summary, the `MA732` class demonstrates how embedded systems often encapsulate hardware communication protocols within classes for cleaner, more manageable code, particularly in complex applications like robotics control systems. This approach is essential for ensuring modularity, maintainability, and accurate operation in systems where precise control and data acquisition are crucial.


#pragma once

#include "mbed.h"

#include "hal/spi_api.h"

#include "fw/ccm.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"

namespace moteus {

class MA732 {
 public:
  struct Options : public Stm32Spi::Options {
    uint16_t filter_us = 1024;
    uint8_t bct = 0;

    Options(const Stm32Spi::Options& v) : Stm32Spi::Options(v) {}
  };

  MA732(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        spi_([&]() {
        auto copy = options;
        copy.mode = 0;
        return copy;
      }()) {
    error_ = SetConfig(options);
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.write(0x0000);
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0x0000);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.finish_write();
  }

  bool error() const { return error_; }

  // Return true on success.
  bool SetConfig(const Options& options) {
    const auto desired_filter = [&]() {
      const auto filter_us = options.filter_us;
      if (filter_us <= 64) { return 51; }
      if (filter_us <= 128) { return 68; }
      if (filter_us <= 256) { return 102; }
      if (filter_us <= 1024) { return 119; }
      if (filter_us <= 2048) { return 136; }
      if (filter_us <= 4096) { return 153; }
      if (filter_us <= 8192) { return 170; }
      if (filter_us <= 16384) { return 187; }
      return 187;
    }();

    // FW = 0x0e
    if (SetRegister(0x0e, desired_filter)) { return true; }

    // BCT = 0x01
    if (SetRegister(0x01, options.bct)) { return true; }

    return false;
  }

 private:
  bool SetRegister(uint8_t reg, uint8_t desired) {
    spi_.write(0x4000 | (reg << 8));

    timer_->wait_us(2);

    const auto current_value = spi_.write(0x0000) >> 8;

    if (current_value == desired) { return false; }

    spi_.write(0x8000 | (reg << 8) | desired);

    // TODO: This must be fixed!
    timer_->wait_ms(20);

    const auto final_value = (spi_.write(0x0000) >> 8);
    return final_value != desired;
  }

  MillisecondTimer* const timer_;
  Stm32Spi spi_;
  bool error_ = false;
};

}
