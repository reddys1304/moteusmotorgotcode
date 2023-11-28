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
This code snippet defines the `Stm32I2c` class within the `moteus` namespace. It represents a driver for an I2C (Inter-Integrated Circuit) interface on an STM32 microcontroller, utilizing the mbed framework.

### Key Components:

- **Namespace (`moteus`):** The class is part of the `moteus` namespace, suggesting it's a part of a larger robotics or motor control project.

- **Class (`Stm32I2c`):**
  - The class provides an interface for I2C communication, commonly used in microcontrollers for connecting with various peripherals like sensors and displays.

- **Options Structure:**
  - Holds configuration parameters like `sda` and `scl` pins, `frequency`, and `i2c_mode`.

- **Constructor and `Initialize` Method:**
  - The constructor takes an `Options` structure and initializes the I2C hardware.
  - `Initialize` sets up the I2C hardware with specific configurations like disabling the peripheral, setting timing values, and then re-enabling it.

- **Read and Write Methods:**
  - `StartReadMemory` and `StartWriteMemory` are methods to initiate reading from and writing to an I2C device. They set up the necessary hardware registers for the transaction.

- **CheckRead and Poll Methods:**
  - `CheckRead` checks the status of a read operation.
  - `Poll` is used to handle the I2C communication, progressing the state machine based on the status of the hardware registers.

### Functionality and Usage:

This class provides a means to communicate with I2C devices on an STM32 microcontroller. The primary methods handle the start of read/write operations and polling to continue or complete these operations. The class seems to be designed to be non-blocking, where initiating a read/write does not wait for completion; instead, the `Poll` method is called periodically to advance the communication.

### Example Usage:

A typical use case might look like this:

```cpp
moteus::Stm32I2c::Options i2c_options;
i2c_options.sda = MBED_CONF_APP_I2C_SDA;
i2c_options.scl = MBED_CONF_APP_I2C_SCL;

moteus::Stm32I2c i2c(i2c_options);

// Start a read operation
i2c.StartReadMemory(slave_address, register_address, buffer);

// In a loop, continuously call poll
while (i2c.busy()) {
  i2c.Poll();
  // Other tasks
}

// Check the result of the read operation
auto status = i2c.CheckRead();
if (status == moteus::Stm32I2c::ReadStatus::kComplete) {
  // Read operation successful
}
```

In this example, an I2C read operation is initiated, and the `Poll` method is called in a loop to process the communication. Once the operation is complete, the result is checked with `CheckRead`.

To fully utilize this class, one needs to understand the specifics of the STM32 I2C peripheral and the mbed framework. The class provides a more user-friendly interface to the somewhat complex and low-level nature of I2C communication on STM32 microcontrollers.
#pragma once

#include "mbed.h"

#include "mjlib/base/string_span.h"

#include "fw/stm32_i2c_timing.h"

namespace moteus {

class Stm32I2c {
 public:
  struct Options {
    PinName sda = NC;
    PinName scl = NC;
    int frequency = 400000;
    I2cMode i2c_mode = I2cMode::kFast;
  };

  Stm32I2c(const Options& options) : options_(options) {
    Initialize();
  }

  void Initialize() {
    i2c_init(&mbed_i2c_, options_.sda, options_.scl);
    i2c_ = mbed_i2c_.i2c.handle.Instance;
    // The mbed libraries only generate timings for a small number of
    // fixed scenarios.  We aren't in those, so we'll rely on a number
    // from CubeMX.
    i2c_->CR1 &= ~(I2C_CR1_PE);

    // PE must be low for a bit, so wait.
    for (int i = 0; i < 1000; i++);

    // Now figure out the actual timing values.
    TimingInput timing_input;
    timing_input.peripheral_hz = HAL_RCC_GetSysClockFreq();
    timing_input.i2c_hz = options_.frequency;
    timing_input.i2c_mode = options_.i2c_mode;

    const auto timing = CalculateI2cTiming(timing_input);
    if (timing.error) {
      // These values weren't achievable.  Mark everything as an
      // error.
      valid_ = false;
    } else {
      valid_ = true;
      i2c_->CR1 =
          (timing.digital_noise_filter << I2C_CR1_DNF_Pos) |
          (((timing_input.analog_filter == AnalogFilter::kOff)
            ? 1 : 0) << I2C_CR1_ANFOFF_Pos) |
          0;
      i2c_->TIMINGR = timing.timingr;
    }

    // Now re-enable and wait a bit.
    i2c_->CR1 |= (I2C_CR1_PE);
    for (int i = 0; i < 1000; i++);
  }

  void StartReadMemory(uint8_t slave_address,
                       uint8_t address,
                       mjlib::base::string_span data) {
    if (!valid_) { return; }
    if (mode_ != Mode::kIdle ||
        (i2c_->CR2 & I2C_CR2_START) != 0 ||
        (i2c_->ISR & I2C_ISR_BUSY) != 0) {
      mode_ = Mode::kError;
      return;
    }

    slave_address_ = slave_address;
    rx_data_ = data;

    i2c_->ICR |= (I2C_ICR_STOPCF | I2C_ICR_NACKCF);

    i2c_->CR2 = (
        I2C_CR2_START |
        // I2C_CR2_RD_WRN | // we are writing the address to read from
        // I2C_CR2_AUTOEND | // we are going to send a repeated start
        (1 << I2C_CR2_NBYTES_Pos) |
        ((slave_address_ << 1) << I2C_CR2_SADD_Pos) |
        0);
    i2c_->TXDR = address;

    mode_ = Mode::kSentRegisterRead;
  }

  void StartWriteMemory(uint8_t slave_address,
                        uint8_t address,
                        std::string_view data) {
    if (!valid_) { return; }
    if (mode_ != Mode::kIdle ||
        (i2c_->CR2 & I2C_CR2_START) != 0 ||
        (i2c_->ISR & I2C_ISR_BUSY) != 0) {
      mode_ = Mode::kError;
      return;
    }

    tx_data_ = data;

    i2c_->CR2 = (
        I2C_CR2_START |
        I2C_CR2_AUTOEND |
        ((1 + data.size()) << I2C_CR2_NBYTES_Pos) |
        ((slave_address << 1) << I2C_CR2_SADD_Pos) |
        0);
    i2c_->TXDR = address;

    offset_ = 0;
    mode_ = Mode::kWritingData;
  }

  enum class ReadStatus {
    kNoStatus,
    kComplete,
    kError,
  };

  ReadStatus CheckRead() {
    if (!valid_) {
      return ReadStatus::kError;
    }

    if (mode_ == Mode::kComplete) {
      mode_ = Mode::kIdle;
      return ReadStatus::kComplete;
    }
    if (mode_ == Mode::kError) {
      // Re-initialize.
      Initialize();

      mode_ = Mode::kIdle;
      return ReadStatus::kError;
    }
    return ReadStatus::kNoStatus;
  }

  void Poll() {
    switch (mode_) {
      case Mode::kIdle:
      case Mode::kComplete:
      case Mode::kError: {
        break;
      }
      case Mode::kSentRegisterRead: {
        if ((i2c_->ISR & I2C_ISR_TC) == 0) {
          break;
        }

        // Clear any NACKs
        i2c_->ICR |= I2C_ICR_NACKCF;

        // Now we send our repeated start to retrieve the result.
        i2c_->CR2 =
            (I2C_CR2_START |
             I2C_CR2_RD_WRN |  // we need to read the data
             I2C_CR2_AUTOEND |
             ((slave_address_ << 1) << I2C_CR2_SADD_Pos) |
             (rx_data_.size() << I2C_CR2_NBYTES_Pos) |
             0);

        offset_ = 0;
        mode_ = Mode::kReadingData;

        break;
      }
      case Mode::kReadingData: {
        if ((i2c_->ISR & I2C_ISR_RXNE) == 0) {
          break;
        }

        // We have data.
        rx_data_[offset_++] = i2c_->RXDR;

        if (offset_ >= rx_data_.size()) {
          // Clear any NACKs
          i2c_->ICR |= I2C_ICR_NACKCF;

          mode_ = Mode::kComplete;
        }

        break;
      }
      case Mode::kWritingData: {
        if ((i2c_->ISR & I2C_ISR_TXE) == 0) {
          break;
        }

        // We can send the next byte.
        if (offset_ >= static_cast<int32_t>(tx_data_.size())) {
          // We are done.
          i2c_->ICR |= I2C_ICR_NACKCF;

          mode_ = Mode::kComplete;
        } else {
          i2c_->TXDR = tx_data_[offset_];
          offset_++;
        }
        break;
      }
    }

    if (i2c_->ISR & I2C_ISR_NACKF) {
      mode_ = Mode::kError;
      i2c_->ICR |= I2C_ICR_NACKCF;
      return;
    }
  }

  bool busy() const {
    switch (mode_) {
      case Mode::kIdle:
      case Mode::kComplete:
      case Mode::kError: {
        return false;
      }
      case Mode::kSentRegisterRead:
      case Mode::kReadingData:
      case Mode::kWritingData: {
        return true;
      }
    }
    MJ_ASSERT(false);
    return false;
  }

 private:
  const Options options_;
  bool valid_ = false;
  i2c_t mbed_i2c_;

  enum class Mode {
    kIdle,
    kSentRegisterRead,
    kReadingData,
    kWritingData,
    kComplete,
    kError,
  };

  Mode mode_ = Mode::kIdle;
  I2C_TypeDef* i2c_ = nullptr;
  uint8_t slave_address_ = 0;
  mjlib::base::string_span rx_data_;
  std::string_view tx_data_;
  int32_t offset_ = 0;
};

}
