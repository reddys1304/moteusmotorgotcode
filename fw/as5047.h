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

The `AS5047` class in the provided code is designed for interfacing with the AS5047 magnetic rotary encoder using SPI (Serial Peripheral Interface) communication. It's a part of the `mjbots/moteus` project and encapsulates the functionality required to communicate with this specific type of encoder. Let's analyze the key aspects of this class:

### Overview
- This class is within the `moteus` namespace and is intended for use with the `mbed` framework, which is common in ARM Cortex-M microcontroller-based applications.
- It utilizes the previously discussed `Stm32Spi` class to handle SPI communication.

### Structure `Options`
- Inherits the `Stm32Spi::Options` structure to configure SPI communication parameters. This structure would typically include settings like pin assignments, SPI mode, frequency, etc.

### Constructor
- Initializes an instance of the `Stm32Spi` class with the provided SPI options. This initialization sets up the necessary configurations for SPI communication with the AS5047 encoder.

### Methods
1. **Sample**
   - Performs a blocking SPI transfer to sample data from the AS5047 encoder. The `spi_.write(0xffff)` sends a dummy 16-bit value (`0xffff`) to generate clock pulses needed for the SPI read operation.
   - The returned value is masked with `0x3fff` to extract the relevant 14 bits of encoder data and then left-shifted by 2, aligning it as needed.

2. **StartSample**
   - Initiates an SPI write operation to start the sampling process. This is likely part of a non-blocking or asynchronous read operation.

3. **FinishSample**
   - Completes the SPI transfer started by `StartSample` and processes the received data in a similar way to `Sample`.

### Usage in the Context of Robotics
- The `AS5047` class provides a clear interface for reading values from an AS5047 encoder, which is often used in robotics for precise angle measurements in motor control applications.
- The class abstracts the low-level details of SPI communication, allowing higher-level code to interact with the encoder in a simple and straightforward manner.
- The non-blocking approach with `StartSample` and `FinishSample` allows for more efficient use of resources, especially important in real-time systems like robotics, where managing CPU time and avoiding blocking operations are crucial.

In summary, the `AS5047` class is a specialized interface for the AS5047 magnetic rotary encoder, leveraging SPI communication for data transfer. It demonstrates how embedded systems often encapsulate hardware communication protocols within classes for cleaner, more manageable code, particularly in complex applications like robotics.


#pragma once

#include "mbed.h"

#include "hal/spi_api.h"

#include "fw/ccm.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"

namespace moteus {

class AS5047 {
 public:
  using Options = Stm32Spi::Options;

  AS5047(const Options& options)
      : spi_(options) {
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.write(0xffff) & 0x3fff) << 2;
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0xffff);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.finish_write() & 0x3fff);
  }

 private:
  Stm32Spi spi_;
};

}
