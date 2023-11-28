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
This code snippet defines the `Stm32DigitalMonitor` class within the `moteus` namespace. This class is designed to read the state of a digital input pin on an STM32 microcontroller without configuring the pin, meaning it reads the current state of the pin directly without altering its mode (input, output, etc.) or other settings. It's part of a larger project for controlling and interfacing with hardware, likely in a robotics context.

### Key Components:

- **Constructor (`Stm32DigitalMonitor(PinName pin)`):** 
  - Takes a `PinName` as an argument, which represents the GPIO pin to be monitored.
  - Determines the GPIO port (`GPIO_TypeDef*`) that corresponds to the provided `PinName`.
  - Assigns `reg_in_` to the input data register (`IDR`) of the determined GPIO port.
  - `mask_` is set to a bitmask corresponding to the specific pin within the GPIO port.

- **`read()` Method:**
  - Returns a boolean value indicating the state of the specified pin (high or low).
  - It checks if the corresponding bit in the `IDR` register is set or not, using the `mask_`.

### Functionality:

- This class provides a direct and minimalistic way to read the state of a digital pin.
- It is useful in scenarios where the pin state needs to be read without changing the pin configuration or in a performance-critical context where minimal overhead is required.
- The class avoids the overhead of Mbed's `DigitalIn` class, which configures the pin mode and other settings.

### Usage:

To use `Stm32DigitalMonitor` in a project:
1. Create an instance of `Stm32DigitalMonitor`, specifying the pin to monitor.
2. Call the `read()` method to get the current state of the pin.

### Example:

```cpp
#include "Stm32DigitalMonitor.h"

// Assuming pin PA_0 needs to be monitored
moteus::Stm32DigitalMonitor pinMonitor(PA_0);

int main() {
    while (true) {
        bool pinState = pinMonitor.read();
        // Use pinState as needed
    }
}
```

This class is particularly useful in embedded systems where direct hardware access is required for performance reasons or when existing frameworks provide unnecessary overhead for simple tasks like reading a digital input.

#pragma once

#include "mbed.h"

namespace moteus {

/// Read a digital input, but without configuring it in any way.
class Stm32DigitalMonitor {
 public:
  Stm32DigitalMonitor(PinName pin) {
    const uint32_t port_index1 = STM_PORT(pin);
    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index1) {
        case PortA: return GPIOA_BASE;
        case PortB: return GPIOB_BASE;
        case PortC: return GPIOC_BASE;
        case PortD: return GPIOD_BASE;
        case PortE: return GPIOE_BASE;
        case PortF: return GPIOF_BASE;
      }
      MJ_ASSERT(false);
      return GPIOA_BASE;
      }());
    reg_in_ = &gpio->IDR;
    mask_ = static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin) & 0xf));
  }

  bool read() {
    return (*reg_in_ & mask_) != 0;
  }

 private:
  volatile uint32_t* reg_in_ = nullptr;
  uint32_t mask_ = 0;
};

}
