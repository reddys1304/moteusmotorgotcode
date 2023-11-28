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

#include "fw/stm32_gpio_interrupt_in.h"

namespace moteus {

Stm32GpioInterruptIn::Callback Stm32GpioInterruptIn::entries_[Stm32GpioInterruptIn::kMaxCallbacks] = {};

}


This code snippet defines the `Stm32GpioInterruptIn` namespace within the `moteus` project. The `Stm32GpioInterruptIn` class appears to be designed for handling GPIO interrupts on an STM32 microcontroller. However, the provided snippet only shows a small part of the class, specifically the initialization of a static member variable.

### Key Components:

- **Namespace (`moteus`):** The code is part of the `moteus` namespace, which suggests it's part of a larger project, likely related to motor control or robotics.

- **Class (`Stm32GpioInterruptIn`):**
  - This class is intended to handle GPIO interrupts. GPIO (General-Purpose Input/Output) interrupts are used in microcontrollers to respond to changes in input pins, such as a button press or a sensor signal.

- **Static Member Variable (`entries_`):**
  - `entries_` is a static array of `Callback` objects.
  - `Callback` is presumably a type (possibly a function pointer or a functor) used to store the callback functions that will be called when the corresponding GPIO interrupt occurs.
  - `kMaxCallbacks` defines the maximum number of callbacks that can be stored, which is not shown in the snippet but is likely defined elsewhere in the class.

### Functionality and Usage:

While the complete functionality of the `Stm32GpioInterruptIn` class isn't clear from this snippet, we can infer its general purpose:

- **GPIO Interrupt Handling:** The class likely provides methods to attach callbacks to specific GPIO pins. When a pin state changes and triggers an interrupt, the corresponding callback function is called.

- **Efficient Response to Hardware Events:** By using interrupts, the microcontroller can efficiently respond to changes in hardware state without constantly polling the pin.

### Example Usage:

A typical use case might look something like this (hypothetical, as the full class definition is not provided):

```cpp
void onPinChange() {
    // Handle pin change
}

// Inside some initialization function
moteus::Stm32GpioInterruptIn::attachInterrupt(PIN_X, onPinChange, RISING);
```

In this example, `attachInterrupt` is a hypothetical method that configures an interrupt on a specific pin (`PIN_X`) and attaches the `onPinChange` function as a callback, which is called on the rising edge of the signal.

To fully understand and utilize this class, the complete class definition including its methods and the exact nature of the `Callback` type would be necessary.