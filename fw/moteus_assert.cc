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
This C++ code is part of the firmware for the moteus motor controller, which seems to be designed for embedded systems using the mbed OS platform. The code focuses on handling assertion failures and defining a function to be called when the system should terminate (`mbed_die`). Here's a detailed overview:

### Namespaces and External Functions
1. **Namespace mjlib::base**: Contains the `assertion_failed` function, which is likely used throughout the firmware for debugging and error-checking purposes.
2. **External "C"**: The `mbed_die` function is declared within an `extern "C"` block, ensuring C linkage for this function. This is important for compatibility with C libraries and mbed OS internals, which are generally C-based.

### Assertion Handling
- **assertion_failed Function**: This function is a custom implementation for handling assertions in the firmware. It calls `mbed_assert_internal`, an mbed OS function, providing details about the failed assertion (such as the expression, file name, and line number). This is crucial for debugging and identifying issues during firmware development.

### System Termination Handling (`mbed_die`)
- **mbed_die Function**: This function is designed to be called when the system needs to shut down, typically due to a critical error or failure condition. Its responsibilities include:
    1. **Motor Controller Safety**: Calls `moteus::MoteusEnsureOff` to ensure that the motor controller is safely turned off. This is a crucial safety measure to prevent unintended motor movements or damage in case of system failure.
    2. **LED Indication**: Initializes and controls a debug LED defined by `g_hw_pins.debug_led1`. The LED blinks in a loop, serving as an indication that the system has encountered a critical error and has ceased normal operation.

### Overall Functionality
- The code is a part of the moteus firmware, providing mechanisms for error handling and system termination. The focus is on ensuring safety (especially regarding motor control) and providing clear indications (via LED) when the system encounters a critical error.
- The integration with mbed OS is evident in the use of mbed-specific functions and APIs, like `mbed_assert_internal` and `gpio_init_out`.
- Such error handling and safety mechanisms are essential in embedded systems, especially those controlling hardware like motors, where software errors can have real-world consequences.

### Conclusion
This code snippet plays a critical role in the reliability and safety of the moteus motor controller firmware. It ensures that in the event of a system failure, the hardware is brought to a safe state, and the failure condition is clearly indicated, aiding in diagnostics and maintenance.
#include "mbed_assert.h"
#include "mbed.h"

#include "hal/gpio_api.h"

#include "fw/moteus_hw.h"

namespace mjlib {
namespace base {

void assertion_failed(const char* expression, const char* filename, int line) {
  mbed_assert_internal(expression, filename, line);
}

}
}

extern "C" {
void mbed_die(void) {
  // We want to ensure the motor controller is disabled and flash an
  // LED which exists.
  moteus::MoteusEnsureOff();

  gpio_t led;
  gpio_init_out(&led, moteus::g_hw_pins.debug_led1);

  // Now flash an actual LED.
  for (;;) {
    gpio_write(&led, 0);
    wait_ms(200);
    gpio_write(&led, 1);
    wait_ms(200);
  }
}
}
