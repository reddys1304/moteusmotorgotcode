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


ChatGPT
This code defines a MillisecondTimer class for precise timing operations on embedded systems using the STM32 microcontroller. It initializes a hardware timer to provide millisecond and microsecond timing capabilities. The timer's bit width depends on the target platform (32-bit or 16-bit). It offers methods to read milliseconds and microseconds, perform time subtraction, and provide precise wait_ms and wait_us functions for time delays. This class is essential for time-sensitive tasks in embedded systems and ensures accurate timing control.

The provided code is a C++ class definition, part of the `mjbots/moteus` GitHub repository. This class, named `MillisecondTimer`, is designed to provide timing functionality, specifically for managing time intervals in milliseconds and microseconds. Let's analyze its key components and functionality:

### Class `MillisecondTimer`
- Located in the `moteus` namespace.
- It's designed to work with the `mbed` framework, which is a platform and operating system for IoT devices based on ARM microcontrollers.

### Constructor
- The constructor initializes a hardware timer (`TIM_MST`), which is a common component in microcontrollers used for keeping track of time or creating delays.
- The timer's settings (such as `Period`, `Prescaler`, `ClockDivision`, and `CounterMode`) are configured for 1 microsecond tick intervals.

### TimerType
- A typedef (`TimerType`) is used to define the type of the timer's counter. This type changes based on the width of the timer (`TIM_MST_BIT_WIDTH`), which can be either 32-bit or 16-bit, indicating a hardware-specific detail.

### Methods
1. **read_ms()**
   - Returns the current timer count in milliseconds.

2. **read_us()**
   - Returns the current timer count in microseconds.

3. **subtract_us(a, b)**
   - A static method to subtract two `TimerType` values, used to calculate time intervals.

4. **wait_ms(delay_ms)**
   - Pauses execution for the specified number of milliseconds by internally calling `wait_us()`.

5. **wait_us(delay_us)**
   - Pauses execution for the specified number of microseconds.
   - Handles longer delays by breaking them into smaller intervals (50,000 microseconds) to maintain timer accuracy.

6. **wait_us_helper(delay_us)**
   - An internal helper method for `wait_us`. It accurately waits for the specified microseconds, accounting for potential uncertainty in the start time within a microsecond.

### Private Member
- `TIM_HandleTypeDef handle_`: A structure used to handle the timer functionalities, likely part of the mbed or hardware-specific library.

### Usage Context
This class is likely used in robotic applications where precise timing and delays are essential, such as controlling motors, reading sensors at regular intervals, or implementing non-blocking delays in the system. The use of a hardware timer ensures high accuracy and efficiency compared to software-based timing, which is crucial in real-time systems like robotics. 

This class is a good example of low-level hardware interfacing in C++ for embedded systems, demonstrating direct interaction with microcontroller hardware features.

If you have more code segments or specific questions related to this `MillisecondTimer` class or its interaction with other parts of the system, feel free to share!

#pragma once

#include "mbed.h"

#ifdef wait_us
#undef wait_us
#endif

namespace moteus {

class MillisecondTimer {
 public:
  MillisecondTimer() {
    TIM_MST_RCC;

    constexpr int kExtraPrescaler =
#if defined(TARGET_STM32G4)
        1
#else
#error "Unknown target"
#endif
        ;

    handle_.Instance = TIM_MST;
#if TIM_MST_BIT_WIDTH == 32
    handle_.Init.Period = 0xffffffff;
#elif TIM_MST_BIT_WIDTH == 16
    handle_.Init.Period = 0xffff;
#endif
    handle_.Init.Prescaler =
        (uint32_t)(SystemCoreClock / kExtraPrescaler /
                   1000000U) - 1;  // 1 us tick
    handle_.Init.ClockDivision = 0;
    handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle_);
  }

#if TIM_MST_BIT_WIDTH == 32
  using TimerType = uint32_t;
#elif TIM_MST_BIT_WIDTH == 16
  using TimerType = uint16_t;
#else
# error "unsupported timer"
#endif

  TimerType read_ms() {
    return TIM_MST->CNT / 1000;
  }

  TimerType read_us() {
    return TIM_MST->CNT;
  }

  static TimerType subtract_us(TimerType a, TimerType b) {
    return static_cast<TimerType>(a - b);
  }

  void wait_ms(uint32_t delay_ms) {
    wait_us(delay_ms * 1000);
  }

  void wait_us(uint32_t delay_us) {
    while (delay_us > 50000) {
      wait_us_helper(50000);
      delay_us -= 50000;
    }
    wait_us_helper(delay_us);
  }

  void wait_us_helper(uint32_t delay_us) {
    TimerType current = TIM_MST->CNT;
    TimerType elapsed = 0;
    while (true) {
      const TimerType next = TIM_MST->CNT;
      elapsed += static_cast<TimerType>(next - current);
      // We check delay_us + 1 since we don't know where in the
      // current microsecond we started.
      if (elapsed >= (delay_us + 1)) { return; }
      current = next;
    }
  }

 private:
  TIM_HandleTypeDef handle_ = {};
};

}
