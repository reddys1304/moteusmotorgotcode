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
The code snippet you've provided defines a conditional compilation directive, `MOTEUS_CCM_ATTRIBUTE`, which is used to set specific attributes for functions or variables in the `mjbots/moteus` codebase, particularly for the STM32G4 microcontroller target.

### Explanation of the Directive
- `#if defined(TARGET_STM32G4)`: This line checks if the `TARGET_STM32G4` macro is defined. This macro would typically be defined during the compilation process, indicating that the code is being compiled for the STM32G4 series of microcontrollers.
- `__attribute__ ((section (".ccmram")))`: This is a GCC (GNU Compiler Collection) specific attribute. It tells the compiler to place the function or variable in a specific section of memory, in this case, `.ccmram`. CCMRAM stands for Core-Coupled Memory RAM, which is a type of memory available on certain ARM microcontrollers, including the STM32 series.
  - CCMRAM is typically faster than the regular RAM and is closely coupled to the CPU, providing efficient access.
  - It's often used for performance-critical code, such as interrupt service routines or frequently used data, to enhance the system's overall performance.
- `#else` / `#endif`: These lines ensure that if `TARGET_STM32G4` is not defined, the `MOTEUS_CCM_ATTRIBUTE` macro expands to nothing, meaning it won't modify the placement of functions or variables.

### Usage in the `mjbots/moteus` Codebase
- In the `mjbots/moteus` repository, this directive is likely used to optimize certain functions or data structures for performance when compiled for the STM32G4 target.
- For example, critical functions or frequently accessed data might be annotated with `MOTEUS_CCM_ATTRIBUTE` to ensure they're placed in CCMRAM for faster execution.

### Importance in Embedded Systems
- In embedded systems, especially those with real-time constraints like robotic applications, performance optimization is crucial.
- Utilizing special hardware features like CCMRAM can significantly improve the responsiveness and efficiency of the system.

This directive is an excellent example of how low-level hardware features can be leveraged in software to optimize performance, a common practice in embedded systems development.






#pragma once

#if defined(TARGET_STM32G4)
#define MOTEUS_CCM_ATTRIBUTE __attribute__ ((section (".ccmram")))
#else
#define MOTEUS_CCM_ATTRIBUTE
#endif
