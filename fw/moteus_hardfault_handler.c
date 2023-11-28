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
This C code snippet is an implementation of a Hard Fault handler for an ARM Cortex-M processor, typically used in embedded systems like the STM32 microcontroller series. The Hard Fault handler is a critical component of the firmware, dealing with severe errors during the execution of the program. Here's a breakdown of what this function does:

### Function: `hard_fault_handler_c`

- **Purpose**: This function is called when the processor encounters a Hard Fault, a type of fault that cannot be recovered from and usually indicates a serious error in the program, such as accessing invalid memory.

### Extracting Fault Information

- **Stacked Registers**: The function takes an argument, `hardfault_args`, which is a pointer to a set of registers saved by the processor on the stack upon entering the Hard Fault handler. These registers are:
  - `R0-R3`: General-purpose registers.
  - `R12`: Another general-purpose register.
  - `LR` (Link Register): Holds the return address.
  - `PC` (Program Counter): Holds the address of the instruction that caused the fault.
  - `PSR` (Program Status Register): Contains the processor's status information.

- The values of these registers at the time of the fault are extracted into variables, which can be used for debugging purposes. The `(void)` cast is used to avoid compiler warnings about unused variables.

### Safety Measures

- **Disabling the Motor Driver**: The function includes specific code to disable a motor driver, which is important in hardware like motor controllers to prevent damage or dangerous behavior in the case of a system fault. The specific lines of code:
  - `GPIOC->BSRR = (1 << (3 + 16));`
  - `GPIOA->BSRR = (1 << (3 + 16));`

  These lines use direct memory access to set specific bits in the GPIO port's Bit Set/Reset Register (BSRR), which controls the pins connected to the motor driver, effectively turning it off.

### Infinite Loop

- The function ends with an infinite loop (`while (1);`). This is common in fault handlers as it prevents the program from continuing to run and potentially causing further errors or damage.

### Conclusion

This Hard Fault handler is tailored to a specific application, likely a motor controller, and includes both debugging information and safety mechanisms. The extraction of register values can be invaluable for diagnosing the cause of the fault, and the immediate disabling of the motor driver is crucial for safety. This handler is an essential part of robust and safe firmware for embedded systems like motor controllers.
#include "fw/stm32.h"

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  volatile unsigned int stacked_r0;
  volatile unsigned int stacked_r1;
  volatile unsigned int stacked_r2;
  volatile unsigned int stacked_r3;
  volatile unsigned int stacked_r12;
  volatile unsigned int stacked_lr;
  volatile unsigned int stacked_pc;
  volatile unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  (void)stacked_r0;
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  (void)stacked_r1;
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  (void)stacked_r2;
  stacked_r3 = ((unsigned long) hardfault_args[3]);
  (void)stacked_r3;

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  (void)stacked_r12;
  stacked_lr = ((unsigned long) hardfault_args[5]);
  (void)stacked_lr;
  stacked_pc = ((unsigned long) hardfault_args[6]);
  (void)stacked_pc;
  stacked_psr = ((unsigned long) hardfault_args[7]);
  (void)stacked_psr;

  // Do our best to disable the motor driver, so we cause fewer
  // explosions!

  GPIOC->BSRR = (1 << (3 + 16));
  GPIOA->BSRR = (1 << (3 + 16));

  // TODO(jpieper): Verify that the 8323 pin assignments match these
  // hard-coded constants.

  while (1);
}
