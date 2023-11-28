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

The `Stm32Dma` class in the provided code is designed for managing Direct Memory Access (DMA) channels and DMA multiplexer (DMAMUX) settings in STM32 microcontrollers, particularly within the STM32G4 series. This is part of the `mjbots/moteus` repository, geared towards embedded systems for robotic applications. Let's delve into the key aspects:

### Overview
- The class is within the `moteus` namespace and is meant for use with the `mbed` framework, indicating it's for ARM Cortex-M based microcontrollers.

### Functionality
1. **FindChannelIndex**
   - Given a DMA channel (represented by `DMA_Channel_TypeDef*`), this function calculates its index.
   - The index is determined based on the address difference between the given channel and the base channel of either DMA1 or DMA2, normalized by the address difference between two consecutive channels.
   - The result is shifted left by 2 bits (`<< 2`), which likely aligns with the bit position used in further configuration or status registers.

2. **SelectDmamux**
   - This function selects the appropriate DMAMUX channel for a given DMA channel.
   - The STM32G4 series has a DMAMUX hardware block that allows flexible routing of DMA requests to DMA channels. This function calculates the correct DMAMUX channel based on the provided DMA channel.
   - The calculation considers different STM32G4 sub-families (as seen in the `#if defined` directives), which have variations in DMAMUX channel configurations.
   - The function uses `FindChannelIndex` to help calculate the correct DMAMUX channel.

3. **u32**
   - A template function to cast a pointer or other value to a `uint32_t`. This is commonly used in embedded systems programming for dealing with registers and memory-mapped hardware.

### Usage in Embedded Systems
- The `Stm32Dma` class encapsulates lower-level, hardware-specific details of DMA configuration in STM32 microcontrollers, simplifying DMA setup and management in other parts of the code.
- Efficient use of DMA is critical in high-performance embedded systems, like those in robotics, where CPU offloading and high-speed data transfers are essential for real-time operation.

### Integration with Robotics
In the context of the `mjbots/moteus` project, this class likely plays a crucial role in handling high-speed data transfer tasks, such as sensor data acquisition or motor control signals, without burdening the main CPU, thus ensuring smooth and efficient robotic operations.

This kind of abstraction and encapsulation of hardware-specific details is a common practice in embedded systems development, making the code more portable, readable, and maintainable.

#pragma once

#include "mbed.h"

namespace moteus {

class Stm32Dma {
 public:
  static uint32_t FindChannelIndex(DMA_Channel_TypeDef* channel) {
    if (channel < DMA2_Channel1) {
      return ((u32(channel) - u32(DMA1_Channel1)) /
              (u32(DMA1_Channel2) - u32(DMA1_Channel1))) << 2;
    }
    return ((u32(channel) - u32(DMA2_Channel1)) /
            (u32(DMA2_Channel2) - u32(DMA2_Channel1))) << 2;
  }

  static DMAMUX_Channel_TypeDef* SelectDmamux(DMA_Channel_TypeDef* channel) {
    const auto base = (channel < DMA2_Channel1) ? DMAMUX1_Channel0 :
#if defined (STM32G471xx) || defined (STM32G473xx) || defined (STM32G474xx) || defined (STM32G483xx) || defined (STM32G484xx)
        DMAMUX1_Channel8;
#elif defined (STM32G431xx) || defined (STM32G441xx) || defined (STM32GBK1CB)
    DMAMUX1_Channel6;
#else
    DMAMUX1_Channel7;
#endif /* STM32G4x1xx) */
    return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
        u32(base) + (FindChannelIndex(channel) >> 2U) *
        (u32(DMAMUX1_Channel1) - u32(DMAMUX1_Channel0)));
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }
};

}
