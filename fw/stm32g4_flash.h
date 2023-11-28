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
This C++ header file defines the `Stm32G4Flash` class, a part of the `moteus` project, specifically designed for handling flash memory operations on STM32G4 series microcontrollers. This class implements the `mjlib::micro::FlashInterface` and provides methods to interact with the flash memory.

### Overview

- The `Stm32G4Flash` class provides functionality to erase, lock, unlock, and program the flash memory of STM32G4 microcontrollers.

### Key Components

- **`GetInfo` Method**: Returns information about the flash memory segment that this interface manages. It specifies the start and end addresses of the last 4k block of flash memory.

- **`Erase` Method**: Erases the flash memory. This is typically required before programming new data into flash.

- **`Unlock` Method**: Unlocks the flash memory for write operations. Flash memory needs to be unlocked before it can be erased or programmed.

- **`Lock` Method**: Locks the flash memory, preventing any write operations. This is usually done after programming is completed.

- **`ProgramByte` Method**: Programs a single byte into flash memory. This method handles the accumulation of bytes into a 64-bit word and programs it into flash when full.

### Usage

- This class is used in applications where data needs to be persistently stored in the microcontroller's flash memory, such as configuration settings or non-volatile data storage.

- It is suitable for embedded systems based on the STM32G4 series where flash memory operations are required.

### Advantages

- **Ease of Use**: Provides a simple interface for flash operations like erase and program, abstracting the underlying complexity.
- **Efficiency**: Accumulates bytes to be written into a 64-bit word, reducing the number of programming operations.
- **Safety**: Includes checks and procedures to ensure safe operations on flash memory (e.g., locking and unlocking mechanisms).

### Considerations

- The implementation is specific to STM32G4 microcontrollers and utilizes STM32 HAL functions.
- Flash memory has a limited number of write cycles, so it's important to use this class judiciously to extend the lifespan of the flash memory.
- Erasing flash is a time-consuming operation and may block the CPU for a significant duration.
- Incorrect usage can lead to data corruption or loss. It's crucial to follow proper procedures for unlocking, erasing, programming, and locking the flash.
#pragma once

#include "mbed.h"

#include "mjlib/micro/flash.h"

namespace moteus {

class Stm32G4Flash : public mjlib::micro::FlashInterface {
 public:
  Stm32G4Flash() {}
  ~Stm32G4Flash() override {}

  Info GetInfo() override {
    Info result;
    // The final 4k of flash
    result.start = reinterpret_cast<char*>(0x807f000);
    result.end = result.start + 0x1000;
    return result;
  }

  void Erase() override {
    uint32_t page_err = 0;
    FLASH_EraseInitTypeDef erase_options{};
    erase_options.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_options.Banks = FLASH_BANK_2;
    erase_options.Page = 126;
    erase_options.NbPages = 2;
    if (HAL_FLASHEx_Erase(&erase_options, &page_err) != HAL_OK) {
      mbed_die();
    }
    if (page_err != 0xffffffff) {
      mbed_die();
    }
  }

  void Unlock() override {
    HAL_FLASH_Unlock();
  }

  void Lock() override {
    if (shadow_bits_) {
      FlushWord();
    }
    HAL_FLASH_Lock();
  }

  void ProgramByte(char* ptr, uint8_t value) override {
    const uint32_t intaddr = reinterpret_cast<uint32_t>(ptr);
    const uint32_t this_shadow = intaddr & ~(0x7);
    const uint32_t offset = intaddr & 0x7;

    if (this_shadow != shadow_start_ && shadow_start_ != 0) {
      FlushWord();
    }

    shadow_start_ = this_shadow;

    shadow_ |= (static_cast<uint64_t>(value) << (offset * 8));
    shadow_bits_ |= (0xffull << (offset * 8));

    if (shadow_bits_ == 0xffffffffffffffffull) {
      FlushWord();
    }
  }

 private:
  void FlushWord() {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, shadow_start_, shadow_);
    shadow_start_ = 0;
    shadow_ = 0;
    shadow_bits_ = 0;
  }

  uint32_t shadow_start_ = 0;
  uint64_t shadow_ = 0;
  uint64_t shadow_bits_ = 0;
};

}
