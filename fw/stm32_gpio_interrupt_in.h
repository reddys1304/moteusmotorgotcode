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

The `Stm32GpioInterruptIn` class in the provided code is designed for managing GPIO (General Purpose Input/Output) interrupts on STM32 microcontrollers, specifically within the context of the `mjbots/moteus` project. This class is tailored to handle interrupts efficiently, with some unique characteristics tailored to its intended use. Let's analyze its key components and functionalities:

### Overview
- The class is defined within the `moteus` namespace and is intended for use with the `mbed` framework.
- It manages GPIO interrupts, allowing functions to be called in response to changes (both rising and falling edges) on specific GPIO pins.

### Structure `Callback`
- Holds a callback function and its associated data, along with a reference count. The callback function is executed when the interrupt associated with the corresponding GPIO pin is triggered.

### Method `Make`
- A static method that attempts to create an instance of `Stm32GpioInterruptIn` for a specific pin. If the pin already has an interrupt assigned, it returns an empty optional, indicating failure to create the instance.

### Constructor and Destructor
- The constructor sets up the GPIO pin for interrupt detection and configures the NVIC (Nested Vectored Interrupt Controller) for handling the interrupt.
- The destructor cleans up the configuration, disabling the interrupt and reducing the reference count of the associated callback.

### Method `read`
- Returns the current state of the GPIO pin (high or low).

### Static Methods and Variables
- `FindCallback`: Searches for a matching callback or an empty slot in the `entries_` array.
- `FindIrqIndex` and `FindIrqN`: Determine the IRQ (interrupt request) index and type based on the GPIO pin number.
- `ISR_Routine`: A static interrupt service routine that clears the interrupt and calls the appropriate callbacks.
- `entries_`: A static array of `Callback` structures, storing the callbacks for different GPIO pins.
- `irq_count`: A static array used to keep track of the number of active interrupts for each IRQ line.

### Characteristics and Usage
- The class is designed for scenarios where multiple GPIO pins might trigger the same callback function. This is efficient but could result in spurious callbacks if different pins are set to trigger the same function.
- It uses direct register access and NVIC manipulation for efficiency and speed, which is critical in low-level embedded system programming, especially in real-time applications like robotics.
- The use of `std::optional` for the factory method `Make` adds a level of safety, preventing the creation of an interrupt handler if the requested pin already has an interrupt associated with it.

In summary, the `Stm32GpioInterruptIn` class provides a low-level, efficient mechanism for handling GPIO interrupts on STM32 microcontrollers, which is essential in embedded systems where responsive and efficient interrupt handling is crucial. The design choices reflect a balance between efficiency and the specific needs of the `mjbots/moteus` project's application domain.


#pragma once

#include <array>
#include <optional>

#include "mbed.h"

#include "mjlib/base/assert.h"

#include "fw/ccm.h"

namespace moteus {

/// This is brain dead and fast.  It assumes that many pins will have
/// the same callback.  It may result in spurious callbacks.  Use
/// accordingly.
class Stm32GpioInterruptIn {
 public:
  using CallbackFunction = void (*)(uint32_t);

  struct Callback {
    CallbackFunction function = {};
    uint32_t data = 0;
    int ref_count = 0;

    Callback() {}
    Callback(CallbackFunction function_in, uint32_t data_in)
        : function(function_in),
          data(data_in) {}
  };

  static std::optional<Stm32GpioInterruptIn>
  Make(PinName pin, CallbackFunction function, uint32_t data) {
    // See if something already has this interrupt channel claimed.
    const uint32_t pin_index = STM_PIN(pin);
    if (EXTI->IMR1 & (1 << pin_index)) {
      // Something already has this channel. :(
      return {};
    }

    Stm32GpioInterruptIn result{pin, function, data};

    if (!result.entry_) {
      // Somehow we exhausted our entry table.
      return {};
    }

    return result;
  }

  Stm32GpioInterruptIn(Stm32GpioInterruptIn&& rhs)
      : pin_(rhs.pin_),
        entry_(rhs.entry_),
        reg_in_(rhs.reg_in_),
        mask_(rhs.mask_) {
    rhs.pin_ = NC;
  }

  Stm32GpioInterruptIn& operator=(Stm32GpioInterruptIn&& rhs) {
    pin_ = rhs.pin_;
    entry_ = rhs.entry_;
    reg_in_ = rhs.reg_in_;
    mask_ = rhs.mask_;

    rhs.pin_ = NC;

    return *this;
  }

  Stm32GpioInterruptIn(PinName pin, CallbackFunction function, uint32_t val)
      : pin_(pin) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    const uint32_t port_index = STM_PORT(pin);
    const uint32_t pin_index = STM_PIN(pin);

    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index) {
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


    Callback cbk(function, val);
    entry_ = FindCallback(cbk);
    entry_->ref_count++;

    // Set up as GPIO and input.
    pin_function(pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));

    // Configure our port as the one for this pin.
    const auto shift = 4u * (pin_index & 0x03);
    const auto original = SYSCFG->EXTICR[pin_index >> 2] & ~(0x0fu << shift);
    SYSCFG->EXTICR[pin_index >> 2] = original | (port_index << shift);

    // We want both rising and falling edges.
    EXTI->RTSR1 |= (1 << pin_index);
    EXTI->FTSR1 |= (1 << pin_index);

    // Enable the external interrupt.
    EXTI->IMR1 |= (1 << pin_index);

    const auto irq_index = FindIrqIndex(pin_);

    if (irq_count(irq_index) == 0) {
      // We need to configure and enable this IRQ.
      const auto irqn = FindIrqN(irq_index);

      NVIC_SetVector(
          irqn, reinterpret_cast<uint32_t>(
              &Stm32GpioInterruptIn::ISR_Routine));
      HAL_NVIC_SetPriority(irqn, 1, 0);
      NVIC_EnableIRQ(irqn);
    }
    (irq_count(irq_index))++;
  }

  ~Stm32GpioInterruptIn() {
    // Check if we're in the "unspecified but valid state"
    // post-move-from.
    if (pin_ == NC) { return; }

    entry_->ref_count--;

    const auto pin_index = STM_PIN(pin_);

    EXTI->IMR1 &= ~(1 << pin_index);

    const auto irq_index = FindIrqIndex(pin_);

    if (irq_count(irq_index) == 1) {
      // We need to disable this IRQ.
      const auto irqn = FindIrqN(irq_index);
      NVIC_DisableIRQ(irqn);
      NVIC_ClearPendingIRQ(irqn);
    }
    irq_count(irq_index)--;

    if (entry_->ref_count == 0) {
      entry_->function = nullptr;
      entry_->data = 0;
    }
  }

  bool read() {
    return (*reg_in_ & mask_) != 0;
  }

 private:
  static Callback* FindCallback(Callback cbk) {
    // First look for matches.
    for (auto& entry : entries_) {
      if (entry.function == cbk.function &&
          entry.data == cbk.data) {
        return &entry;
      }
    }
    // Then look for empties.
    for (auto& entry : entries_) {
      if (entry.function == nullptr) {
        entry = cbk;
        return &entry;
      }
    }
    return nullptr;
  }

  static int FindIrqIndex(PinName pin) {
    const auto pin_index = STM_PIN(pin);
    if (pin_index <= 4) { return pin_index; }
    if (pin_index <= 9) { return 5; }
    if (pin_index <= 15) { return 6; }
    return 0;
  }

  static IRQn_Type FindIrqN(int index) {
    constexpr IRQn_Type kIrqN[7] = {
      EXTI0_IRQn,
      EXTI1_IRQn,
      EXTI2_IRQn,
      EXTI3_IRQn,
      EXTI4_IRQn,
      EXTI9_5_IRQn,
      EXTI15_10_IRQn,
    };
    return kIrqN[index];
  }

  static void ISR_Routine() MOTEUS_CCM_ATTRIBUTE {
    // Clear everything in one fell swoop!
    EXTI->PR1 = 0x0000ffff;

    for (auto& entry : entries_) {
      if (!entry.function) { continue; }
      entry.function(entry.data);
    }
  }

  static constexpr int kMaxCallbacks = 3;

  static Callback entries_[kMaxCallbacks];

  // There are at most 7 external IRQ lines.
  static int& irq_count(int i) {
    static std::array<int, 7> data = { {} };
    return data[i];
  }

  PinName pin_;

  Callback* entry_ = nullptr;

  volatile uint32_t* reg_in_ = nullptr;
  uint32_t mask_ = 0;
};

}
