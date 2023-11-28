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
This snippet presents the implementation of a `ClockManager` class, part of a larger robotics or embedded systems project, likely intended for controlling and configuring the system's internal clock. The class is designed to work within an embedded software environment, specifically using the Mbed framework, which is evident from the inclusion of the `mbed.h` header. Here's an overview of its key components and functionalities:

### 1. **Header Inclusions**
   - `mbed.h`: Includes the Mbed framework, which provides APIs for various microcontroller functionalities.
   - `mjlib/micro/async_stream.h`, `mjlib/micro/command_manager.h`, `mjlib/micro/persistent_config.h`: These headers are part of the `mjlib` library, likely a custom library for handling asynchronous operations, command management, and configuration persistence.

### 2. **ClockManager Class**
   - **Purpose**: Manages the clock settings of the microcontroller, particularly the High-Speed Internal (HSI) clock trim value.
   - **Configuration**: Utilizes `mjlib::micro::PersistentConfig` for storing persistent settings, which likely includes the clock trim configuration.

### 3. **Key Methods**
   - `UpdateConfig()`: Updates the HSI trim value based on the current configuration. It ensures that the trim value stays within valid bounds.
   - `SetTrim(int)`: Sets an additional trim value to fine-tune the clock frequency. This method includes bounds checking to ensure the trim value remains within a specified range (`kMaxExtraTrim`).
   - `Command(...)`: Processes incoming commands to either report the current microsecond count or adjust the clock trim. It uses `mjlib::micro::CommandManager` for handling command input and output, showcasing an interactive component of the system.

### 4. **Clock Configuration**
   - The class directly manipulates the `RCC->ICSCR` register, which is specific to STM32 microcontrollers. This register is used to adjust the calibration of the internal high-speed clock.
   - The `hsitrim` field in the `Config` struct is used to store the base trim value for the HSI clock.

### 5. **Utility and Safety**
   - Implements safety checks to ensure the clock trim values are within acceptable limits.
   - Provides a structured approach to modifying critical system parameters (like clock settings) with safeguards against invalid configurations.

### Usage
- **Embedded Systems**: Ideal for robotics, control systems, or any embedded application using an STM32 microcontroller where precise timing control is necessary.
- **Configuration Management**: Useful in scenarios where runtime configuration of system parameters is required, especially in field-deployed systems where physical access might be limited.

### Overall Impression
The `ClockManager` is a specialized component designed for embedded systems requiring precise control and adjustment of the microcontroller's internal clock. It demonstrates an integration of low-level hardware control with higher-level command handling and configuration management, essential for robust and flexible embedded system design.
#pragma once

#include "mbed.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"

#include "fw/millisecond_timer.h"

namespace moteus {

class ClockManager {
 public:
  static constexpr int kMaxExtraTrim = 8;

  ClockManager(MillisecondTimer* timer,
               mjlib::micro::PersistentConfig& persistent_config,
               mjlib::micro::CommandManager& command_manager)
      : timer_(timer) {
    persistent_config.Register("clock", &clock_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("clock", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void UpdateConfig() {
    const int32_t trim =
        std::max<int32_t>(
            0, std::min<int32_t>(127, clock_.hsitrim + extra_trim_));
    RCC->ICSCR = (RCC->ICSCR & ~0xff000000) | (trim << 24);
  }

  void SetTrim(int extra_trim) {
    extra_trim_ = std::max(-kMaxExtraTrim, std::min(kMaxExtraTrim, extra_trim));
    UpdateConfig();
  }

  int trim() const {
    return extra_trim_;
  }

  void Command(const std::string_view& command,
               const mjlib::micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");
    const auto cmd_text = tokenizer.next();

    if (cmd_text == "us") {
      snprintf(output_, sizeof(output_), "%" PRIu32 "\r\n",
               static_cast<uint32_t>(timer_->read_us()));
      WriteMessage(output_, response);
    } else if (cmd_text == "trim") {
      const auto value_str = tokenizer.next();

      if (value_str == "") {
        snprintf(output_, sizeof(output_), "%d\r\n", extra_trim_);
        WriteMessage(output_, response);
      } else {
        SetTrim(std::strtol(value_str.data(), nullptr, 0));
        WriteMessage("OK\r\n", response);
      }
    } else {
      WriteMessage("ERR unknown clock\r\n", response);
    }
  }

  void WriteMessage(const std::string_view& message,
                    const mjlib::micro::CommandManager::Response& response) {
    mjlib::micro::AsyncWrite(*response.stream, message, response.callback);
  }

 private:
  struct Config {
    int32_t hsitrim = 64;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(hsitrim));
    }
  };

  MillisecondTimer* const timer_;
  Config clock_;
  char output_[16] = {};
  int extra_trim_ = 0;
};

}
