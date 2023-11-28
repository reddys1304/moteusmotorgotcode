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
This source file implements the `SystemInfo` class defined in the `moteus` namespace. The `SystemInfo` class provides functionality to monitor various system statistics and metrics, especially for embedded systems in real-time applications like robotics or motor control.

### Key Components:

1. **SystemInfoData Structure**: This nested structure holds various system metrics such as pool size, pool available memory, idle rate, CAN bus reset count, and a millisecond counter. This data structure is used for telemetry purposes.

2. **Constructor**: The `SystemInfo` constructor initializes the `Impl` class, which manages the system information, using references to `mjlib::micro::Pool` and `mjlib::micro::TelemetryManager`. This suggests integration with a telemetry system for monitoring and logging purposes.

3. **Polling Method**: `PollMillisecond` is a method that updates the system metrics every millisecond. It updates the pool size, available memory, idle rate, and increments the millisecond counter. The idle rate is calculated based on the difference in the `idle_count` static variable, which is expected to be incremented in an idle loop elsewhere in the system.

4. **CAN Reset Counter**: The `SetCanResetCount` method allows setting the count of CAN bus resets, indicating integration with a CAN bus system, commonly used in automotive and industrial control systems.

5. **Millisecond Counter**: The `millisecond_counter` method provides the current value of the millisecond counter, tracking system uptime or elapsed time since the last reset.

6. **SystemInfo::Impl Class**: This private inner class actually implements the functionality of the `SystemInfo` class. It includes a pool reference for memory management, a structure to hold system data, and a function for updating telemetry data.

7. **Telemetry Registration**: The `Impl` constructor registers the `SystemInfoData` structure with the telemetry manager, allowing system metrics to be monitored and possibly reported externally.

### Usage and Implications:

- **Embedded Systems Monitoring**: The class is designed for embedded systems where continuous monitoring of system health and performance is crucial.
- **Telemetry and Diagnostics**: The integration with a telemetry system suggests usage in scenarios where remote monitoring or diagnostics is needed.
- **Real-Time Performance**: The millisecond-based polling and tracking of idle count indicate a focus on real-time performance monitoring, essential in high-reliability systems.

### General Observations:

- This class seems to be part of a larger framework or library (`mjlib`) designed for microcontroller-based applications, providing utilities for system health monitoring and diagnostics.
- The class appears well-integrated with a system for memory management (`mjlib::micro::Pool`) and telemetry (`mjlib::micro::TelemetryManager`), indicating a sophisticated approach to embedded system design.
- The use of a millisecond counter starting near the int32 maximum value suggests careful handling of counter overflows, a common concern in long-running embedded systems.
#include "fw/system_info.h"

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/visitor.h"

#include "mjlib/micro/telemetry_manager.h"

namespace moteus {

volatile uint32_t SystemInfo::idle_count = 0;

namespace {
struct SystemInfoData {
  uint32_t pool_size = 0;
  uint32_t pool_available = 0;

  uint32_t idle_rate = 0;
  uint32_t can_reset_count = 0;

  // We deliberately start this counter near to int32 overflow so that
  // any applications that use it will likely have to handle it
  // properly.
  uint32_t ms_count = (1ull<<31) - 300000;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(pool_size));
    a->Visit(MJ_NVP(pool_available));
    a->Visit(MJ_NVP(idle_rate));
    a->Visit(MJ_NVP(can_reset_count));
    a->Visit(MJ_NVP(ms_count));
  }
};
}

class SystemInfo::Impl {
 public:
  Impl(mjlib::micro::Pool& pool, mjlib::micro::TelemetryManager& telemetry)
      : pool_(pool) {
    data_updater_ = telemetry.Register("system_info", &data_);
  }

  void PollMillsecond() {
    data_.ms_count++;
    ms_count_++;
    if (ms_count_ >= 10) {
      ms_count_ = 0;
    } else {
      return;
    }

    data_.pool_size = pool_.size();
    data_.pool_available = pool_.available();

    const auto this_idle_count = idle_count;
    data_.idle_rate = this_idle_count - last_idle_count_;
    last_idle_count_ = this_idle_count;

    data_updater_();
  }

  void SetCanResetCount(uint32_t value) {
    data_.can_reset_count = value;
  }

  mjlib::micro::Pool& pool_;

  uint8_t ms_count_ = 0;
  uint32_t last_idle_count_ = 0;
  SystemInfoData data_;
  mjlib::base::inplace_function<void ()> data_updater_;
};

SystemInfo::SystemInfo(mjlib::micro::Pool& pool,
                       mjlib::micro::TelemetryManager& telemetry)
    : impl_(&pool, pool, telemetry) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::PollMillisecond() {
  impl_->PollMillsecond();
}

void SystemInfo::SetCanResetCount(uint32_t value) {
  impl_->SetCanResetCount(value);
}

uint32_t SystemInfo::millisecond_counter() const {
  return impl_->data_.ms_count;
}

}
