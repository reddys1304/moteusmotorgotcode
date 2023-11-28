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
This header file defines the `SystemInfo` class in the `moteus` namespace, which is designed to keep track of various system health and performance metrics for embedded systems, presumably as part of a larger robotic or motor control system. The class is part of the `mjlib` framework, which appears to be a library for microcontroller-based applications.

### Key Components:

1. **Constructor**: `SystemInfo` is constructed with references to `mjlib::micro::Pool` and `mjlib::micro::TelemetryManager`. These dependencies suggest that it uses dynamic memory allocation (`Pool`) and is capable of logging or reporting system metrics (`TelemetryManager`).

2. **Destructors**: The presence of a destructor (`~SystemInfo()`) implies that the class may be managing resources that need explicit cleanup or finalization.

3. **Polling Method**: `PollMillisecond()` is likely intended to be called periodically (every millisecond) to update or check various system metrics. This method's functionality could include tracking system uptime, monitoring resource usage, or other periodic checks.

4. **CAN Bus Reset Counter**: `SetCanResetCount(uint32_t)` suggests that the class tracks resets or errors in a Controller Area Network (CAN) bus, which is commonly used in automotive and industrial control systems for communication between microcontrollers and devices.

5. **Millisecond Counter**: `millisecond_counter()` provides access to a counter that likely tracks the system's uptime or the elapsed time since the last reset, in milliseconds.

6. **Idle Count**: `static volatile uint32_t idle_count` is a static member, indicating that it's shared across all instances of the class. It's marked as `volatile`, suggesting it may be modified by interrupt service routines or other concurrent processes. This counter is likely incremented during CPU idle periods, providing a measure of CPU load or utilization.

### Usage and Implications:

- **Embedded Systems Monitoring**: This class is designed for embedded systems where monitoring performance, resource usage, and system health is crucial, particularly in real-time applications like robotics or motor control.

- **Telemetry and Diagnostics**: Integration with a telemetry manager suggests that `SystemInfo` can be used for diagnostic purposes, logging, and possibly remote monitoring of the system's health.

- **Real-Time Performance**: The millisecond-based polling indicates a focus on real-time performance monitoring, which is essential in systems requiring high reliability and timely responses.

### General Observations:

- The class seems to be part of a larger framework designed for embedded systems, offering utilities for system health monitoring and diagnostics.
- The use of a dynamic memory pool (`mjlib::micro::Pool`) suggests careful management of memory resources, which is typical in microcontroller environments where memory is limited.
- Tracking the idle count can be a simple yet effective way to gauge system load and detect potential performance bottlenecks or issues.
#pragma once

#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

namespace moteus {

/// This class keeps track of things like how many main loops we
/// execute per primary event, and other system health issues like
/// memory usage.
class SystemInfo {
 public:
  SystemInfo(mjlib::micro::Pool&, mjlib::micro::TelemetryManager&);
  ~SystemInfo();

  void PollMillisecond();
  void SetCanResetCount(uint32_t);

  uint32_t millisecond_counter() const;

  // Increment this from an idle thread.
  static volatile uint32_t idle_count;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
