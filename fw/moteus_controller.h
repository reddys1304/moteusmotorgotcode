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
This C++ header file defines the `MoteusController` class, a key component of the moteus motor controller's firmware. The moteus motor controller is a sophisticated system used for controlling brushless DC (BLDC) motors. Let's break down the primary elements of this class:

### Class Definition: `MoteusController`
- **Purpose**: This class serves as the central hub that ties together various subsystems necessary for controlling a BLDC motor. It acts as a glue layer, integrating various components into a cohesive controller.

### Constructor Parameters
- The constructor takes multiple parameters, each representing a different subsystem or utility of the motor controller:
  1. **mjlib::micro::Pool***: Likely used for memory management, possibly providing a pool allocator.
  2. **mjlib::micro::PersistentConfig***: Handles configuration management, possibly storing and retrieving settings that need to persist across reboots.
  3. **mjlib::micro::CommandManager***: Manages command parsing and execution, which is crucial for interpreting commands sent to the motor controller.
  4. **mjlib::micro::TelemetryManager***: Manages telemetry data, useful for monitoring, diagnostics, and logging.
  5. **mjlib::multiplex::MicroServer***: Manages communication with external systems or devices, possibly through a protocol like CAN or UART.
  6. **ClockManager***: Manages time-related functions, possibly providing time stamps for logging or scheduling tasks.
  7. **SystemInfo***: Provides information about the system, like firmware version, hardware diagnostics, etc.
  8. **MillisecondTimer***: A utility for timing-related functions, likely used for timing motor control loops or measuring time intervals.
  9. **FirmwareInfo***: Contains information about the firmware, such as version numbers, which could be used for updates or compatibility checks.

### Core Functionalities
- **Start, Poll, PollMillisecond**: These functions are likely used to start the controller, handle periodic tasks, and manage time-sensitive operations, respectively. `Poll` and `PollMillisecond` might be called in a main loop or interrupt service routine to handle regular updates and time-critical tasks.

- **bldc_servo()**: Provides access to the BLDC servo control subsystem. This is key for performing actual motor control tasks like setting speed, position, and applying control algorithms.

- **multiplex_server()**: Exposes the communication interface, allowing the motor controller to interact with external systems, receive commands, and send data.

### Design Notes
- The use of `mjlib` and `multiplex` namespaces suggests the use of a modular library designed for embedded systems, focusing on motor control and communication.
- The class employs a `PoolPtr` for its implementation (`impl_`), indicating the use of the Pimpl (Pointer to Implementation) idiom. This is a common technique in C++ to hide implementation details and reduce compilation dependencies, leading to cleaner interfaces and potentially faster compile times.
- The overall structure of the class suggests a well-organized and modular approach to motor control, with clear separation of concerns and a focus on maintainability and scalability.

### Conclusion
The `MoteusController` class is a central piece in the moteus motor controller firmware, integrating various subsystems necessary for BLDC motor control, including memory management, communication, telemetry, and timing. The use of modern C++ practices and a modular approach indicates a sophisticated and well-designed system.
#pragma once

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/multiplex/micro_server.h"

#include "fw/bldc_servo.h"
#include "fw/clock_manager.h"
#include "fw/firmware_info.h"
#include "fw/millisecond_timer.h"
#include "fw/system_info.h"

namespace moteus {

/// Glues together the various pieces of hardware that make a moteus
/// controller board.
class MoteusController {
 public:
  MoteusController(mjlib::micro::Pool*,
                   mjlib::micro::PersistentConfig* config,
                   mjlib::micro::CommandManager* command_manager,
                   mjlib::micro::TelemetryManager* telemetry_manager,
                   mjlib::multiplex::MicroServer* multiplex_protocol,
                   ClockManager*,
                   SystemInfo*,
                   MillisecondTimer*,
                   FirmwareInfo*);
  ~MoteusController();

  void Start();
  void Poll();
  void PollMillisecond();

  BldcServo* bldc_servo();

  mjlib::multiplex::MicroServer::Server* multiplex_server();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};
}
