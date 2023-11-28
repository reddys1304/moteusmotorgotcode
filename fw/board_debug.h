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
Purpose: Provides debugging utilities for the controller board.
Key Components:
BoardDebug class: Manages debugging functionalities.
Impl class: Implements the actual debugging logic, defined in board_debug.cc.
Functionality:
LED control, motor stop, raw PWM commands, voltage control, position commands, calibration routines, and more.
Parsing and handling of various debug commands.
Special calibration modes and motor control tests.
Histogram generation for analysis.
Usage: Offers comprehensive debug and testing tools for the motor control system, including interactive command processing.
#pragma once

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"

#include "fw/bldc_servo.h"

namespace moteus {

/// Utilities for bringing up the controller board.
class BoardDebug {
 public:
  BoardDebug(mjlib::micro::Pool*,
             mjlib::micro::CommandManager*,
             mjlib::micro::TelemetryManager*,
             mjlib::multiplex::MicroServer*,
             BldcServo* bldc_servo);
  ~BoardDebug();

  void PollMillisecond();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
