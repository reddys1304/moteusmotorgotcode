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
err.h
This header file defines a custom error code enumeration and integrates it with the standard C++ error handling mechanisms.

Namespaces and Includes: It includes necessary headers and declares usage of the moteus and mjlib namespaces.
Enum errc: Defines an enumeration errc which lists various error codes specific to the system. This includes errors related to DMA stream, UART communication, motor driver faults, and other system-specific issues.
Function make_error_code: A factory function to create error_code objects from errc enumeration values.
Integration with C++ Error Handling:
Specializes the mjlib::micro::is_error_code_enum for the errc enumeration, enabling integration with the standard <system_error> facility in C++.
Provides an empty map for IsEnum specialization. This is likely used for serialization or introspection elsewhere in the project.
error.cc
This source file implements the custom error category and the factory function declared in err.h.

MoteusErrorCategory Class: A custom error category derived from mjlib::micro::error_category. It provides a name ("moteus") and a custom message for each error condition defined in errc.
Function Implementation:
moteus_error_category() returns a static instance of MoteusErrorCategory. This function is used to create consistent error categories across different parts of the program.
make_error_code(errc err) creates an error_code object with the given errc value and the custom MoteusErrorCategory. This function is how errors are created and reported within the system.
Usage and Context
The errc enumeration and the associated error handling infrastructure are specifically tailored for the moteus system. They likely correspond to various fault conditions that can occur in a motor control system or similar embedded hardware.
These files demonstrate a structured approach to error handling in a complex system. By defining a custom error category and enumeration, the system can generate and handle errors more effectively, improving debuggability and reliability.
Integration with the C++ standard error handling allows these custom errors to be used seamlessly with other parts of the C++ standard library, making the code more consistent and robust.
Overall Impression
The approach taken in these files indicates a well-organized and systematic way of handling errors in an embedded or robotics system. By defining specific error codes and integrating them with C++'s error handling mechanisms, the system becomes more maintainable and easier to debug.
#pragma once

#include <array>
#include <type_traits>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/error_code.h"

namespace moteus {

enum class errc {
  kSuccess = 0,

  kDmaStreamTransferError = 1,
  kDmaStreamFifoError = 2,
  kUartOverrunError = 3,
  kUartFramingError = 4,
  kUartNoiseError = 5,
  kUartBufferOverrunError = 6,
  kUartParityError = 7,

  kCalibrationFault = 32,
  kMotorDriverFault = 33,
  kOverVoltage = 34,
  kEncoderFault = 35,
  kMotorNotConfigured = 36,
  kPwmCycleOverrun = 37,
  kOverTemperature = 38,
  kStartOutsideLimit = 39,
  kUnderVoltage = 40,
  kConfigChanged = 41,
  kThetaInvalid = 42,
  kPositionInvalid = 43,
  kDriverEnableFault = 44,
  kStopPositionDeprecated = 45,
  kTimingViolation = 46,
};

mjlib::micro::error_code make_error_code(errc);
}

namespace mjlib {
namespace base {
template <>
struct IsEnum<moteus::errc> {
  static constexpr bool value = true;

  static std::array<std::pair<moteus::errc, const char*>, 0> map() {
    return {{}};
  }
};
}
namespace micro {

template <>
struct is_error_code_enum<moteus::errc> : std::true_type {};

}
}
