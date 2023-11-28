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
#include "fw/error.h"

namespace moteus {

namespace {
struct MoteusErrorCategory : mjlib::micro::error_category {
  const char* name() const noexcept override { return "moteus"; }
  std::string_view message(int condition) const override {
    switch (static_cast<errc>(condition)) {
      case errc::kSuccess: return "success";
      case errc::kDmaStreamTransferError: return "dma stream transfer error";
      case errc::kDmaStreamFifoError: return "dma stream fifo error";
      case errc::kUartOverrunError: return "uart overrun error";
      case errc::kUartFramingError: return "uart framing error";
      case errc::kUartNoiseError: return "uart noise error";
      case errc::kUartBufferOverrunError: return "uart buffer overrun";
      case errc::kUartParityError: return "uart parity error";
      case errc::kCalibrationFault: return "calibration fault";
      case errc::kMotorDriverFault: return "motor driver fault";
      case errc::kOverVoltage: return "over voltage";
      case errc::kEncoderFault: return "encoder fault";
      case errc::kMotorNotConfigured: return "motor not configured";
      case errc::kPwmCycleOverrun: return "pwm cycle overrun";
      case errc::kOverTemperature: return "over temperature";
      case errc::kStartOutsideLimit: return "start outside limit";
      case errc::kUnderVoltage: return "under voltage";
      case errc::kConfigChanged: return "config changed";
      case errc::kThetaInvalid: return "theta invalid";
      case errc::kPositionInvalid: return "position invalid";
      case errc::kDriverEnableFault: return "driver enable";
      case errc::kStopPositionDeprecated: return "stop position deprecated";
      case errc::kTimingViolation: return "timing violation";
    }
    return "unknown";
  }
};

const mjlib::micro::error_category& moteus_error_category() {
  static MoteusErrorCategory result;
  return result;
}
}

mjlib::micro::error_code make_error_code(errc err) {
  return mjlib::micro::error_code(
      static_cast<int>(err), moteus_error_category());
}

}
