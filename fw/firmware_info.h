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
This header file defines the `FirmwareInfo` class within the `moteus` namespace, a part of a larger project likely focused on embedded systems or robotics. The class is designed to provide information about the firmware running on a device.

### Class Overview:

#### `FirmwareInfo` Class:
- **Purpose**: To store and provide access to firmware-related information like version, model number, and serial number.
- **Constructor**: Accepts a memory pool (`mjlib::micro::Pool`), a telemetry manager (`mjlib::micro::TelemetryManager`), a firmware version, and a model number. These dependencies suggest that the class may be part of a larger system where memory management and telemetry are important concerns.
- **Destructor**: Defined but the implementation details are not provided in the header file.

#### Methods:
- `model_number`: Returns the model number of the device.
- `firmware_version`: Returns the firmware version encoded in a specific format (major.minor.micro).
- `serial_number`: Returns the unique serial number of the device, structured as an array of three `uint32_t` numbers.

#### Inner Struct:
- `SerialNumber`: A structure to hold the serial number, consisting of an array of three `uint32_t` elements.

#### Private Members:
- `impl_`: A pointer to the implementation class `Impl`, following the Pimpl (Pointer to Implementation) idiom. This suggests that the details of the `FirmwareInfo` class are hidden from the user, providing an abstraction layer and potentially easing maintenance and compatibility.

### Key Features:

- **Firmware Identification**: The class provides mechanisms to identify firmware by version and model number, crucial for compatibility checks, updates, and troubleshooting.
- **Unique Device Identification**: Offers a method to retrieve a unique serial number, aiding in device tracking and management.
- **Abstraction and Encapsulation**: Adheres to the Pimpl idiom, hiding implementation details and exposing only the necessary interface to the user. This reduces coupling and can improve compile times.

### Usage:

The `FirmwareInfo` class is likely used in embedded systems or devices where firmware plays a critical role. It can be utilized for:
- Displaying firmware details to the user or system administrator.
- Performing firmware compatibility checks during updates or maintenance.
- Logging and telemetry, where details about the device’s firmware are essential.
- Unique identification of devices in a network or system.

### Conclusion:

The `FirmwareInfo` class is a well-structured component that provides essential information about a device's firmware in an embedded system or similar application. Its design reflects good software engineering practices, offering a clean and easy-to-use interface for accessing firmware-related data while keeping implementation details hidden and separated.
#pragma once

#include <array>

#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

namespace moteus {

/// Holds information about the firmare.
class FirmwareInfo {
 public:
  FirmwareInfo(mjlib::micro::Pool&, mjlib::micro::TelemetryManager&,
               uint32_t version,
               uint32_t model);
  ~FirmwareInfo();

  uint32_t model_number() const;

  /// Return the firmware version encoded as 0x010203 major.minor.micro
  uint32_t firmware_version() const;

  struct SerialNumber {
    std::array<uint32_t, 3> number = {};
  };

  /// Return the unique serial number for this device.
  SerialNumber serial_number() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
