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
This source file implements the `FirmwareInfo` class defined in the corresponding header file. The class is part of the `moteus` namespace, likely intended for use in embedded systems or robotics applications. It provides methods to access firmware-related information, such as version, serial number, and hardware model.

### Class and Methods Overview:

#### `FirmwareInfo::Impl` Class (Implementation Details):
- **Purpose**: Encapsulates the implementation details of the `FirmwareInfo` class.
- **Data Members**: 
  - `Info info_`: A struct holding firmware version, model number, serial number, hardware family, and hardware revision.
- **Constructor**: Initializes the `Info` struct with firmware version, model, and serial number. It also registers this information for telemetry.

#### `FirmwareInfo` Class:
- **Constructor**: Initializes the `Impl` class with firmware version and model number.
- **Destructor**: Defined but the implementation details are not shown.
- **Methods**:
  - `firmware_version`: Returns the firmware version.
  - `serial_number`: Returns the device's serial number.

#### `Info` Struct:
- **Members**:
  - `uint32_t version`: Firmware version.
  - `std::array<uint32_t, 3> serial_number`: An array to store the serial number.
  - `uint32_t model`: Model number.
  - `uint8_t family`: Hardware family identifier.
  - `uint8_t hwrev`: Hardware revision.
- **Serialize Method**: Serializes the data members for telemetry or other purposes.

### Implementation Details:

1. **Telemetry Registration**: The constructor of `Impl` class registers the firmware information (`Info` struct) with the telemetry system, indicating that this information could be monitored or logged remotely.

2. **Serial Number Extraction**: The constructor fetches the device's unique serial number from a specific memory address. This address (`0x1fff7590`) is likely specific to the STM32G4 microcontroller family, as indicated by the preprocessor directive.

3. **Hardware Version and Model**: The firmware version and model number are set in the constructor and stored in the `Info` struct.

4. **GCC Diagnostic Directives**: The code uses GCC diagnostic directives to suppress specific warnings, potentially related to string operations.

### Usage:

- **Firmware Identification**: Allows for identifying the firmware running on the device, which is useful for compatibility checks, updates, and debugging.
- **Device Identification**: Provides a unique serial number for each device, which can be critical in networked or multi-device environments.
- **Telemetry and Monitoring**: The integration with a telemetry system suggests that this information could be important for remote monitoring and diagnostics.

### Conclusion:

The `FirmwareInfo` class in the `moteus` namespace is a well-structured component for providing essential information about a device's firmware. It is designed to work within an embedded system, with a focus on telemetry and device identification. The use of a separate implementation class (`Impl`) follows the Pimpl idiom, encapsulating the details and providing a clean interface to the rest of the system.
#include "fw/firmware_info.h"

#include "fw/measured_hw_rev.h"
#include "fw/moteus_hw.h"

namespace moteus {

namespace {
struct Info {
  uint32_t version = 0;
  std::array<uint32_t, 3> serial_number = {};
  uint32_t model = 0;
  uint8_t family = g_measured_hw_family;
  uint8_t hwrev = g_measured_hw_rev;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(version));
    a->Visit(MJ_NVP(serial_number));
    a->Visit(MJ_NVP(model));
    a->Visit(MJ_NVP(family));
    a->Visit(MJ_NVP(hwrev));
  }
};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overread"

class FirmwareInfo::Impl {
 public:
  Impl(mjlib::micro::TelemetryManager& telemetry,
       uint32_t version,
       uint32_t model) {
    info_.version = version;
    info_.model = model;

    const int32_t* const device_signature =
        reinterpret_cast<const int32_t*>(
#if defined(TARGET_STM32G4)
            0x1fff7590
#else
#error "Unknown target"
#endif
                                         );

    std::memcpy(&info_.serial_number[0], device_signature,
                sizeof(uint32_t) * 3);
    telemetry.Register("firmware", &info_);
  }

  Info info_;
};

FirmwareInfo::FirmwareInfo(mjlib::micro::Pool& pool,
                           mjlib::micro::TelemetryManager& telemetry,
                           uint32_t version,
                           uint32_t model)
    : impl_(&pool, telemetry, version, model) {}

FirmwareInfo::~FirmwareInfo() {}

uint32_t FirmwareInfo::firmware_version() const {
  return impl_->info_.version;
}

FirmwareInfo::SerialNumber FirmwareInfo::serial_number() const {
  SerialNumber result;
  for (int i = 0; i < 3; i++) {
    result.number[i] = impl_->info_.serial_number[i];
  }
  return result;
}

#pragma GCC diagnostic pop

}
