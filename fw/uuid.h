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
This C++ header file declares the `Uuid` class within the `moteus` namespace. The class is designed to manage a Universally Unique Identifier (UUID), often used for ensuring unique identification in various systems.

### Key Components and Functionality

- **Persistent Configuration Dependency**: The `Uuid` class constructor takes a reference to an instance of `mjlib::micro::PersistentConfig`. This suggests that the UUID is intended to be stored persistently, probably in non-volatile memory, to ensure its uniqueness across system reboots.

- **Data Structure**: The `Data` nested class is a structure that holds the UUID. It consists of a 16-byte (`std::array<uint8_t, 16>`) array, which aligns with the standard UUID size of 128 bits.

- **Serialization Support**: The `Data` structure includes a `Serialize` method, templated for a generic `Archive`. This method uses the `MJ_NVP` macro from the `mjlib` library for serialization purposes, indicating that the UUID can be easily serialized and deserialized, possibly for storage or transmission.

- **Private `Update` Method**: The `Update` method is declared privately within the `Uuid` class, suggesting that it's used internally by the class to update or generate the UUID. The implementation details of how the UUID is generated or updated are not visible in this header file.

### Usage

The `Uuid` class is likely used in scenarios where a unique identifier is required, such as:

- Differentiating between multiple instances of devices or components in a system.
- Ensuring unique identification for data logging or tracking purposes.
- Any application where a persistent, non-repeating identifier is necessary.

### Design Considerations

- The UUID is likely intended to be generated once and stored persistently to maintain its uniqueness.
- The use of a persistent configuration mechanism suggests that the UUID might be set during manufacturing or initial setup and then retained throughout the device's lifecycle.
- The serialization functionality implies that the UUID might be communicated or stored in various formats, adapting to different use cases.

### Limitations

- The header file does not reveal how the UUID is generated or whether it conforms to any specific version or variant of UUID standards (e.g., UUIDv4 for random UUIDs).
- The actual implementation details of how the UUID is used within the broader system are not provided in this header.
#pragma once

#include <array>
#include <cstdint>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"

namespace moteus {

class Uuid {
 public:
  Uuid(mjlib::micro::PersistentConfig& config);

  struct Data {
    std::array<uint8_t, 16> uuid = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(uuid));
    }
  };

 private:
  void Update();

  Data data_;
};

}
