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
This C++ source file defines the `Uuid` class within the `moteus` namespace. The class is responsible for handling a Universally Unique Identifier (UUID), and it interacts with a persistent configuration system for storage.

### Implementation Details:

1. **Global UUID from OTP Memory**: The `g_otp_uuid` pointer is defined as a constant and points to a specific memory location (`0x1fff7000`). This location is typically reserved for One-Time Programmable (OTP) memory in certain microcontrollers, suggesting that the UUID may be read from a pre-programmed area in the hardware.

2. **Constructor**: The `Uuid` constructor takes a reference to a `mjlib::micro::PersistentConfig` object. It registers the `data_` member (which holds the UUID) with the configuration system and calls the `Update` method to initialize the UUID.

3. **Update Method**: The `Update` method checks the validity of the UUID stored in the OTP memory. It considers the UUID valid if any byte in the OTP UUID is not `0xff`. If valid, it copies the UUID from the OTP memory into the `data_.uuid` array.

### Key Points:

- **Use of OTP Memory**: The use of OTP memory for storing the UUID suggests that the UUID is pre-programmed into the hardware and is intended to be unique and unchangeable for each device. This is common in manufacturing processes where each device needs a unique identifier.

- **Persistent Configuration Integration**: The UUID is integrated with a persistent configuration system, ensuring that it remains consistent across power cycles and system resets.

- **Conditional Update**: The UUID is updated from OTP memory only if it is valid (not all bytes are `0xff`). This indicates a fail-safe mechanism to avoid overwriting the UUID with invalid data.

### Usage:

- The `Uuid` class is likely used in systems where a hardware-based, unchangeable unique identifier is required. This can be crucial for device authentication, tracking, and unique configuration management.

### Considerations:

- The actual source of the UUID (the OTP memory location) is hardware-specific, indicating that this code is tailored to a specific type of microcontroller or system.
- The decision to not update the UUID if it is all `0xff` might be a safeguard against uninitialized memory or manufacturing errors.
#include "fw/uuid.h"

namespace moteus {

const uint8_t* const g_otp_uuid = reinterpret_cast<const uint8_t*>(0x1fff7000);

Uuid::Uuid(mjlib::micro::PersistentConfig& config) {
  config.Register("uuid", &data_, [this]() { this->Update(); });

  Update();
}

void Uuid::Update() {
  // If any of the UUID bytes are non- 0xff, then just force our UUID
  // to be that from OTP.
  const bool otp_uuid_valid = [&]() {
    for (size_t i = 0; i < data_.uuid.size(); i++) {
      if (g_otp_uuid[i] != 0xff) { return true; }
    }
    return false;
  }();

  if (!otp_uuid_valid) { return; }

  for (size_t i = 0; i < data_.uuid.size(); i++) {
    data_.uuid[i] = g_otp_uuid[i];
  }
}



}
