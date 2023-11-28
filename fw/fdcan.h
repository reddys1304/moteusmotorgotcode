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
This header file is part of the `moteus` project, and it defines a C++ class `FDCan` for interfacing with an FDCAN (Flexible Data-rate Controller Area Network) peripheral, commonly used in automotive and industrial applications for high-speed communication.

### Class Overview:

#### `FDCan` Class:
- **Purpose**: Provides functionalities to control and interact with an FDCAN peripheral.
- **Constructor**: Accepts an `Options` struct to configure the FDCAN settings.
- **Methods**:
  - `ConfigureFilters`: Configures message filters.
  - `Send`: Sends a message over the FDCAN network.
  - `Poll`: Checks for and retrieves received messages.
  - `RecoverBusOff`: Handles bus-off conditions.
  - `status`: Retrieves the status of the FDCAN peripheral.
  - `config`: Returns the configuration of the FDCAN peripheral.
  - `ParseDlc`: Parses the Data Length Code (DLC) to determine the size of the data.

#### Enumerations and Structs:

1. **`FilterAction`**: Defines actions to be taken for messages based on filters (Disable, Accept, Reject).
2. **`FilterMode`**: Describes the mode of a filter (Range, Dual, Mask).
3. **`FilterType`**: Distinguishes between standard and extended message identifiers.
4. **`Filter`**: Struct to define a single filter's properties.
5. **`Rate`**: Contains parameters for timing configurations.
6. **`FilterConfig`**: Struct for global filter configurations and an array of individual filters.
7. **`Options`**: Configurable options for the FDCAN peripheral, such as bit rates, pin names, and filter configurations.
8. **`SendOptions`**: Options for sending messages, like bitrate switch and frame format.
9. **`Config`**: Contains the current configuration of the FDCAN peripheral, including clock and rate information.

### Key Features:

- **Flexibility**: The class provides extensive configuration options, allowing detailed control over the FDCAN peripheral's behavior.
- **Filter Management**: Supports configuring both individual and global filters to control which messages are processed or ignored.
- **Advanced Communication Options**: Supports features like automatic retransmission, remote frame, FDCAN frame format, and bitrate switching.
- **Error Handling**: Includes capabilities to recover from bus-off conditions, a critical state in CAN communication where a node becomes disconnected from the network.

### Usage:

The `FDCan` class is designed to be used in embedded systems where reliable, high-speed communication is crucial, such as in automotive control units or industrial automation systems. The class abstracts the complexities of configuring and managing an FDCAN peripheral, making it easier to integrate into a larger system.

### Conclusion:

Overall, the `FDCan` class is a comprehensive and versatile implementation for handling FDCAN communication in complex embedded systems. It exhibits well-structured and clear abstraction over the underlying hardware, which is essential for robust and maintainable code in critical applications like automotive systems.
#pragma once

#include <string_view>

#include "mbed.h"

#include "mjlib/base/string_span.h"

namespace moteus {

class FDCan {
 public:
  enum class FilterAction {
    kDisable,
    kAccept,
    kReject,
  };

  enum class FilterMode {
    kRange,
    kDual,
    kMask,
  };

  enum class FilterType {
    kStandard,
    kExtended,
  };

  struct Filter {
    uint32_t id1 = 0;
    uint32_t id2 = 0;

    FilterMode mode = FilterMode::kRange;
    FilterAction action = FilterAction::kDisable;
    FilterType type = FilterType::kStandard;
  };

  struct Rate {
    int prescaler = -1;
    int sync_jump_width = -1;
    int time_seg1 = -1;
    int time_seg2 = -1;
  };

  struct FilterConfig {
    FilterAction global_std_action = FilterAction::kAccept;
    FilterAction global_ext_action = FilterAction::kAccept;
    FilterAction global_remote_std_action = FilterAction::kAccept;
    FilterAction global_remote_ext_action = FilterAction::kAccept;

    const Filter* begin = nullptr;
    const Filter* end = nullptr;
  };

  struct Options {
    PinName td = NC;
    PinName rd = NC;
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;

    FilterConfig filters;

    bool automatic_retransmission = false;
    bool remote_frame = false;
    bool fdcan_frame = false;
    bool bitrate_switch = false;
    bool restricted_mode = false;
    bool bus_monitor = false;

    bool delay_compensation = false;
    uint32_t tdc_offset = 0;
    uint32_t tdc_filter = 0;

    // If any members of this are non-negative, force them to be used
    // instead of the auto-calculated values.
    Rate rate_override;
    Rate fdrate_override;

    Options() {}
  };

  FDCan(const Options& options = Options());

  enum class Override {
    kDefault,
    kRequire,
    kDisable,
  };

  struct SendOptions {
    Override bitrate_switch = Override::kDefault;
    Override fdcan_frame = Override::kDefault;
    Override remote_frame = Override::kDefault;
    Override extended_id = Override::kDefault;

    SendOptions() {}
  };

  void ConfigureFilters(const FilterConfig&);

  void Send(uint32_t dest_id,
            std::string_view data,
            const SendOptions& = SendOptions());

  /// @return true if a packet was available.
  bool Poll(FDCAN_RxHeaderTypeDef* header, mjlib::base::string_span);

  void RecoverBusOff();

  FDCAN_ProtocolStatusTypeDef status();

  struct Config {
    int clock = 0;
    Rate nominal;
    Rate data;
  };

  Config config() const;

  static int ParseDlc(uint32_t dlc_code);

 private:
  void Init();

  Options options_;
  Config config_;

  FDCAN_GlobalTypeDef* can_ = nullptr;
  FDCAN_HandleTypeDef hfdcan1_;
  FDCAN_ProtocolStatusTypeDef status_result_ = {};
  uint32_t last_tx_request_ = 0;
};

}
