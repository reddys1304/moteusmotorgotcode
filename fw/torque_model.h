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
This C++ header file defines the `TorqueModel` class within the `moteus` namespace. The class is designed to model the relationship between torque and phase current for an electric motor, providing functions to convert between these two quantities. This is essential in motor control applications where precise control of torque is required.

### Implementation Details:

1. **Constructor**: The `TorqueModel` constructor initializes the model with parameters specific to the motor's electrical and mechanical characteristics. These parameters include:
   - `torque_constant`: A factor that relates motor current to torque.
   - `current_cutoff_A`: The threshold current above which the motor's behavior may change (possibly entering a saturation regime).
   - `current_scale`: A scaling factor used in the conversion calculations.
   - `torque_scale`: Another scaling factor used in the conversion calculations.

2. **Current to Torque Conversion (`current_to_torque`)**: This method converts a given current to the corresponding torque. It uses logarithmic and linear scaling to model the motor's behavior, considering both linear and rotation (possibly saturation) regimes.

3. **Torque to Current Conversion (`torque_to_current`)**: This method does the inverse, converting a given torque value back to the corresponding current. Similar to the `current_to_torque` method, it handles different regimes of motor operation.

4. **Member Variables**: The class stores the motor parameters (`torque_constant_`, `current_cutoff_A_`, `current_scale_`, `torque_scale_`) as constants, indicating these are fixed characteristics of the motor and do not change during operation.

### Key Points:

- **Performance Consideration**: Both conversion methods are marked with `__attribute__((always_inline))`, suggesting a need for these methods to be inlined for performance reasons, likely due to their use in time-critical motor control loops.

- **Approximations**: The methods use `log2f_approx` and `pow2f_approx` for logarithmic and exponential operations, indicating the use of approximate calculations for efficiency.

- **Handling Motor Characteristics**: The model seems to account for both linear and non-linear behavior (possible saturation) of the motor, which is crucial for accurate motor control in different operational regimes.

### Usage:

- The `TorqueModel` class is likely used in the context of a motor control system, such as in robotic actuators or electric vehicle drivetrains, where precise control of motor torque is essential.
- The parameters for the model would typically be derived from the motor's specifications or through empirical testing.

### Considerations:

- The accuracy of the model highly depends on the correctness of the input parameters, which should be carefully determined for the specific motor in use.
- The use of approximations for mathematical operations is a trade-off between computational speed and accuracy, which is common in real-time control systems.
#pragma once

#include <algorithm>

#include "fw/math.h"

namespace moteus {

/// Provides facilities for converting to and from torque and phase
/// current for a given motor.
class TorqueModel {
 public:
  TorqueModel(float torque_constant,
              float current_cutoff_A,
              float current_scale,
              float torque_scale)
      : torque_constant_(torque_constant),
        current_cutoff_A_(current_cutoff_A),
        current_scale_(current_scale),
        torque_scale_(torque_scale) {}

  float current_to_torque(float current) const __attribute__((always_inline)) {
    // We always calculate the "rotation" region cutoff so as to have
    // a somewhat constant calculation time.
    const float rotation_extra =
        torque_scale_ *
        log2f_approx(1.0f + std::max(
                         0.0f, (std::abs(current) - current_cutoff_A_)) *
                     current_scale_);
    return ((std::abs(current) < current_cutoff_A_) ?
            // rotation_extra will always be 0 here, but we add it in
            // anyways to force the evaluation of the above code no
            // matter what.  Thus our loop timing will be relatively
            // constant even when we go into the rotation regime.
            (current * torque_constant_) :
            // In this case, rotation_extra should be non-zero.
            std::copysignf(current_cutoff_A_ * torque_constant_ +
                           rotation_extra,
                           current));
  }

  float torque_to_current(float torque) const __attribute__((always_inline)) {
    const float a = (std::abs(torque) - current_cutoff_A_ * torque_constant_) /
                    torque_scale_;
    const float rotation_extra = (pow2f_approx(a) - 1.0f) / current_scale_;

    const float cutoff_torque =
        current_cutoff_A_ * torque_constant_;
    return (std::abs(torque) < cutoff_torque) ?
        (torque / torque_constant_) :
        std::copysign(current_cutoff_A_ + rotation_extra,
                      torque);
  }

  const float torque_constant_;
  const float current_cutoff_A_;
  const float current_scale_;
  const float torque_scale_;
};

}
