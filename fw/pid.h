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
This code defines a PID (Proportional, Integral, Derivative) controller class in C++. The `PID` class is part of the `moteus` namespace and is designed to compute a control output based on the error between a desired setpoint and a measured process variable. This is a common control algorithm used in various applications for process control and robotics.

### Key Components:

1. **Config Structure**: Holds the configuration parameters for the PID controller.
   - `kp`, `ki`, `kd`: Proportional, Integral, and Derivative gains.
   - `iratelimit`: Limits the rate of change of the integral term.
   - `ilimit`: Limits the maximum value of the integral term.
   - `max_desired_rate`: Limits the rate of change of the desired setpoint.
   - `sign`: Direction of the controller (can be positive or negative).

2. **State Structure**: Represents the current state of the PID controller.
   - Keeps track of the integral term, current desired value, error, rate of error, and intermediate calculations like proportional (`p`), derivative (`d`), and their sum (`pd`).
   - Provides a method to clear its state.

3. **PID Constructor**: Takes pointers to `Config` and `State` instances.

4. **Apply Method**: The main method to compute the PID output.
   - Takes measurements, desired values (and their rates), and frequency of updates as inputs.
   - Applies rate limits to the desired values and error calculations.
   - Computes the PID output based on the current error, error rate, and integral term.

5. **ApplyOptions Structure**: Allows scaling of the PID terms (`kp_scale`, `kd_scale`, `ki_scale`).

### Usage:

- To use this class, you need to instantiate a `PID::Config` with the desired PID parameters, a `PID::State` to maintain the controller's state, and then create a `PID` object with these instances.
- The `Apply` method is called repeatedly, providing current measurements and desired values. It returns the control output based on the PID calculations.

### Application:

- This PID controller can be used in a variety of control systems, such as motor control, process control in industrial settings, robotics, and more.
- The inclusion of rate limits and scaling options makes it versatile and suitable for systems where precise control and stability are crucial.
#pragma once

#include <cmath>
#include <limits>

#include "mjlib/base/limit.h"
#include "mjlib/base/visitor.h"

#include "fw/ccm.h"

namespace moteus {

class PID {
 public:
  struct Config {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float iratelimit = -1.0f;
    float ilimit = 0.0f;
    float max_desired_rate = 0.0f;  // 0 is unlimited
    int8_t sign = 1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(kp));
      a->Visit(MJ_NVP(ki));
      a->Visit(MJ_NVP(kd));
      a->Visit(MJ_NVP(iratelimit));
      a->Visit(MJ_NVP(ilimit));
      a->Visit(MJ_NVP(max_desired_rate));
      a->Visit(MJ_NVP(sign));
    }
  };

  struct State {
    float integral = 0.0f;
    // When starting with desired rate limits in place, we by default
    // always accept the first desired command with no limiting (users
    // can of course override this value if they want to start from
    // some predetermined value).
    float desired = std::numeric_limits<float>::quiet_NaN();

    // The following are not actually part of the "state", but are
    // present for purposes of being logged with it.
    float error = 0.0f;
    float error_rate = 0.0f;

    float p = 0.0f;
    float d = 0.0f;
    float pd = 0.0f;
    float command = 0.0f;

    void Clear() MOTEUS_CCM_ATTRIBUTE {
      // We implement this solely for speed, because on at least
      // Cortex-M4, just calling the constructor delegates to memset,
      // which is much slower than memberwise assignment.
      integral = 0.0f;

      desired = std::numeric_limits<float>::quiet_NaN();

      error = 0.0f;
      error_rate = 0.0f;
      p = 0.0f;
      d = 0.0f;
      pd = 0.0f;
      command = 0.0f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(integral));
      a->Visit(MJ_NVP(desired));
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(error_rate));
      a->Visit(MJ_NVP(p));
      a->Visit(MJ_NVP(d));
      a->Visit(MJ_NVP(pd));
      a->Visit(MJ_NVP(command));
    }
  };

  PID(const Config* config, State* state)
      : config_(config), state_(state) {}

  struct ApplyOptions {
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    float ki_scale = 1.0f;

    ApplyOptions() {}
  };

  float Apply(float measured, float input_desired,
              float measured_rate, float input_desired_rate,
              int rate_hz,
              ApplyOptions apply_options = {}) MOTEUS_CCM_ATTRIBUTE {
    float desired = {};
    float desired_rate = {};

    // First apply max_desired_rate
    if (config_->max_desired_rate != 0.0f &&
        std::isfinite(state_->desired)) {
      const float max_step = config_->max_desired_rate / rate_hz;
      const float proposed_step = input_desired - state_->desired;
      const float actual_step =
          mjlib::base::Limit(proposed_step, -max_step, max_step);
      desired = state_->desired + actual_step;
      desired_rate =
          mjlib::base::Limit(input_desired_rate, -config_->max_desired_rate,
                             config_->max_desired_rate);
    } else {
      desired = input_desired;
      desired_rate = input_desired_rate;
    }

    state_->desired = desired;
    state_->error = measured - desired;
    state_->error_rate = measured_rate - desired_rate;

    const float max_i_update = config_->iratelimit / rate_hz;
    float to_update_i = state_->error * config_->ki / rate_hz;
    if (max_i_update > 0.0f) {
      if (to_update_i > max_i_update) {
        to_update_i = max_i_update;
      } else if (to_update_i < -max_i_update) {
        to_update_i = -max_i_update;
      }
    }

    state_->integral += to_update_i;

    if (state_->integral > config_->ilimit) {
      state_->integral = config_->ilimit;
    } else if (state_->integral < -config_->ilimit) {
      state_->integral = -config_->ilimit;
    }

    state_->p = apply_options.kp_scale * config_->kp * state_->error;
    state_->d = apply_options.kd_scale * config_->kd * state_->error_rate;
    state_->pd = state_->p + state_->d;

    state_->command = config_->sign *
        (state_->pd + apply_options.ki_scale * state_->integral);

    return state_->command;
  }

 private:
  const Config* const config_;
  State* const state_;
};

}
