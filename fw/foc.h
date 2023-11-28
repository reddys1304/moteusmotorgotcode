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
The code you provided is part of a C++ library, presumably for a robotics project, as indicated by the namespace `moteus`. It deals with various mathematical transformations commonly used in control systems, specifically those related to motor control in robotics. Let's break down the key components:

### SinCos Structure
- `SinCos`: A simple structure to store the sine (`s`) and cosine (`c`) of a given angle.

### Cordic Class
- `Cordic`: This class is used for computing the sine and cosine of an angle using the CORDIC algorithm, a simple and efficient algorithm to calculate trigonometric functions.
- It is specifically optimized for the STM32G4 microcontroller, as indicated by the `TARGET_STM32G4` preprocessor directive.
- The `operator()` method computes the sine and cosine of a given angle (in a 32-bit Q format), returning a `SinCos` structure.

### DqTransform Structure
- `DqTransform`: Implements the Direct-Quadrature-Zero (DQZ) transformation, a key component in field-oriented control of AC motors.
- It transforms three-phase quantities (a, b, c) into a rotating reference frame (d, q).

### InverseDqTransform Structure
- `InverseDqTransform`: The inverse of the DQZ transformation, converting from the rotating reference frame back to the three-phase quantities.

### ClarkTransform Structure
- `ClarkTransform`: Implements the Clarke transformation, another transformation used in control of AC motors, converting three-phase quantities into two-phase quantities.

### InverseClarkTransform Structure
- `InverseClarkTransform`: The inverse of the Clarke transformation, converting two-phase quantities back into three-phase quantities.

### ParkTransform Structure
- `ParkTransform`: Implements the Park transformation, similar to the DQZ transformation, used for converting quantities from a stationary reference frame to a rotating reference frame.

### InverseParkTransform Structure
- `InverseParkTransform`: The inverse of the Park transformation.

### Usage
These transformations are crucial in advanced motor control algorithms, especially in robotics and electric vehicle applications. They allow for the decoupling of torque and flux components in AC motors, enabling more precise control over the motor's operation.

### Conclusion
The library is a clear example of the application of mathematical concepts in robotics and control systems. The use of such transformations is essential for efficient and responsive motor control in complex systems such as robots or electric vehicles.
#pragma once

#include <algorithm>
#include <cmath>

#include "fw/math.h"

#ifdef TARGET_STM32G4
#include "stm32g4xx_ll_cordic.h"
#endif

namespace moteus {

// A helper to cache the result of sin and cos on a given quantity.
struct SinCos {
  float s;
  float c;
};

class Cordic {
 public:
#ifdef TARGET_STM32G4
  Cordic() {
    __HAL_RCC_CORDIC_CLK_ENABLE();
    LL_CORDIC_Config(
        CORDIC,
        LL_CORDIC_FUNCTION_COSINE,
        LL_CORDIC_PRECISION_5CYCLES,
        LL_CORDIC_SCALE_0,
        LL_CORDIC_NBWRITE_1,
        LL_CORDIC_NBREAD_2,
        LL_CORDIC_INSIZE_32BITS,
        LL_CORDIC_OUTSIZE_32BITS);
  }

  SinCos operator()(int32_t theta_q31) const {
    LL_CORDIC_WriteData(CORDIC, theta_q31);
    SinCos result;
    result.c = from_q31(LL_CORDIC_ReadData(CORDIC));
    result.s = from_q31(LL_CORDIC_ReadData(CORDIC));
    return result;
  };

  static float from_q31(uint32_t val) {
    return static_cast<float>(static_cast<int32_t>(val)) * (1.0f / 2147483648.0f);
  }
#else
  Cordic() {}

  SinCos operator()(int32_t theta_q31) const {
    SinCos result;
    result.s = std::sin(static_cast<float>(theta_q31) * kPi * (1.0f / 2147483648.0f));
    result.c = std::cos(static_cast<float>(theta_q31) * kPi * (1.0f / 2147483648.0f));
    return result;
  }

  SinCos radians(float theta) const {
    return (*this)(RadiansToQ31(theta));
  }
#endif
};


struct DqTransform {
  DqTransform(const SinCos& sc, float a, float b, float c)
      : d((2.0f / 3.0f) *
          (a * sc.c +
           (kSqrt3_4 * sc.s - 0.5f * sc.c) * b +
           (-kSqrt3_4 * sc.s - 0.5f * sc.c) * c)),
        q((2.0f / 3.0f) *
          (-sc.s * a -
           (-kSqrt3_4 * sc.c - 0.5f * sc.s) * b -
           (kSqrt3_4 * sc.c - 0.5f * sc.s) * c)) {}

  const float d;
  const float q;
};

struct InverseDqTransform {
  InverseDqTransform(const SinCos& sc, float d, float q)
      : a(sc.c * d - sc.s * q),
        b((kSqrt3_4 * sc.s - 0.5f * sc.c) * d - (-kSqrt3_4 * sc.c - 0.5f * sc.s) * q),
        c((-kSqrt3_4 * sc.s - 0.5f * sc.c) * d - (kSqrt3_4 * sc.c - 0.5f * sc.s) * q) {}

  const float a;
  const float b;
  const float c;
};

struct ClarkTransform {
  ClarkTransform(float a, float b, float c)
      : x((2.0f * a - b  - c) * (1.0f / 3.0f)),
        y((b - c) * (1.0f / kSqrt3)) {}

  const float x;
  const float y;
};

struct InverseClarkTransform {
  InverseClarkTransform(float x, float y)
      : a(x),
        b((-x + kSqrt3 * y) / 2.0f),
        c((-x - kSqrt3 * y) / 2.0f) {}

  const float a;
  const float b;
  const float c;
};

struct ParkTransform {
  ParkTransform(const SinCos& sc, float x, float y)
      : d(sc.c * x + sc.s * y),
        q(sc.c * y - sc.s * x) {}

  const float d;
  const float q;
};

struct InverseParkTransform {
  InverseParkTransform(const SinCos& sc, float d, float q)
      : x(sc.c * d - sc.s * q),
        y(sc.c * q + sc.s * d) {}

  const float x;
  const float y;
};

}
