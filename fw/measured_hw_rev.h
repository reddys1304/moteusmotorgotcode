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
This header defines global variables to identify the hardware family and revision for the servo control system. These variables (g_measured_hw_family and g_measured_hw_rev) are marked extern, meaning they are defined elsewhere but accessible globally.

#pragma once

namespace moteus {

// The current board family.
extern volatile uint8_t g_measured_hw_family;

// The current board revision.
extern volatile uint8_t g_measured_hw_rev;

}
