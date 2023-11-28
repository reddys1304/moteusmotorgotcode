#!/usr/bin/python3

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

This Python script calculates and prints out a series of temperature values. It's designed to work with a specific type of temperature sensor, likely a thermistor, which changes its resistance with temperature. The script uses this resistance change to calculate the corresponding temperature in Celsius.

Here's a breakdown of how the script works:

### The `temp` Function
1. **Voltage Calculation (`v`)**: It first converts a raw sensor value (`counts`) into a voltage reading (`v`). This is based on the assumption that the maximum voltage is 3.3V and the ADC (Analog-to-Digital Converter) has a resolution of 4096 steps (12-bit ADC).
   
2. **Constants `B` and `R`**:
   - `B` is the Beta parameter of the thermistor, a constant indicating the thermistor's characteristics. The value 4050.0 is typical for many thermistors.
   - `R` is the resistance value at the reference temperature (usually 25°C) for the thermistor, here assumed to be 10,000 Ohms (10K Ohms).

3. **Resistance Calculation (`r_t`)**: It calculates the current resistance of the thermistor (`r_t`) based on the voltage reading.

4. **Temperature Calculation**: Using the Beta parameter formula, the script calculates the temperature in Kelvin and then converts it to Celsius.

### The `main` Function
- The `main` function iterates over a range of raw ADC values (from 0 to 4095, in steps of 128).
- For each ADC value, it calls the `temp` function to calculate the corresponding temperature and prints it out in a formatted string, showing both the temperature in Celsius and the raw ADC value.

### Usage
This script is useful for generating a lookup table for temperature values based on raw ADC readings from a thermistor. This approach is often used in embedded systems where computational resources are limited, and a quick reference to a pre-calculated table is more efficient than performing complex calculations at runtime.

### Important Note
The script assumes specific thermistor characteristics (Beta value and resistance at 25°C). For a different thermistor, these values would need to be adjusted accordingly. Also, the ADC resolution and maximum voltage are set for a specific system and may need to be adapted for other hardware configurations.
import math

def temp(counts):
    v = 3.3 * max(1, counts) / 4096.0
    B = 4050.0
    R = 10000.0

    r_t = 3.3 * R / v - R
    return 1.0 / (1.0 / (273.15 + 25.0) + (1.0 / B) * math.log(r_t / 47000)) - 273.15


def main():
    for x in range(0, 4096, 128):
        print("  {:.2f}f, // {}".format(temp(x), x))


if __name__ == '__main__':
    main()
