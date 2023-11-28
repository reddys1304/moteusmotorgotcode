#!/usr/bin/python3 -B

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
This Python script is an asynchronous program designed to interact with a motor controller, specifically a `moteus` controller. It utilizes the `asyncio` library for asynchronous programming and `matplotlib` for plotting data. The script's main goal is to configure the motor controller and visualize the results.

### Key Components of the Script:

1. **Import Statements**:
   - `asyncio`: For handling asynchronous programming.
   - `matplotlib.pyplot`: For plotting graphs.
   - `moteus`: A custom module for interfacing with the moteus motor controller.

2. **`main` Function (Asynchronous)**:
   - Initializes a `moteus.Controller` and `moteus.Stream`.
   - Sends a command to the controller to reset the encoder scale (`"d enc-scal 0"`).
   - Waits for 3 seconds (`await asyncio.sleep(3.0)`).
   - Sends another command (`"d enc-ecal"`) to get encoder calibration data and decodes the received data.
   - Parses the received data into two lists: `indices` and `values`.
   - Plots these values using `matplotlib` and displays the plot.

3. **Executing the Main Function**:
   - The script uses `asyncio.run(main())` to run the asynchronous `main` function.

### How It Works:

1. The script starts by creating a `moteus.Controller` object, which is likely an interface to communicate with a moteus motor controller.

2. It then initializes a `moteus.Stream` object, possibly for data streaming from the controller.

3. The script sends a command to reset the encoder scale to zero. This step might be a part of the initialization or calibration process.

4. The script pauses for 3 seconds to allow for the command to take effect or for the motor to reach a stable state.

5. It then requests encoder calibration data from the controller, which is received as a string, decoded into UTF-8 format, and split into lines.

6. Each line of data is further split into pairs of values, which are then separated into two lists: one for indices (or positions) and one for corresponding values.

7. Finally, it plots these values against the indices using `matplotlib` and displays this plot.

### Usage:

This script can be used to calibrate and visualize the performance of a motor controlled by a moteus controller. It's useful in settings where precise motor control and feedback are necessary, such as in robotics or industrial automation. The plot can help in understanding the behavior of the encoder and motor at different positions or states.
import asyncio
import matplotlib.pyplot as plt
import moteus

async def main():
    c = moteus.Controller()
    s = moteus.Stream(s)

    await s.command(b"d enc-scal 0")
    await asyncio.sleep(3.0)
    results = (await s.command(b"d enc-ecal")).decode('utf8')
    data = [[float(a) for a in x.strip().split(' ')] for x in results.split('\n')]
    indices = [x[0] for x in data]
    values = [x[1] for x in data]
    plt.plot(indices, values)
    plt.show()


if __name__ == '__main__':
    asyncio.run(main())
