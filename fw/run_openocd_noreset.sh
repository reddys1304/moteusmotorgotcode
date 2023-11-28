#!/bin/bash

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "reset_config none separate" \
    $@


This Bash script is used to run OpenOCD (Open On-Chip Debugger) with a set of specified configuration files and commands for the STM32G4 series microcontroller, using an ST-Link interface. The script also accepts additional command-line arguments.

### Breakdown of the Script:

1. **OpenOCD Command**:
   - `openocd`: This is the command to run OpenOCD, a tool used for programming and debugging microcontrollers.

2. **Configuration Files**:
   - `-f interface/stlink.cfg`: This specifies the configuration file for the ST-Link interface. ST-Link is a debugging and programming tool for STM32 and STM8 microcontrollers.
   - `-f target/stm32g4x.cfg`: This indicates the configuration file for the STM32G4 series microcontroller. It contains specific settings for this family of microcontrollers.

3. **OpenOCD Command**:
   - `-c "reset_config none separate"`: This OpenOCD command configures the reset behavior. `none` means no specific reset type is used, and `separate` indicates that the debug and target resets are handled separately.

4. **Additional Arguments** (`$@`):
   - `$@`: This is a Bash special parameter that represents all the arguments supplied to the script. It allows the user to pass additional OpenOCD commands or parameters when running the script.

### Purpose of the Script:

- The script is a flexible way to start OpenOCD with basic configuration for the STM32G4 series microcontroller and the ST-Link interface. 
- The ability to pass additional arguments (`$@`) makes the script versatile for various debugging or programming tasks, as the user can include any specific OpenOCD commands needed for their particular use case.

### Usage:

To use this script:
1. You need to have OpenOCD and the necessary drivers for the ST-Link interface installed on your system.
2. Connect the ST-Link debugger to your STM32G4 microcontroller.
3. Run the script from a Bash shell, adding any additional OpenOCD commands or parameters as needed.

This script is typically used by embedded software developers and hardware engineers working with STM32 microcontrollers for tasks like debugging, flashing firmware, or accessing specific registers or memory areas of the microcontroller.