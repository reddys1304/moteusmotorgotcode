#!/bin/bash

# This clears the nSWBOOT0 option byte, which forces the device to
# start executing from user code regardless of the state of the PB8
# pin.
This Bash script is used to modify the option bytes of an STM32G4 series microcontroller using OpenOCD (Open On-Chip Debugger). Specifically, it clears the `nSWBOOT0` option byte, which affects how the device starts execution after a reset. 

### Breakdown of the Script:

1. **OpenOCD Command**:
   - `openocd`: It's the command to run OpenOCD.

2. **Configuration Files**:
   - `-f interface/stlink.cfg`: Specifies the configuration file for the ST-Link interface (the hardware tool used for programming and debugging STM32 devices).
   - `-f target/stm32g4x.cfg`: Indicates the configuration file for the STM32G4 series microcontroller.

3. **OpenOCD Commands**:
   - `-c "init"`: Initializes the OpenOCD server.
   - `-c "reset halt"`: Resets the target and halts it.
   - `-c "stm32l4x option_write 0 0x20 0x00000000 0x04000000"`: Executes a command to write to the option bytes. The `nSWBOOT0` bit is cleared, changing how the microcontroller starts up.
   - `-c "stm32l4x option_load 0"`: Loads the new option byte settings into the microcontroller.
   - `-c "reset"`: Resets the target microcontroller again to apply the new settings.
   - `-c "exit"`: Exits the OpenOCD server.

### Purpose of the Script:

- The primary purpose is to modify the `nSWBOOT0` option byte. In STM32 microcontrollers, this option byte can determine the start of execution (either from system memory or user flash memory) after a reset. 
- By clearing this option byte (`0x00000000`), the script ensures that the device starts executing from user code, regardless of the state of the PB8 pin.
- This operation is commonly done during firmware development or when changing the boot behavior of the microcontroller for specific application requirements.

### Usage:

To use this script, you need to have OpenOCD installed and an ST-Link debugger connected to your STM32G4 microcontroller. The script is run from a bash shell, typically on a Linux or macOS system (it can also be run on Windows using tools like Git Bash). It's generally used by firmware developers or hardware engineers working with STM32 microcontrollers.
openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "stm32l4x option_write 0 0x20 0x00000000 0x04000000" \
    -c "stm32l4x option_load 0" \
    -c "reset" \
    -c "exit"
