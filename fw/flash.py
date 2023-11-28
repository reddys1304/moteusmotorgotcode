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
This Python script is designed to flash firmware onto an STM32G4 microcontroller, typically used in embedded systems or robotic applications. It seems specifically tailored for use with the `moteus` project by mjbots Robotic Systems, LLC.

### Script Overview:

1. **Imports**: Standard Python modules for system interaction, file handling, and subprocess management.

2. **Constants**:
   - `BINPREFIX`: Determines the prefix for the `objcopy` command based on the machine's architecture.
   - `OBJCOPY`: Command for the GNU `objcopy` utility, used to copy and translate object files.
   - `OPENOCD`: Command string for OpenOCD (Open On-Chip Debugger), configured for an ST-Link interface and an STM32G4 target.

3. **Main Function**:
   - Parses command-line arguments using `argparse`.
   - Optional `--erase` argument to perform a mass erase before programming.
   - Accepts ELF (Executable and Linkable Format) file paths for the `moteus` firmware and bootloader.
   - Creates a temporary directory for intermediate files.
   - Uses `objcopy` to extract and create binary files from specified sections of the ELF files.
   - Invokes OpenOCD to program the STM32G4 microcontroller with the generated binaries.

### Detailed Process:

1. **Argument Parsing**:
   - Provides an interface to specify the ELF files for the `moteus` firmware and bootloader. Defaults are provided if no arguments are specified.

2. **Binary File Creation**:
   - Extracts specific sections (`.isr_vector`, `.text`, `.ARM.extab`, `.ARM.exidx`, `.data`, `.bss`, `.ccmram`) from the ELF files and converts them into binary format using `objcopy`. The sections are chosen based on the memory layout of the target microcontroller.

3. **Flashing with OpenOCD**:
   - Initializes OpenOCD with specified configurations for the debugger interface and microcontroller target.
   - If the `--erase` option is used, performs a mass erase of the microcontroller.
   - Programs the microcontroller with the binary files at specific memory addresses (`0x8000000`, `0x800c000`, `0x08010000`).
   - Verifies the programming and performs a system reset to start the microcontroller with the new firmware.

### Usage:

This script is likely used by developers or firmware engineers working on the `moteus` project to upload new firmware versions to the STM32G4 microcontroller. It automates the process of extracting the necessary binary sections from the ELF files and programming them onto the microcontroller. 

### Conclusion:

The script is a practical tool for firmware deployment in embedded systems, particularly for projects using STM32 microcontrollers. It simplifies the process of firmware updates, ensuring that the correct sections of the firmware are placed in the appropriate memory regions of the microcontroller.
import argparse
import platform
import subprocess
import sys
import tempfile


BINPREFIX = '' if platform.machine().startswith('arm') else 'arm-none-eabi-'

OBJCOPY = BINPREFIX + 'objcopy'
OPENOCD = 'openocd -f interface/stlink.cfg -f target/stm32g4x.cfg '


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--erase', action='store_true')
    parser.add_argument(
        'elffile', nargs='?',
        default='bazel-out/stm32g4-opt/bin/fw/moteus.elf')
    parser.add_argument(
        'bootloader', nargs='?',
        default='bazel-out/stm32g4-opt/bin/fw/can_bootloader.elf')

    args = parser.parse_args()

    tmpdir = tempfile.TemporaryDirectory()

    moteus_elffile = args.elffile
    bootloader_elffile = args.bootloader

    subprocess.check_call(
        f'{OBJCOPY} -Obinary ' +
        f'-j .isr_vector ' +
        f'{moteus_elffile} {tmpdir.name}/out.08000000.bin',
        shell=True)

    subprocess.check_call(
        f'{OBJCOPY} -Obinary ' +
        f'-j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss ' +
        f'{bootloader_elffile} {tmpdir.name}/out.0800c000.bin',
        shell=True)

    subprocess.check_call(
        f'{OBJCOPY} -Obinary ' +
        f'-j .text -j .ARM.extab -j .ARM.exidx -j .data -j .ccmram -j .bss ' +
        f'{moteus_elffile} {tmpdir.name}/out.08010000.bin',
        shell=True)

    subprocess.check_call(
        f'{OPENOCD} -c "init" ' +
        f'-c "reset_config none separate; ' +
        f' halt; ' +
        (f' stm32l4x mass_erase 0; ' if args.erase else '') +
        f' program {tmpdir.name}/out.08000000.bin verify 0x8000000; ' +
        f' program {tmpdir.name}/out.0800c000.bin verify 0x800c000; ' +
        f' program {tmpdir.name}/out.08010000.bin verify  ' +
        f' reset exit 0x08010000"',
        shell=True)


if __name__ == '__main__':
    main()
