# COSGC-ROBOT

Project scope
This directory contains hardware, firmware, and experimental code for the COSGC robotics projects. The repository holds multiple platform variants (e.g. , ) and helper tools for hardware testing, sensors, motor controllers, and prototyping.

Languages & Tools
- Primary firmware is C++ (Arduino / PlatformIO compatible) and shell scripts for build automation.
- Hardware design artifacts (PCBs, schematics) and test documentation may be present in subfolders.

Typical dependencies
- PlatformIO or Arduino toolchain for microcontroller builds
- Wiring and motor-control libraries depending on the platform (see subfolder READMEs)

Where to start
- Open the  folder and read the subfolder  entries (COSGCmega/COSGCmini) for platform-specific wiring and build instructions.

Notes
- This directory is experimental — branches may contain WIP code and hardware test scripts.
