# GNU/Linux command line utility for OTA-UART example

This tool is used to transmit any compiled ESP32 image via a UART interface to the corresponding OTA software.

# Building

Compiled with gcc without further dependencies.
Install gcc on any Debian based system with 
`sudo apt install build-essentials gcc`

Compile with:

`gcc update.c -o update`


# Usage

`./update <ttyPort> <filename>`

e.g.,

`./update /dev/ttyUSB0 /home/FLipMouse/firmware.bin`
